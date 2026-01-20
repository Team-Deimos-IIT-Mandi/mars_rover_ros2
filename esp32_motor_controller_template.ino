/*
 * ESP32-S3 MINI – SINGLE MOTOR PID CONTROLLER (CAN DEBUGGING EDITION)
 * Team Deimos IIT Mandi  
 * 
 * ╔═══════════════════════════════════════════════════════════╗
 * ║  CONFIGURATION REQUIRED - CHANGE THESE FOR EACH WHEEL:    ║
 * ║  1. CAN_ID_MOTOR (line 28)                                ║
 * ║  2. WHEEL_NAME (line 29) - for debug messages             ║
 * ╚═══════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include "driver/pcnt.h"
#include "driver/twai.h"
#include <Adafruit_NeoPixel.h>

// ================== RGB LED CONFIG ==================
#define RGB_LED_PIN    6
#define NUM_PIXELS     2
#define LED_BRIGHTNESS 50

Adafruit_NeoPixel statusLed(NUM_PIXELS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// ================== CAN CONFIG ==================
#define CAN_TX_PIN GPIO_NUM_1
#define CAN_RX_PIN GPIO_NUM_2

// ╔═══════════════════════════════════════════════════════════╗
// ║  ⚠️  CHANGE THIS FOR EACH WHEEL  ⚠️                        ║
// ╚═══════════════════════════════════════════════════════════╝
#define CAN_ID_MOTOR 0x121   // FL=0x124, FR=0x121, BL=0x123, BR=0x122
#define WHEEL_NAME "FR"      // "FL", "FR", "BL", "BR"

// ================== MOTOR CONFIG =================
#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 5
#define PWM_PIN       10
#define DIR_PIN       11
#define PCNT_UNIT     PCNT_UNIT_0
#define ENCODER_PPR   400

// ================== PID CONSTANTS =================
#define KP 0.1418
#define KI 2.683
#define KD 0.0

#define MOTOR_GAIN   9.670
#define MOTOR_OFFSET 350.574

// ================== CONTROL PARAMS =================
#define PWM_FREQ         25000
#define PWM_RESOLUTION   8
#define PWM_MAX          255
#define MIN_PWM_OUTPUT   20

#define RPM_SAMPLE_TIME_MS   50
#define CONTROL_LOOP_TIME_MS 50
#define EMA_ALPHA            0.3
#define RPM_DEADBAND         5.0
#define INTEGRAL_LIMIT       1000.0

#define MAX_CAN_SPEED 1000
#define MAX_RPM       2500
#define CAN_TIMEOUT_MS 500

#define DEBUG_CAN true  // Set to false to reduce serial output

// ================== CAN STRUCT ==================
typedef struct {
  int16_t speed;     // magnitude
  int8_t  direction; // 1 = forward, 0 = reverse
} __attribute__((packed)) motor_cmd_t;

// ================== VARIABLES ==================
float targetRPM = 0.0;
float currentRPM = 0.0;
float filteredRPM = 0.0;

float errorSum = 0.0;
float lastError = 0.0;
bool firstRPM = true;

int currentPWM = 0;

unsigned long lastRPMTime = 0;
unsigned long lastControlTime = 0;
unsigned long lastCANUpdate = 0;
unsigned long lastDebugTime = 0;

// Safety Variables
bool encoderFaultDetected = false;
bool canTimeoutDetected = false;
bool isActiveBraking = false;
unsigned long lastBlinkTime = 0;
bool blinkState = false;

// Statistics
uint32_t canMessagesReceived = 0;
uint32_t canMessagesIgnored = 0;

// ================== FUNCTION DECL ==================
bool initCAN();
void processCAN();
void checkCANTimeout();
void initPCNT();
void measureRPM();
void runPID();
float calculateRPM(int16_t count);
void checkEncoderHealth();
void updateSafetyLED();
void printDebugInfo();

// ================== SETUP ==================
void setup() {
  // 1. SAFETY FIRST: Hard Stop
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW); 

  Serial.begin(115200);
  delay(500);

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.printf("║  MOTOR CONTROLLER [%s] - v2.0          ║\n", WHEEL_NAME);
  Serial.println("║  Team Deimos IIT Mandi                 ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.printf("CAN ID: 0x%03X\n\n", CAN_ID_MOTOR);

  // 2. Initialize LED
  statusLed.begin();
  statusLed.setBrightness(LED_BRIGHTNESS);
  statusLed.show();

  initCAN();

  // 3. Attach PWM
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_PIN, 0);

  initPCNT();

  lastRPMTime = millis();
  lastControlTime = millis();
  lastCANUpdate = millis();
  lastDebugTime = millis();

  Serial.println("✓ Motor Controller Ready");
  Serial.println("[Waiting for CAN commands...]\n");
}

// ================== LOOP ==================
void loop() {
  unsigned long now = millis();

  processCAN();
  checkCANTimeout();
  checkEncoderHealth();
  updateSafetyLED();

  if (now - lastRPMTime >= RPM_SAMPLE_TIME_MS) {
    measureRPM();
    lastRPMTime += RPM_SAMPLE_TIME_MS; 
  }

  if (now - lastControlTime >= CONTROL_LOOP_TIME_MS) {
    runPID();
    lastControlTime += CONTROL_LOOP_TIME_MS; 
  }

  // Print periodic status (every 5 seconds)
  if (DEBUG_CAN && now - lastDebugTime > 5000) {
    printDebugInfo();
    lastDebugTime = now;
  }
}

// ================== SAFETY FUNCTIONS ==================
void checkEncoderHealth() {
  if (abs(currentPWM) > 100 && abs(currentRPM) < 10.0) {
    encoderFaultDetected = true;
  } else {
    encoderFaultDetected = false;
  }
}

void updateSafetyLED() {
  unsigned long now = millis();
  
  if (now - lastBlinkTime > 300) {
    blinkState = !blinkState;
    lastBlinkTime = now;
  }

  uint32_t color = 0;

  if (canTimeoutDetected) {
    if (blinkState) color = statusLed.Color(255, 0, 255); // Purple
    else color = 0;
  }
  else if (encoderFaultDetected) {
    if (blinkState) color = statusLed.Color(255, 160, 0); // Yellow
    else color = 0;
  }
  else if (isActiveBraking) {
    if (blinkState) color = statusLed.Color(50, 250, 50); // Lime
    else color = 0;
  }
  else {
    color = 0; // LED OFF when normal
  }

  statusLed.fill(color);
  statusLed.show();
}

// ================== CAN ==================
bool initCAN() {
  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = {
    .acceptance_code = 0,
    .acceptance_mask = 0xFFFFFFFF,
    .single_filter = true
  };

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("✓ CAN Driver Installed");
    twai_start();
    Serial.println("✓ CAN Started @500kbps");
    return true;
  }
  Serial.println("✗ CAN Driver Install FAILED!");
  return false;
}

void processCAN() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    
    // Check if this message is for us
    if (msg.identifier != CAN_ID_MOTOR) {
      canMessagesIgnored++;
      if (DEBUG_CAN) {
        Serial.printf("⊘ CAN RX [0x%03X]: Not for us (our ID: 0x%03X)\n", 
                      msg.identifier, CAN_ID_MOTOR);
      }
      continue;
    }
    
    if (msg.data_length_code != sizeof(motor_cmd_t)) {
      if (DEBUG_CAN) {
        Serial.printf("✗ CAN RX [0x%03X]: Invalid length %d (expected %d)\n",
                      msg.identifier, msg.data_length_code, sizeof(motor_cmd_t));
      }
      continue;
    }

    motor_cmd_t cmd;
    memcpy(&cmd, msg.data, sizeof(cmd));

    float rpm = ((float)cmd.speed * MAX_RPM) / MAX_CAN_SPEED;
    if (cmd.direction == 0) rpm = -rpm;

    targetRPM = rpm;
    lastCANUpdate = millis();
    canMessagesReceived++;
    
    if (DEBUG_CAN) {
      Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
      Serial.printf("✓ CAN RX [%s] #%lu\n", WHEEL_NAME, canMessagesReceived);
      Serial.printf("   ID: 0x%03X\n", msg.identifier);
      Serial.printf("   Speed: %d\n", cmd.speed);
      Serial.printf("   Direction: %d (%s)\n", cmd.direction, cmd.direction ? "FWD" : "REV");
      Serial.printf("   → Target RPM: %.2f\n", targetRPM);
      Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    }
  }
}

void checkCANTimeout() {
  if (millis() - lastCANUpdate > CAN_TIMEOUT_MS) {
    if (!canTimeoutDetected && DEBUG_CAN) {
      Serial.println("⚠ CAN TIMEOUT: No commands received, stopping motor");
    }
    targetRPM = 0;
    canTimeoutDetected = true;
  } else {
    canTimeoutDetected = false;
  }
}

// ================== ENCODER ==================
void initPCNT() {
  pcnt_config_t ch0 = {
    .pulse_gpio_num = ENCODER_A_PIN,
    .ctrl_gpio_num  = ENCODER_B_PIN,
    .lctrl_mode     = PCNT_MODE_REVERSE,
    .hctrl_mode     = PCNT_MODE_KEEP,
    .pos_mode       = PCNT_COUNT_DEC,
    .neg_mode       = PCNT_COUNT_INC,
    .counter_h_lim  = 32767,
    .counter_l_lim  = -32768,
    .unit           = PCNT_UNIT,
    .channel        = PCNT_CHANNEL_0
  };
  pcnt_unit_config(&ch0);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
}

void measureRPM() {
  int16_t count;
  pcnt_get_counter_value(PCNT_UNIT, &count);
  pcnt_counter_clear(PCNT_UNIT); 

  float rawRPM = calculateRPM(count);

  if (firstRPM) {
    filteredRPM = rawRPM;
    firstRPM = false;
  } else {
    filteredRPM = EMA_ALPHA * rawRPM + (1.0 - EMA_ALPHA) * filteredRPM;
  }
  currentRPM = filteredRPM;
}

float calculateRPM(int16_t count) {
  const int CPR = ENCODER_PPR * 4; 
  return (float)count * 60000.0 / (CPR * RPM_SAMPLE_TIME_MS);
}

// ================== PID ==================
void runPID() {
  if (fabs(targetRPM) < 0.1 && fabs(currentRPM) < 5.0 && fabs(errorSum) < 1.0) {
    ledcWrite(PWM_PIN, 0);
    currentPWM = 0;
    errorSum = 0; 
    isActiveBraking = false;
    return; 
  }

  float error = targetRPM - currentRPM;

  if (fabs(targetRPM) < 0.1 && abs(currentPWM) > 40) {
    isActiveBraking = true;
  } else {
    isActiveBraking = false;
  }

  if (fabs(error) < RPM_DEADBAND) error = 0;

  float p = KP * error;

  bool saturated = (currentPWM >= PWM_MAX && error > 0) || (currentPWM <= -PWM_MAX && error < 0);
  
  if (!saturated) {
    errorSum += error * (CONTROL_LOOP_TIME_MS / 1000.0);
    errorSum = constrain(errorSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  }

  float i = KI * errorSum;
  float d = KD * (error - lastError);

  float ffMag = (fabs(targetRPM) + MOTOR_OFFSET) / MOTOR_GAIN;
  float ff = (targetRPM >= 0) ? ffMag : -ffMag;

  float control = ff + p + i + d;

  if (control >= 0) {
    digitalWrite(DIR_PIN, HIGH);
    currentPWM = (int)control;
  } else {
    digitalWrite(DIR_PIN, LOW);
    currentPWM = (int)fabs(control);
  }

  if (currentPWM > 0 && currentPWM < MIN_PWM_OUTPUT) {
    currentPWM = MIN_PWM_OUTPUT;
  }

  currentPWM = constrain(currentPWM, 0, PWM_MAX);
  ledcWrite(PWM_PIN, currentPWM);

  lastError = error;
}

// ================== DEBUG INFO ==================
void printDebugInfo() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.printf("║  STATUS [%s]                           ║\n", WHEEL_NAME);
  Serial.println("╚════════════════════════════════════════╝");
  Serial.printf("CAN Messages RX:   %lu\n", canMessagesReceived);
  Serial.printf("CAN Messages IGN:  %lu\n", canMessagesIgnored);
  Serial.printf("Target RPM:        %.2f\n", targetRPM);
  Serial.printf("Current RPM:       %.2f\n", currentRPM);
  Serial.printf("PWM Output:        %d\n", currentPWM);
  Serial.printf("Encoder Fault:     %s\n", encoderFaultDetected ? "YES" : "NO");
  Serial.printf("CAN Timeout:       %s\n", canTimeoutDetected ? "YES" : "NO");
  Serial.printf("Uptime:            %lu seconds\n", millis() / 1000);
  Serial.println("════════════════════════════════════════\n");
}
