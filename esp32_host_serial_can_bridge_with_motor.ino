/*
 * ESP32 HOST - SERIAL to CAN BRIDGE + MOTOR CONTROLLER
 * Team Deimos IIT Mandi
 * 
 * FUNCTION: 
 * - Receives velocity commands from ROS2 via Serial
 * - Controls ONE motor directly (e.g., Front Left)
 * - Sends CAN messages to 3 other motor controllers
 * 
 * ╔═══════════════════════════════════════════════════════════╗
 * ║  CONFIGURATION REQUIRED:                                  ║
 * ║  1. Set which motor THIS ESP controls (line 35-36)        ║
 * ║  2. CAN IDs for the OTHER 3 motors (line 39-41)           ║
 * ╚═══════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include "driver/twai.h"
#include "driver/pcnt.h"
#include <Adafruit_NeoPixel.h>

// ================== RGB LED CONFIG ==================
#define RGB_LED_PIN    6
#define NUM_PIXELS     2
#define LED_BRIGHTNESS 50

Adafruit_NeoPixel statusLed(NUM_PIXELS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// ================== CAN CONFIG =================
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

// ╔═══════════════════════════════════════════════════════════╗
// ║  ⚠️  CONFIGURE WHICH MOTOR THIS ESP CONTROLS  ⚠️           ║
// ╚═══════════════════════════════════════════════════════════╝
#define THIS_MOTOR_INDEX 0    // 0=FL, 1=RL, 2=FR, 3=RR
#define THIS_MOTOR_NAME "FL"  // For debug messages

// CAN IDs for the OTHER 3 motors (don't include this motor's ID)
#define CAN_ID_MOTOR_1 0x121  // FR
#define CAN_ID_MOTOR_2 0x123  // BL  
#define CAN_ID_MOTOR_3 0x122  // BR

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

#define DEBUG_ENABLED true

// ================== SERIAL PROTOCOL =================
struct CommandPacket {
  uint8_t header = 0xA5;
  float fl_vel;  // rad/s
  float rl_vel;  // rad/s
  float fr_vel;  // rad/s
  float rr_vel;  // rad/s
  uint8_t terminator = 0x5A;
} __attribute__((packed));

// ================== CAN MOTOR COMMAND =================
typedef struct {
  int16_t speed;     // magnitude (0..1000)
  int8_t  direction; // 1 = forward, 0 = reverse
} __attribute__((packed)) motor_cmd_t;

// ================== VARIABLES =================
// Serial
uint8_t serialBuffer[128];
int bufferIndex = 0;
unsigned long lastCommandTime = 0;
unsigned long lastDebugTime = 0;
uint32_t packetsReceived = 0;
uint32_t canMessagesSent = 0;
uint32_t canErrors = 0;

// Motor control
float targetRPM = 0.0;
float currentRPM = 0.0;
float filteredRPM = 0.0;
float errorSum = 0.0;
float lastError = 0.0;
bool firstRPM = true;
int currentPWM = 0;
unsigned long lastRPMTime = 0;
unsigned long lastControlTime = 0;

// Safety
bool encoderFaultDetected = false;
bool canTimeoutDetected = false;
bool isActiveBraking = false;
unsigned long lastBlinkTime = 0;
bool blinkState = false;

// Store all 4 motor velocities
float motorVelocities[4] = {0.0, 0.0, 0.0, 0.0};

// ================== FUNCTION DECLARATIONS =================
void setupCAN();
void processSerial();
void sendMotorCAN(uint32_t id, float velocity_rad_s);
void initPCNT();
void measureRPM();
void runPID();
float calculateRPM(int16_t count);
void checkEncoderHealth();
void updateSafetyLED();
void printDebugInfo();

// ================== SETUP =================
void setup() {
  // Safety first
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  ESP32 HOST + MOTOR CONTROLLER        ║");
  Serial.println("║  Team Deimos IIT Mandi                 ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.printf("This ESP controls: %s motor\n", THIS_MOTOR_NAME);
  Serial.printf("CAN IDs for others: 0x%03X, 0x%03X, 0x%03X\n\n", 
                CAN_ID_MOTOR_1, CAN_ID_MOTOR_2, CAN_ID_MOTOR_3);

  // Initialize LED
  statusLed.begin();
  statusLed.setBrightness(LED_BRIGHTNESS);
  statusLed.show();

  // Setup CAN
  setupCAN();
  
  // Setup motor control
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(PWM_PIN, 0);
  initPCNT();

  lastRPMTime = millis();
  lastControlTime = millis();
  lastDebugTime = millis();
  
  Serial.println("✓ Ready to receive serial commands from ROS2");
  Serial.println("✓ CAN bus active for 3 remote motors");
  Serial.println("✓ Local motor controller active");
  Serial.println("\n[Waiting for ROS2 commands...]\n");
}

// ================== MAIN LOOP =================
void loop() {
  unsigned long now = millis();

  processSerial();
  
  // Motor control timing
  if (now - lastRPMTime >= RPM_SAMPLE_TIME_MS) {
    measureRPM();
    lastRPMTime += RPM_SAMPLE_TIME_MS;
  }

  if (now - lastControlTime >= CONTROL_LOOP_TIME_MS) {
    runPID();
    lastControlTime += CONTROL_LOOP_TIME_MS;
  }

  checkEncoderHealth();
  updateSafetyLED();
  
  // Debug output
  if (DEBUG_ENABLED && now - lastDebugTime > 5000) {
    printDebugInfo();
    lastDebugTime = now;
  }
  
  // Safety timeout
  if (now - lastCommandTime > 1000 && lastCommandTime != 0) {
    targetRPM = 0.0;
    sendMotorCAN(CAN_ID_MOTOR_1, 0.0);
    sendMotorCAN(CAN_ID_MOTOR_2, 0.0);
    sendMotorCAN(CAN_ID_MOTOR_3, 0.0);
    
    if (DEBUG_ENABLED) {
      Serial.println("⚠ TIMEOUT: No commands received, all motors stopped");
    }
    lastCommandTime = 0;
  }
}

// ================== CAN SETUP =================
void setupCAN() {
  twai_general_config_t g =
    TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = {0, 0xFFFFFFFF, true};

  if (twai_driver_install(&g, &t, &f) == ESP_OK) {
    Serial.println("✓ CAN Driver Installed");
  } else {
    Serial.println("✗ CAN Driver Install FAILED!");
    while(1) delay(1000);
  }
  
  if (twai_start() == ESP_OK) {
    Serial.println("✓ CAN Started @500kbps");
  } else {
    Serial.println("✗ CAN Start FAILED!");
    while(1) delay(1000);
  }
}

// ================== SERIAL PROCESSING =================
void processSerial() {
  while (Serial.available() > 0) {
    uint8_t byte = Serial.read();
    
    if (byte == 0xA5 && bufferIndex == 0) {
      serialBuffer[bufferIndex++] = byte;
    }
    else if (bufferIndex > 0 && bufferIndex < sizeof(CommandPacket)) {
      serialBuffer[bufferIndex++] = byte;
      
      if (bufferIndex == sizeof(CommandPacket)) {
        CommandPacket* cmd = (CommandPacket*)serialBuffer;
        
        if (cmd->terminator == 0x5A) {
          packetsReceived++;
          lastCommandTime = millis();
          
          // Store all velocities
          motorVelocities[0] = cmd->fl_vel;
          motorVelocities[1] = cmd->rl_vel;
          motorVelocities[2] = cmd->fr_vel;
          motorVelocities[3] = cmd->rr_vel;
          
          if (DEBUG_ENABLED) {
            Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            Serial.printf("📦 RX Serial Packet #%lu\n", packetsReceived);
            Serial.printf("   FL: %.3f | RL: %.3f | FR: %.3f | RR: %.3f rad/s\n",
                         cmd->fl_vel, cmd->rl_vel, cmd->fr_vel, cmd->rr_vel);
          }
          
          // Set target for THIS motor
          targetRPM = motorVelocities[THIS_MOTOR_INDEX] * 100.0; // Scale as needed
          
          if (DEBUG_ENABLED) {
            Serial.printf("   → Local motor [%s]: %.2f RPM\n", THIS_MOTOR_NAME, targetRPM);
          }
          
          // Send to OTHER 3 motors via CAN
          // Map indices to CAN IDs based on THIS_MOTOR_INDEX
          if (THIS_MOTOR_INDEX == 0) { // This is FL
            sendMotorCAN(CAN_ID_MOTOR_2, motorVelocities[1]); // RL
            sendMotorCAN(CAN_ID_MOTOR_1, motorVelocities[2]); // FR
            sendMotorCAN(CAN_ID_MOTOR_3, motorVelocities[3]); // RR
          } else if (THIS_MOTOR_INDEX == 1) { // This is RL
            sendMotorCAN(CAN_ID_MOTOR_1, motorVelocities[0]); // FL
            sendMotorCAN(CAN_ID_MOTOR_1, motorVelocities[2]); // FR
            sendMotorCAN(CAN_ID_MOTOR_3, motorVelocities[3]); // RR
          } else if (THIS_MOTOR_INDEX == 2) { // This is FR
            sendMotorCAN(CAN_ID_MOTOR_1, motorVelocities[0]); // FL
            sendMotorCAN(CAN_ID_MOTOR_2, motorVelocities[1]); // RL
            sendMotorCAN(CAN_ID_MOTOR_3, motorVelocities[3]); // RR
          } else { // This is RR
            sendMotorCAN(CAN_ID_MOTOR_1, motorVelocities[0]); // FL
            sendMotorCAN(CAN_ID_MOTOR_2, motorVelocities[1]); // RL
            sendMotorCAN(CAN_ID_MOTOR_1, motorVelocities[2]); // FR
          }
          
          if (DEBUG_ENABLED) {
            Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
          }
        }
        
        bufferIndex = 0;
      }
    }
    else {
      bufferIndex = 0;
    }
  }
}

// ================== CAN TRANSMISSION =================
void sendMotorCAN(uint32_t id, float velocity_rad_s) {
  motor_cmd_t cmd;
  int16_t value = (int16_t)constrain(velocity_rad_s * 100, -MAX_CAN_SPEED, MAX_CAN_SPEED);
  
  if (value >= 0) {
    cmd.speed = value;
    cmd.direction = 1;
  } else {
    cmd.speed = abs(value);
    cmd.direction = 0;
  }

  twai_message_t msg = {};
  msg.identifier = id;
  msg.data_length_code = sizeof(cmd);
  memcpy(msg.data, &cmd, sizeof(cmd));

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(10));
  
  if (result == ESP_OK) {
    canMessagesSent++;
    if (DEBUG_ENABLED) {
      Serial.printf("   ✓ CAN TX [0x%03X]: speed=%d, dir=%d (%.3f rad/s)\n", 
                    id, cmd.speed, cmd.direction, velocity_rad_s);
    }
  } else {
    canErrors++;
    Serial.printf("   ✗ CAN TX FAILED [0x%03X]: Error %d\n", id, result);
  }
}

// ================== ENCODER =================
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

// ================== SAFETY ==================
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

  if (encoderFaultDetected) {
    if (blinkState) color = statusLed.Color(255, 160, 0); // Yellow
    else color = 0;
  }
  else if (isActiveBraking) {
    if (blinkState) color = statusLed.Color(50, 250, 50); // Lime
    else color = 0;
  }
  else {
    color = 0; // OFF when normal
  }

  statusLed.fill(color);
  statusLed.show();
}

// ================== DEBUG ==================
void printDebugInfo() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.printf("║  HOST + MOTOR [%s] STATUS              ║\n", THIS_MOTOR_NAME);
  Serial.println("╚════════════════════════════════════════╝");
  Serial.printf("Serial Packets RX: %lu\n", packetsReceived);
  Serial.printf("CAN Messages TX:   %lu\n", canMessagesSent);
  Serial.printf("CAN Errors:        %lu\n", canErrors);
  Serial.printf("Local Target RPM:  %.2f\n", targetRPM);
  Serial.printf("Local Current RPM: %.2f\n", currentRPM);
  Serial.printf("Local PWM:         %d\n", currentPWM);
  Serial.printf("Encoder Fault:     %s\n", encoderFaultDetected ? "YES" : "NO");
  Serial.printf("Uptime:            %lu seconds\n", millis() / 1000);
  
  twai_status_info_t status;
  twai_get_status_info(&status);
  Serial.printf("CAN State:         %s\n", 
                status.state == TWAI_STATE_RUNNING ? "RUNNING" :
                status.state == TWAI_STATE_BUS_OFF ? "BUS OFF" : "ERROR");
  Serial.println("════════════════════════════════════════\n");
}
