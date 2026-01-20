/*
 * ESP32 HOST - SERIAL to CAN BRIDGE (4 MOTOR MASTER)
 * Team Deimos IIT Mandi
 * 
 * FUNCTION: Receives velocity commands from ROS2 via Serial
 *           Translates to CAN messages for 4 motor controllers
 * 
 * CONNECTIONS:
 * - USB Serial: Connected to ROS2 computer
 * - CAN TX: GPIO 5
 * - CAN RX: GPIO 4
 */

#include <Arduino.h>
#include "driver/twai.h"

// ================= CAN CONFIG =================
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

// CAN IDs for each motor controller
#define CAN_ID_FL 0x124  // Front Left
#define CAN_ID_FR 0x121  // Front Right
#define CAN_ID_BL 0x123  // Back/Rear Left
#define CAN_ID_BR 0x122  // Back/Rear Right

#define MAX_CMD 1000
#define DEBUG_ENABLED true  // Set to false to reduce serial output

// ================= SERIAL PROTOCOL =================
// Matches ROS2 hardware interface CommandPacket (18 bytes)
struct CommandPacket {
  uint8_t header = 0xA5;
  float fl_vel;  // rad/s
  float rl_vel;  // rad/s
  float fr_vel;  // rad/s
  float rr_vel;  // rad/s
  uint8_t terminator = 0x5A;
} __attribute__((packed));

// Feedback to ROS2 (34 bytes) - Optional
struct FeedbackPacket {
  uint8_t header = 0xA5;
  float fl_pos;
  float fl_vel;
  float rl_pos;
  float rl_vel;
  float fr_pos;
  float fr_vel;
  float rr_pos;
  float rr_vel;
  uint8_t terminator = 0x5A;
} __attribute__((packed));

// ================= CAN MOTOR COMMAND =================
typedef struct {
  int16_t speed;     // magnitude (0..1000)
  int8_t  direction; // 1 = forward, 0 = reverse
} __attribute__((packed)) motor_cmd_t;

// ================= VARIABLES =================
uint8_t serialBuffer[128];
int bufferIndex = 0;
unsigned long lastCommandTime = 0;
unsigned long lastDebugTime = 0;

// Statistics
uint32_t packetsReceived = 0;
uint32_t canMessagesSent = 0;
uint32_t canErrors = 0;

// ================= FUNCTION DECLARATIONS =================
void setupCAN();
void processSerial();
void sendMotorCAN(uint32_t id, float velocity_rad_s);
void printDebugInfo();

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  ESP32 SERIAL-CAN BRIDGE v2.0         ║");
  Serial.println("║  Team Deimos IIT Mandi                 ║");
  Serial.println("╚════════════════════════════════════════╝\n");

  // Setup CAN Bus
  setupCAN();
  
  Serial.println("✓ Ready to receive serial commands from ROS2");
  Serial.println("✓ CAN bus active - listening for motor feedback");
  Serial.println("\n[Waiting for ROS2 commands...]\n");
  
  lastDebugTime = millis();
}

// ================= MAIN LOOP =================
void loop() {
  processSerial();
  
  // Print periodic status (every 5 seconds)
  if (DEBUG_ENABLED && millis() - lastDebugTime > 5000) {
    printDebugInfo();
    lastDebugTime = millis();
  }
  
  // Check for timeout (no commands for 1 second = safety stop)
  if (millis() - lastCommandTime > 1000 && lastCommandTime != 0) {
    // Send zero velocity to all motors
    sendMotorCAN(CAN_ID_FL, 0.0);
    sendMotorCAN(CAN_ID_FR, 0.0);
    sendMotorCAN(CAN_ID_BL, 0.0);
    sendMotorCAN(CAN_ID_BR, 0.0);
    
    if (DEBUG_ENABLED) {
      Serial.println("⚠ TIMEOUT: No commands received, motors stopped");
    }
    lastCommandTime = 0; // Reset to prevent repeated messages
  }
}

// ================= CAN SETUP =================
void setupCAN() {
  twai_general_config_t g =
    TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = {0, 0xFFFFFFFF, true};

  if (twai_driver_install(&g, &t, &f) == ESP_OK) {
    Serial.println("✓ CAN Driver Installed");
  } else {
    Serial.println("✗ CAN Driver Install FAILED!");
    while(1) delay(1000); // Halt
  }
  
  if (twai_start() == ESP_OK) {
    Serial.println("✓ CAN Started @500kbps");
  } else {
    Serial.println("✗ CAN Start FAILED!");
    while(1) delay(1000); // Halt
  }
}

// ================= SERIAL PROCESSING =================
void processSerial() {
  while (Serial.available() > 0) {
    uint8_t byte = Serial.read();
    
    // Look for packet header
    if (byte == 0xA5 && bufferIndex == 0) {
      serialBuffer[bufferIndex++] = byte;
    }
    // Continue filling buffer
    else if (bufferIndex > 0 && bufferIndex < sizeof(CommandPacket)) {
      serialBuffer[bufferIndex++] = byte;
      
      // Check if we have a complete packet
      if (bufferIndex == sizeof(CommandPacket)) {
        CommandPacket* cmd = (CommandPacket*)serialBuffer;
        
        // Validate terminator
        if (cmd->terminator == 0x5A) {
          packetsReceived++;
          lastCommandTime = millis();
          
          if (DEBUG_ENABLED) {
            Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            Serial.printf("📦 RX Serial Packet #%lu\n", packetsReceived);
            Serial.printf("   FL: %.3f rad/s\n", cmd->fl_vel);
            Serial.printf("   RL: %.3f rad/s\n", cmd->rl_vel);
            Serial.printf("   FR: %.3f rad/s\n", cmd->fr_vel);
            Serial.printf("   RR: %.3f rad/s\n", cmd->rr_vel);
          }
          
          // Send to CAN bus
          sendMotorCAN(CAN_ID_FL, cmd->fl_vel);
          sendMotorCAN(CAN_ID_BL, cmd->rl_vel);
          sendMotorCAN(CAN_ID_FR, cmd->fr_vel);
          sendMotorCAN(CAN_ID_BR, cmd->rr_vel);
          
          if (DEBUG_ENABLED) {
            Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
          }
        } else {
          if (DEBUG_ENABLED) {
            Serial.printf("✗ Invalid packet terminator: 0x%02X (expected 0x5A)\n", cmd->terminator);
          }
        }
        
        // Reset buffer
        bufferIndex = 0;
      }
    }
    // Invalid sequence, reset
    else {
      bufferIndex = 0;
    }
  }
}

// ================= CAN TRANSMISSION =================
void sendMotorCAN(uint32_t id, float velocity_rad_s) {
  motor_cmd_t cmd;
  
  // Convert rad/s to RPM (assuming wheel radius and gear ratio handled by motor controller)
  // For now, scale to -1000 to +1000 range
  // Adjust this scaling based on your motor controller expectations
  int16_t value = (int16_t)constrain(velocity_rad_s * 100, -MAX_CMD, MAX_CMD);
  
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

// ================= DEBUG INFO =================
void printDebugInfo() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║         SYSTEM STATUS                  ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.printf("Serial Packets RX: %lu\n", packetsReceived);
  Serial.printf("CAN Messages TX:   %lu\n", canMessagesSent);
  Serial.printf("CAN Errors:        %lu\n", canErrors);
  Serial.printf("Uptime:            %lu seconds\n", millis() / 1000);
  
  // Check CAN bus status
  twai_status_info_t status;
  twai_get_status_info(&status);
  Serial.printf("CAN State:         %s\n", 
                status.state == TWAI_STATE_RUNNING ? "RUNNING" :
                status.state == TWAI_STATE_BUS_OFF ? "BUS OFF" :
                status.state == TWAI_STATE_RECOVERING ? "RECOVERING" : "STOPPED");
  Serial.printf("TX Queue:          %lu\n", status.msgs_to_tx);
  Serial.printf("RX Queue:          %lu\n", status.msgs_to_rx);
  Serial.println("════════════════════════════════════════\n");
}
