#include <Heltec.h>

unsigned long lastJetsonMsg = 0;
const int JETSON_TIMEOUT = 3000; // 3 seconds silence = Jetson Dead

void setup() {
  Heltec.begin(true /*Display*/, true /*LoRa*/, true /*Serial*/, true /*PAs*/, BAND_433);
  Serial.begin(115200);
}

void loop() {
  // 1. LISTEN TO JETSON
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    lastJetsonMsg = millis();

    // Forward immediately to Base Station
    LoRa.beginPacket();
    LoRa.print("T:" + data); // "T" prefix for Telemetry
    LoRa.endPacket();
    
    // Update local screen
    Heltec.display->clear();
    Heltec.display->drawString(0,0, "JETSON: ALIVE");
    Heltec.display->drawString(0,15, "TX: " + data);
    Heltec.display->display();
  }

  // 2. CHECK IF JETSON IS DEAD
  if (millis() - lastJetsonMsg > JETSON_TIMEOUT) {
    // Jetson has crashed or USB disconnected!
    
    LoRa.beginPacket();
    LoRa.print("ALERT:JETSON_NO_RESP"); // Critical Alert Packet
    LoRa.endPacket();

    Heltec.display->clear();
    Heltec.display->setFont(ArialMT_Plain_16);
    Heltec.display->drawString(0, 10, "JETSON");
    Heltec.display->drawString(0, 30, "NO RESPONSE");
    Heltec.display->display();
    
    delay(500); // Don't spam too fast
  }
}