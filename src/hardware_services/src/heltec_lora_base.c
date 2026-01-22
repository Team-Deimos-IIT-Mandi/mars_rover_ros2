#include <Heltec.h>

void setup() {
  Heltec.begin(true /*Display*/, true /*LoRa*/, true /*Serial*/, true /*PAs*/, BAND_433);
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    Heltec.display->clear();
    
    // Case A: Critical Alert
    if (incoming.startsWith("ALERT")) {
        Heltec.display->setFont(ArialMT_Plain_16);
        Heltec.display->drawString(0, 0, "CRITICAL ERROR");
        Heltec.display->setFont(ArialMT_Plain_10);
        Heltec.display->drawString(0, 25, incoming); // Shows "JETSON_NO_RESP"
    } 
    // Case B: Telemetry (T:lat,lon,v,s,t)
    else if (incoming.startsWith("T:")) {
        String data = incoming.substring(2); // Remove "T:"
        
        // Quick/Dirty parsing by finding commas would go here
        // For now, let's just print the raw string neatly
        Heltec.display->setFont(ArialMT_Plain_10);
        Heltec.display->drawString(0, 0, "Link: ACTIVE");
        Heltec.display->drawString(0, 15, "Data: " + data);
        
        // Example: You could parse 'data' to show:
        // BAT: 12.4V
        // VEL: 0.5 m/s
    }

    Heltec.display->display();
  }
}