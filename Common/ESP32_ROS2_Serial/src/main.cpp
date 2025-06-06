#include <Arduino.h>

// ESP32 Serial Communication mit ROS2
// Empfängt 4 Integers, sendet 2 Integers zurück

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Serial Communication gestartet");
  Serial.println("Warte auf Daten...");
}

void loop() {
  // Warten auf Start-Marker (0xFF, 0xFE)
  if (Serial.available() >= 2) {
    uint8_t byte1 = Serial.read();
    uint8_t byte2 = Serial.read();
    
    if (byte1 == 0xFF && byte2 == 0xFE) {
      // Start-Marker erkannt, auf Daten warten
      unsigned long timeout = millis() + 1000; // 1 Sekunde Timeout
      
      // Warten auf 16 Bytes Daten + 2 Bytes End-Marker
      while (Serial.available() < 18 && millis() < timeout) {
        delay(1);
      }
      
      if (Serial.available() >= 18) {
        // 4 Integers empfangen (16 Bytes)
        int32_t received_data[4];
        Serial.readBytes((uint8_t*)received_data, 16);
        
        // End-Marker lesen und prüfen
        uint8_t end1 = Serial.read();
        uint8_t end2 = Serial.read();
        
        if (end1 == 0xFD && end2 == 0xFC) {
          // Daten verarbeiten
          Serial.print("Empfangen: ");
          for (int i = 0; i < 4; i++) {
            Serial.print(received_data[i]);
            if (i < 3) Serial.print(", ");
          }
          Serial.println();
          
          // Beispiel-Verarbeitung
          int32_t response_data[2];
          response_data[0] = received_data[0] + received_data[1];  // Summe der ersten beiden
          response_data[1] = received_data[2] * received_data[3];  // Produkt der letzten beiden
          
          // Antwort senden (8 Bytes)
          Serial.write((uint8_t*)response_data, 8);
          
          Serial.print("Gesendet: ");
          Serial.print(response_data[0]);
          Serial.print(", ");
          Serial.println(response_data[1]);
        } else {
          Serial.println("Fehler: Ungültiger End-Marker");
          // Buffer leeren
          while (Serial.available()) {
            Serial.read();
          }
        }
      } else {
        Serial.println("Timeout: Nicht genügend Daten empfangen");
        // Buffer leeren
        while (Serial.available()) {
          Serial.read();
        }
      }
    }
  }
}
