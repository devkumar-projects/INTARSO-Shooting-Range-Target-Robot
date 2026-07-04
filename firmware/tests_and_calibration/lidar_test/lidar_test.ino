#include <HardwareSerial.h>

HardwareSerial LidarSerial(2);

#define RX_PIN 14
#define TX_PIN 27

void setup() {
  Serial.begin(115200); 
  
  LidarSerial.begin(921600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  Serial.println("Lidar Waveshare (D) - High-Speed Mode 921600");
}

void loop() {
  static uint8_t buffer[16];
  static int index = 0;

  while (LidarSerial.available()) {
    uint8_t b = LidarSerial.read();

    if (index == 0 && b != 0x57) 
    if (index == 1) {
       if (b != 0x00 && b != 0x01) { 
         index = 0; 
         continue;
       }
    }
    
    buffer[index++] = b;

    if (index == 16) {
      uint8_t checksum = 0;
      for (int i = 0; i < 15; i++) checksum += buffer[i];

      if (checksum == buffer[15]) {
       
        uint32_t dist_mm = (uint32_t)(buffer[9] << 8) | buffer[8];
     
        uint16_t rssi = (buffer[11] << 8) | buffer[10];

        if (dist_mm > 0 && dist_mm < 50000) { 
          Serial.print("Distance : ");
          Serial.print(dist_mm / 1000.0, 3); 
          Serial.print(" m (");
          Serial.print(dist_mm);
          Serial.print(" mm) | Signal : ");
          Serial.println(rssi);
        } else {
          Serial.print("Out of range (");
          Serial.print(dist_mm);
          Serial.println(")");
        }
      }
      index = 0;
    }
  }
}
