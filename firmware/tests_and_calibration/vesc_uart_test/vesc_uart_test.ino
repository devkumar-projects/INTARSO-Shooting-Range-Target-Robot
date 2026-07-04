#include <Arduino.h>
#include <VescUart.h>
#include <HardwareSerial.h>

// --- UART CONFIGURATION ---
// Using the ESP32's Serial2 port
// Wiring: GPIO 16 (RX2) -> VESC TX | GPIO 17 (TX2) -> VESC RX
HardwareSerial VescSerial(2); 
VescUart vesc;

void setup() {
  // PC monitoring console
  Serial.begin(115200);
  while (!Serial) { delay(10); } 

  Serial.println("=========================================");
  Serial.println("   UNIT TEST: UART COMMUNICATION    ");
  Serial.println("=========================================");

  // Initializing the serial port for the VESC (Baud rate 115200)
  VescSerial.begin(115200, SERIAL_8N1, 16, 17);
  vesc.setSerialPort(&VescSerial);

  Serial.println("[1/3] Connecting to VESC...");
  delay(1000);
}

void loop() {
  Serial.println("\n--- New test iteration ---");

  // TEST 1: READING DATA (Telemetry)
  // Checks whether the RX (ESP) -> TX (VESC) wiring is OK
  if (vesc.getVescValues()) {
    Serial.println("[OK] Data received successfully!");
    Serial.print(" > Battery voltage: "); Serial.print(vesc.data.inpVoltage); Serial.println(" V");
    Serial.print(" > Motor current: "); Serial.print(vesc.data.avgMotorCurrent); Serial.println(" A");
    Serial.print(" > ERPM: "); Serial.println(vesc.data.rpm);
  } else {
    Serial.println("[ERREUR] Unable to read the VESC.");
    Serial.println(" -> Check the TX/RX cable direction.");
    Serial.println(" -> Check that 'UART' is enabled in VESC Tool.");
  }

  // TEST 2: MOTOR COMMAND (Action)
  // Checks whether the TX (ESP) -> RX (VESC) wiring is OK
  Serial.println("[2/3] Motor test: 3% acceleration (Duty Cycle)");
  
  // Sending a small pulse
  for (int i = 0 ; i < 10 ; i++){
    vesc.setDuty(0.2);
    delay(500);
  }
   

  // TEST 3: STOP AND BRAKING
  /*Serial.println("[3/3] Braking test: 5 Amps");
  vesc.setDuty(0); 
  delay(1000);*/
  
  vesc.setDuty(0); // Full stop
  Serial.println("End of test cycle. Pausing for 5 seconds...");
  
  delay(5000); 
}
