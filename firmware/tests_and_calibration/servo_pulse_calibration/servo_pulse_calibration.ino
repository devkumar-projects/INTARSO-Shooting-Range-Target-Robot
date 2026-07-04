#include <ESP32Servo.h>

Servo servo;

// Pin the servo is connected to
const int servoPin = 18;

// Calibration variables
int minPulse = 500;   // initial min value (us)
int maxPulse = 2300;  // initial max value (us)
int step = 10;        // increment for testing
int delayTime = 50;  // delay between each position

void setup() {
  Serial.begin(115200);

  // Attach the servo with a wide range for calibration
  servo.attach(servoPin, minPulse, maxPulse);
  Serial.println("Servo calibration: measuring min/max limits");
}

void loop() {
  // Test the min
  Serial.println("Testing min...");
  for (int pulse = minPulse; pulse <= maxPulse; pulse += step) {
    servo.writeMicroseconds(pulse);
    Serial.print("Pulse: ");
    Serial.println(pulse);
    delay(delayTime);
  }

  // Test the max
  Serial.println("Testing max...");
  for (int pulse = maxPulse; pulse >= minPulse; pulse -= step) {
    servo.writeMicroseconds(pulse);
    Serial.print("Pulse: ");
    Serial.println(pulse);
    delay(delayTime);
  }

  Serial.println("Calibration finished. Adjust minPulse and maxPulse based on the servo's actual movement.");
  //while (true); // stop after one test loop
}
