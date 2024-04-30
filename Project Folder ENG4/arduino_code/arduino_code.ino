#include <Servo.h>
#include <Arduino.h>


#define PAN_PIN 9
#define TILT_PIN 10

Servo panServo;
Servo tiltServo;

void setup() {
  Serial.begin(9600);

  // Attach servos to pins
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  // Set servos to initial position (you may need to adjust these angles)
  panServo.write(65);
  tiltServo.write(50);

  // Wait for servos to reach initial position
  delay(500);
}

void loop() {
  if (Serial.available()) {
    // Read incoming command from serial port
    char command = Serial.read();

    // If 'r' is received, read and print current servo positions
    if (command == 'r') {
      int panAngle = panServo.read();
      int tiltAngle = tiltServo.read();

      Serial.print("Pan Angle: ");
      Serial.print(panAngle);
      Serial.print(", Tilt Angle: ");
      Serial.println(tiltAngle);
    }
  }
// tilt min 167
//tilt max 20
  // Add any other code or functionality here if needed
}
