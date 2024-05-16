#include <Servo.h>

Servo servoX;
Servo servoY;

void setup() {
  servoX.attach(10); // Attach servoX to pin 9
  servoX.attach(9); // Attach servoX to pin 9
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  if (Serial.available() > 0) {
    int posX = Serial.parseInt();
    int posY = Serial.parseInt();
    
    // Adjust servo positions based on received coordinates
    servoX.write(posX);
    servoY.write(posY);
  }
}
