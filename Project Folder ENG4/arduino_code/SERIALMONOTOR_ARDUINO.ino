#include <Servo.h>

Servo servoPan; // for left/right movement
Servo servoTilt; // for up/down movement

const int panPin = 9;
const int tiltPin = 10;

const int panMinAngle = 80;
const int panMaxAngle = 180;
const int tiltMinAngle = 140;
const int tiltMaxAngle = 90;

int mapValue(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void setup() {
  Serial.begin(9600);
  servoPan.attach(panPin);
  servoTilt.attach(tiltPin);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.equals("l")) {
      servoPan.write(panMinAngle);
    } else if (input.equals("r")) {
      servoPan.write(panMaxAngle);
    } else if (input.equals("d")) {
      servoTilt.write(tiltMinAngle);
    } else if (input.equals("u")) {
      servoTilt.write(tiltMaxAngle);
    }
  }
}
