#include <Arduino.h>

const int ledPin = 9;  // Change this to the pin number you want to control

void setup() {
  pinMode(ledPin, OUTPUT);  // Initialize the LED pin as an output
  Serial.begin(9600);       // Start serial communication at 9600 baud
}

void loop() {
  if (Serial.available() > 0) {   // Check if data is available to read
    String input = Serial.readStringUntil('\n');  // Read the incoming data until newline character
    input.trim();  // Remove leading and trailing whitespace

    // Check if the input matches the specified phrase
    if (input.equals("off")) { // Replace "YOUR_SPECIFIC_PHRASE" with the phrase you want to listen for
      digitalWrite(ledPin, LOW);  // Turn off the LED by setting the pin LOW
      Serial.println("Port closed.");  // Send a message indicating that the port is closed
    }
    if (input.equals("on")) { // Replace "YOUR_SPECIFIC_PHRASE" with the phrase you want to listen for
      digitalWrite(ledPin, HIGH);  // Turn off the LED by setting the pin LOW
      Serial.println("Port opened.");  // Send a message indicating that the port is closed
    }
  }
}
