/*
  Blink
  Turns an LED on for one second, then off for one second, repeatedly.
*/

void setup() {
  // Initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards.
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);   // Turn the LED on
  delay(1000);              // Wait for a second
  digitalWrite(13, LOW);    // Turn the LED off
  delay(1000);              // Wait for a second
}
