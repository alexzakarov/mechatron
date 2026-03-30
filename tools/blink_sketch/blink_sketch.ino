// Arduino Blink - Minimal firmware for testing
// LED on pin 13 (PB5)

#define LED_PIN 13

void setup() {
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);  // LED on
    delay(1000);                   // wait 1 second
    digitalWrite(LED_PIN, LOW);   // LED off
    delay(1000);                   // wait 1 second
}
