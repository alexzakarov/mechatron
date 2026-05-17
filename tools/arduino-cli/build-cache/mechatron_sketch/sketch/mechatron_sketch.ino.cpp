#include <Arduino.h>
#line 1 "/private/var/folders/x9/jq4h3j1s587_75sds7hk6f400000gn/T/mechatron_sketch/mechatron_sketch.ino"
// Arduino Sketch
// MECHATRON Code Editor

#line 4 "/private/var/folders/x9/jq4h3j1s587_75sds7hk6f400000gn/T/mechatron_sketch/mechatron_sketch.ino"
void setup();
#line 13 "/private/var/folders/x9/jq4h3j1s587_75sds7hk6f400000gn/T/mechatron_sketch/mechatron_sketch.ino"
void loop();
#line 4 "/private/var/folders/x9/jq4h3j1s587_75sds7hk6f400000gn/T/mechatron_sketch/mechatron_sketch.ino"
void setup() {
    // Initialize serial communication
    Serial.begin(9600);

    // Initialize pins
    pinMode(3, OUTPUT);
    pinMode(10, OUTPUT);
}

void loop() {
    // Your code here
    digitalWrite(3, HIGH);
    delay(10000);
    digitalWrite(3, LOW);
    delay(10000);
    digitalWrite(10, HIGH);
    delay(10000);
    digitalWrite(10, LOW);
    delay(10000);
}

