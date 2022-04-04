#include <Servo.h>

const int SERVO_PIN = 6;
const int SOLENOID_PIN = 10;
Servo servo;

void setup() {
    servo.attach(SERVO_PIN);
    servo.write(0);
    pinMode(SOLENOID_PIN, OUTPUT);
    digitalWrite(SOLENOID_PIN, LOW);
}

void loop() {
    delay(4000);
    digitalWrite(SOLENOID_PIN, HIGH);
    servo.write(180);
    delay(2000);
    digitalWrite(SOLENOID_PIN, LOW);
    servo.write(0);
}
