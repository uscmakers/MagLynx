#include <Servo.h>
Servo myservo;

void setup(){
  myservo.attach(5);
}
void loop() {
  myservo.write(pos);
  delay(15);
}
