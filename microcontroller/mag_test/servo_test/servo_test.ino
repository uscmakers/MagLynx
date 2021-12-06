/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
    myservo.write(180);
    delay(880);
    myservo.write(95);              // tell servo to go to position in variable 'pos'
    delay(880);                       // waits 15ms for the servo to reach the position
}
