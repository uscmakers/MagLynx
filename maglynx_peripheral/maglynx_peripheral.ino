void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  // if BLE message received from central
    // turn servo 180
    for (pos = centralServo.read(); pos <= centralServo.read()+180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      centralServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  
    // send message to peripheral
    delay(500)

}
