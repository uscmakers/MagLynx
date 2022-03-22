
// need to update these pins 1002
const int FL_SERVO_PIN = 9;
const int FR_SERVO_PIN = 9;
const int BL_SERVO_PIN = 9;
const int BR_SERVO_PIN = 9;

enum legFlag {fl, fr, bl, br};

void setup() {
  // put your setup code here, to run once:
  // Variable name key:
  //   frontLeft  = fl
  //   frontRight = fr
  //   backLeft   = bl
  //   backRight  = br
  

  
  Servo flServo, 
        frServo, 
        blServo, 
        brServo;

  // update these pins 1002
  int flMagnet = 5, 
      frMagnet = 5, 
      blMagnet = 5, 
      brMagnet = 5;   
  
  // Attach servos to servo pins
  flServo.attach(FL_SERVO_PIN);
  frServo.attach(FR_SERVO_PIN);
  blServo.attach(BL_SERVO_PIN);
  brServo.attach(BR_SERVO_PIN);

  // Set pins as output pins for magnets/solenoids
  pinMode(flMagnet, OUTPUT);
  pinMode(frMagnet, OUTPUT);
  pinMode(blMagnet, OUTPUT);
  pinMode(brMagnet, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  int direction = getDirectionFromBLE();
  // enum legs leg = fr;

  if(direction == 1){   // forward
    
  }
  else if(direction == -1){   // backward

  }
  
}

// returns: direction: 1 = forward, -1 = backward, 0 = stop
int getDirectionFromBLE(){    
  // call a lot of BLE functions
}
