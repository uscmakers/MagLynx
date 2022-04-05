/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <bluefruit.h>
#include <Servo.h>

// OTA DFU service
BLEDfu bledfu;

// Uart over BLE service
BLEUart bleuart;

// Function prototypes for packetparser.cpp
uint8_t readPacket (BLEUart *ble_uart, uint16_t timeout);
float   parsefloat (uint8_t *buffer);
void    printHex   (const uint8_t * data, const uint32_t numBytes);

// Packet buffer
extern uint8_t packetbuffer[];

const int FL_SERVO_PIN = 5;
const int FR_SERVO_PIN = 6;
const int BL_SERVO_PIN = 7;
const int BR_SERVO_PIN = 8;

enum legFlag {fl, fr, bl, br};

Servo flServo,
      frServo,
      blServo,
      brServo;

int flSolenoid = 1,
    frSolenoid = 2,
    blSolenoid = 3,
    brSolenoid = 4;

boolean isUpright = true;
uint8_t buttNum = 0;
boolean pressed = false;

void setup(void)
{
  flServo.attach(FL_SERVO_PIN);
  frServo.attach(FR_SERVO_PIN);
  blServo.attach(BL_SERVO_PIN);
  brServo.attach(BR_SERVO_PIN);

  pinMode(flSolenoid, OUTPUT);
  pinMode(frSolenoid, OUTPUT);
  pinMode(blSolenoid, OUTPUT);
  pinMode(brSolenoid, OUTPUT);

  flServo.write(90);
  frServo.write(90);
  blServo.write(90);
  brServo.write(90);
      
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println(F("Adafruit Bluefruit52 Controller App Example"));
  Serial.println(F("-------------------------------------------"));

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and start the BLE Uart service
  bleuart.begin();

  // Set up and start advertising
  startAdv();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/

void move_forwards(int Solenoid, Servo servo){
    digitalWrite(Solenoid, HIGH);
    delay(50);
    servo.write(servo.read()+45);
    delay(50);
    digitalWrite(Solenoid, LOW);
  }

  void move_backwards(int Solenoid, Servo servo){
    digitalWrite(Solenoid, HIGH);
    delay(50);
    servo.write(servo.read()-45);
    delay(50);
    digitalWrite(Solenoid, LOW);
  }

//move both front legs forward (pick up solenoid, move servo 45 deg forward, drop solenoids)
//move both front legs backwards and both back legs backwards (pick up solenoids, move servos 45 deg backwards, drop solenoids)
//move both back legs forward (pick up solenoid, move servo 45 deg forward, drop solenoids)

void loop(void)
{
  /*
  if (pressed) {
    Serial.print("entered loop, button pressed. isUpright=");
    Serial.print(isUpright);
    //Serial.print(", buttNum=");
    Serial.println(buttNum);
 
  } else {
    //Serial.print("entered loop, button not pressed. isUpright=");
    Serial.print(isUpright);
    //Serial.print(", buttNum=");    
    Serial.println(buttNum);
  }
  */


    if(buttNum == 1 && pressed){    // 1 button
      digitalWrite(flSolenoid, HIGH);
      flServo.write(flServo.read()+45);
    }else if(buttNum == 1 && !pressed){
      digitalWrite(flSolenoid, LOW);
    }

    if(buttNum == 2 && pressed){    // 2 button
      digitalWrite(frSolenoid, HIGH);
      frServo.write(frServo.read()+45);
    }else if(buttNum == 2 && !pressed){
      digitalWrite(frSolenoid, LOW);
    }

    if(buttNum == 3 && pressed){    // 3 button
      digitalWrite(blSolenoid, HIGH);
      blServo.write(blServo.read()+45);
    }else if(buttNum == 3 && !pressed){
      digitalWrite(blSolenoid, LOW);
    }

    if(buttNum == 4 && pressed){    // 4 button
      digitalWrite(brSolenoid, HIGH);
      brServo.write(brServo.read()+45);
    }else if(buttNum == 1 && !pressed){
      digitalWrite(brSolenoid, LOW);
    }

    //move both front legs forward (pick up solenoid, move servo 45 deg forward, drop solenoids)
    //move both front legs backwards and both back legs backwards (pick up solenoids, move servos 45 deg backwards, drop solenoids)
    //move both back legs forward (pick up solenoid, move servo 45 deg forward, drop solenoids)

    if(buttNum == 5 && pressed && isUpright){    // up button
      //move forward
      //fr & bl solenoid
      //fl & br servos move 90 degrees
        isUpright = false;
        //Serial.println("upButt pressed, moving forward fronts, the backs, then all");
        move_forwards(frSolenoid, frServo);
        delay(50);
        move_forwards(flSolenoid, flServo);
        delay(50);
        flServo.write(flServo.read()-45);
        frServo.write(frServo.read()-45);
        blServo.write(blServo.read()-45);
        brServo.write(brServo.read()-45);
        delay(50);
        move_forwards(blSolenoid, blServo);
        delay(50);
        move_forwards(brSolenoid, brServo);
        isUpright = true;
      delay(1000);
    }
    if(buttNum == 6 && pressed && isUpright){    // down button
      //move backwards
        isUpright = false;
        //Serial.println("down pressed, moving backwards");
        move_backwards(blSolenoid, blServo);
        delay(50);
        move_backwards(brSolenoid, brServo);
        delay(50);
        flServo.write(flServo.read()+45);
        frServo.write(frServo.read()+45);
        blServo.write(blServo.read()+45);
        brServo.write(brServo.read()+45);
        delay(50);
        move_backwards(flSolenoid, flServo);
        delay(50);
        move_backwards(frSolenoid, frServo);
        isUpright = true;
      delay(1000);
    }
    if(buttNum == 7 && pressed){    // left button
      // nothing?
    }
    if(buttNum == 8 && pressed){    // right button
      // nothing?
    }

  // Wait for new data to arrive
  uint8_t len = readPacket(&bleuart, 500);
  if (len == 0) return;

  // Buttons
  if (packetbuffer[1] == 'B') {
    buttNum = packetbuffer[2] - '0';
    pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttNum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }
  }
    
    delay(100);

}
