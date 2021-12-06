#include <Servo.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS33.h>

//Adafruit_LSM6DS33 lsm6ds33;

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

Servo myservo;
int flag = -1;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  myservo.attach(9); //attaches servo to pin 9
  myservo.write(95);   // sets servo to position 0

  //Serial.begin(115200);

//#if CFG_DEBUG
  // Blocking wait for connection when debug mode is enabled via IDE
  //while ( !Serial ) yield();
//#endif

  //Serial.println("Bluefruit52 BLEUART Example");
  //Serial.println("---------------------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

  //Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
  //Serial.println("Once connected, enter character(s) that you wish to send");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
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

void loop()
{
  // Forward data from HW Serial to BLEUART
  /*while (Serial.available())
  {
    // Delay to wait for enough input, since we have a limited transmission buffer
    delay(2);
  }*/
  
  while (bleuart.available())
  {
    
    // rotate servo "turn" degrees back and forth
    //bleuart only reads int :(
    uint8_t s = 1;
    uint8_t read = bleuart.read();
    //char buf[6] = "servo";

    if (read == 49)
    {
      /*for (pos = 0; pos <= turn; pos += 1) { // goes from 0 degrees to "turn" degrees
        // in steps of 1 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
      }
      */

      /*for (pos = turn; pos >= 0; pos -= 1) { // goes from "turn" degrees to 0 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
      }
      */

      //when flag = -1 servo turns counter clockwise, when flag = 1 servo turns clockwise

      myservo.write(95+flag*45); //turns servo at the fastest speed clockwise (ccw = 180)
      digitalToggle(LED_BUILTIN); //COMMENT OUT LATER
      delay(920);
      myservo.write(95); //stops servo
      digitalWrite(LED_BUILTIN, LOW); //COMMENT OUT LATER
      flag *= -1; //need to test value of delay
      bleuart.write(s);
      //Serial.println(s);
    }
  }
  /*sensors_event_t gyro;
  float x, y, z;
  while ( bleuart.available() )
  {
    lsm6ds33.getEvent(NULL, &gyro, NULL);
    x = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    y = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    z = gyro.gyro.z * SENSORS_RADS_TO_DPS;
    // Storing float in char array for snprintf to work
    char x_buf [sizeof(float)];
    char y_buf [sizeof(float)];
    char z_buf [sizeof(float)];
    memcpy(x_buf,&x,sizeof(float));
    memcpy(y_buf,&y,sizeof(float));
    memcpy(z_buf,&z,sizeof(float));
    char buf[64];
    snprintf (buf, 64, ("Gyro: %g %g %g\n", x_buf, y_buf, z_buf));*/
    /*uint8_t ch;
    ch = (uint8_t) bleuart.read();
    Serial.write(ch);
    //Serial.write(buf);
  }*/
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  //Serial.print("Connected to ");
  //Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  //Serial.println();
  //Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}
