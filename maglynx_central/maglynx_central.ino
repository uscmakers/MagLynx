#include <Servo.h>
Servo centralServo;
int pos = 0;    // variable to store the servo position
void sendAll(const char* str)
{
  Serial.print("[Send to All]: ");
  Serial.println(str);
  
  for(uint8_t id=0; id < BLE_MAX_CONNECTION; id++)
  {
    prph_info_t* peer = &prphs[id];

    if ( peer->bleuart.discovered() )
    {
      peer->bleuart.print(str);
    }
  }
}
/**
 * Callback invoked when scanner picks up an advertising packet
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Since we configure the scanner with filterUuid()
  // Scan callback only invoked for device with bleuart service advertised
  // Connect to the device with bleuart service in advertising packet
  Bluefruit.Central.connect(report);
}
/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  // Find an available ID to use
  int id  = findConnHandle(BLE_CONN_HANDLE_INVALID);

  // Eeek: Exceeded the number of connections !!!
  if ( id < 0 ) return;

  prph_info_t* peer = &prphs[id];
  peer->conn_handle = conn_handle;

  Bluefruit.Connection(conn_handle)->getPeerName(peer->name, sizeof(peer->name)-1);

  Serial.print("Connected to ");
  Serial.println(peer->name);

  Serial.print("Discovering BLE UART service ... ");

  if ( peer->bleuart.discover(conn_handle) )
  {
    Serial.println("Found it");
    Serial.println("Enabling TXD characteristic's CCCD notify bit");
    peer->bleuart.enableTXD();

    Serial.println("Continue scanning for more peripherals");
    Bluefruit.Scanner.start(0);

    peer->bleuart.print("initElectro()");
    sensors_event_t gyro;
    sensors_event_t mag;
    lsm6ds33.getEvent(NULL, &gyro, NULL);
    lis3mdl.getEvent(&mag);
    float x = mag.magnetic.x;
    float y = mag.magnetic.y;
    float z = mag.magnetic.z;
    // Placeholder code for direction
    // Need to convert form mag strength to xyz using https://digilent.com/blog/how-to-convert-magnetometer-data-into-compass-heading/
    prphs[id].x = mag.magnetic.x;
    prphs[id].y = mag.magnetic.y;
    prphs[id].z = mag.magnetic.z;
    prphs[id].str = x + y + z;
    for (int i = 0; i < sizeof(prphs); i++) {
      // This is assuming we've normalized the direction into xyz coordinates such as (1,1,1) or (0,1,0) rather than floats
      /*if (id != i && x == prphs[i].x && y == prphs[i].y && z == prphs[i].z) {
        // Assumes only 2 peripherals will be in the same direction initially
        if (prphs[id].str < prphs[i].str) {
          prphs[id].x *= 2;
          prphs[id].y *= 2;
          prphs[id].z *= 2;
        } else {
          prphs[i].x *= 2;
          prphs[i].y *= 2;
          prphs[i].z *= 2;
        }
      }*/
      Serial.println(prphs[i].str);
    }
  } else
  {
    Serial.println("Found ... NOTHING!");

    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }

  connection_num++;
}
/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  connection_num--;

  // Mark the ID as invalid
  int id  = findConnHandle(conn_handle);

  // Non-existant connection, something went wrong, DBG !!!
  if ( id < 0 ) return;

  // Mark conn handle as invalid
  prphs[id].conn_handle = BLE_CONN_HANDLE_INVALID;

  Serial.print(prphs[id].name);
  Serial.println(" disconnected!");
}


/**
 * Callback invoked when BLE UART data is received
 * @param uart_svc Reference object to the service where the data
 * arrived.
 */
void bleuart_rx_callback(BLEClientUart& uart_svc)
{
  // uart_svc is prphs[conn_handle].bleuart
  uint16_t conn_handle = uart_svc.connHandle();

  int id = findConnHandle(conn_handle);
  prph_info_t* peer = &prphs[id];

  // Print sender's name
  Serial.printf("[From %s]: ", peer->name);

  // Read then forward to all peripherals
  while ( uart_svc.available() )
  {
    // default MTU with an extra byte for string terminator
    char buf[20+1] = { 0 };

    if ( uart_svc.read(buf,sizeof(buf)-1) )
    {
      // if central clue senses magnetic field via magnetometer,
      // rotate servo 180 degrees back and forth
      
//      if (buf.equals("servo")){
//        for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//          // in steps of 1 degree
//          myservo.write(pos);              // tell servo to go to position in variable 'pos'
//          delay(15);                       // waits 15 ms for the servo to reach the position
//        }
//        for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//          myservo.write(pos);              // tell servo to go to position in variable 'pos'
//          delay(15);                       // waits 15 ms for the servo to reach the position
//        }
//      }
      Serial.println(buf);
      sendAll(buf);
    }
  }
}


void setup() {
  centralServo.attach(9);  // attaches the servo on pin 9 to the servo object
  centralServo.write(pos);   // initialize to position 0


  // starts up bluetooth on central, configures itself, 
  // and turns on ble scanning feature to look for peripherals
  Bluefruit.begin(0, 4);

  // Set Name
  Bluefruit.setName("Bluefruit52 Central");

  // Init peripheral pool
  for (uint8_t idx=0; idx<BLE_MAX_CONNECTION; idx++)
  {
    // Invalid all connection handle
    prphs[idx].conn_handle = BLE_CONN_HANDLE_INVALID;

    // All of BLE Central Uart Serivce
    prphs[idx].bleuart.begin();
    prphs[idx].bleuart.setRxCallback(bleuart_rx_callback);
  }

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);   // sets what happens when you connect to a peripheral
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Filter only accept bleuart service in advertising
   * - Don't use active scan (used to retrieve the optional scan response adv packet)
   * - Start(0) = will scan forever since no timeout is given
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
  Bluefruit.Scanner.filterUuid(BLEUART_UUID_SERVICE);
  Bluefruit.Scanner.useActiveScan(false);       // Don't request scan response data
  Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds

}

void loop() {
  // put your main code here, to run repeatedly:

  // turn servo 180
  for (pos = centralServo.read(); pos <= centralServo.read()+180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    centralServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  
  // send message from peripheral to central
  // Read then forward to all peripherals
  while ( uart_svc.available() )    // while clues are connected(?)
  {
    // default MTU with an extra byte for string terminator
    char buf[20+1] = { 0 };
    
    if ( uart_svc.read(buf,sizeof(buf)-1) )
    {
      Serial.println(buf);
      sendAll(buf);
    }
  }

  // delay 500ms

}
