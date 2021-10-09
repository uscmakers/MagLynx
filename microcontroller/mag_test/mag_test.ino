#include <Adafruit_Arcada.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>


Adafruit_Arcada arcada;
Adafruit_LSM6DS33 lsm6ds33;
Adafruit_LIS3MDL lis3mdl;
Adafruit_SHT31 sht30;
Adafruit_APDS9960 apds9960;
Adafruit_BMP280 bmp280;
#define WHITE_LED 43

// millisecond delay between samples
#define DELAY_PER_SAMPLE 150

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  Serial.print("Hello! Arcada CLUE sensor plotter");
  //while (!Serial) yield();
  
  if (!apds9960.begin() || !lsm6ds33.begin_I2C() || !lis3mdl.begin_I2C() || 
      !sht30.begin(0x44) || !bmp280.begin()) {
      Serial.println("Failed to find CLUE sensors!");
      arcada.haltBox("Failed to init CLUE sensors");
  }
 
  pinMode(WHITE_LED, OUTPUT);
  digitalWrite(WHITE_LED, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
    
    sensors_event_t mag;
    lis3mdl.getEvent(&mag);
    float x = mag.magnetic.x;
    float y = mag.magnetic.y;
    float z = mag.magnetic.z;
    char x_buf [sizeof(float)];
    char y_buf [sizeof(float)];
    char z_buf [sizeof(float)];
    memcpy(x_buf,&x,sizeof(float));
    memcpy(y_buf,&y,sizeof(float));
    memcpy(z_buf,&z,sizeof(float));
    Serial.printf("Mag: %f %f %f\n", x, y, z);
}
