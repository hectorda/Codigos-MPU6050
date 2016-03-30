// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/*
Hardware setup:
 MPU6050 Breakout ------------ Arduino
 5V ------------------------ 5V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
*/


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};


double A_R[4]={-16384,-8192,-4096,-2048};
double G_R[4]={-131,-65.5,-32.8,-16.4};

//Configuracion Acelerometro // Opciones 2G, 4G   8G   o  16G
int Ascale=AFS_2G;

//Configuracion Giroscopio // Set the scale below either 250, 500 ,1000  o 2000
int Gscale=GFS_250DPS; 

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

#define LED_PIN 13
bool blinkState = false;

void setup() {

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);

    // initialize device
    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    //accelgyro.setFullScaleGyroRange(Gscale);
    //accelgyro.setFullScaleAccelRange(Ascale);
    
    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
     accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    */
   
    /*Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

    
}

void loop() {

     //Escuchando para cambiar :)
     if (Serial.available()){
          char key = Serial.read();
          char value= Serial.parseInt();
          changeranges(key,value);
          //Serial.print("Cambios");
          delay(1000);
    }
    
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    
    //Serial.print("Gyro range");Serial.println(accelgyro.getFullScaleGyroRange());
    //Serial.print("Accel range");Serial.println(accelgyro.getFullScaleAccelRange());
    Serial.print(ax/A_R[Ascale]); Serial.print(" ");
    Serial.print(ay/A_R[Ascale]); Serial.print(" ");
    Serial.print(az/A_R[Ascale]); Serial.print(" ");
    Serial.print(gx/G_R[Gscale]); Serial.print(" ");
    Serial.print(gy/G_R[Gscale]); Serial.print(" ");
    Serial.println(gz/G_R[Gscale]);
    //delay(100);
}


void changeranges(char key,int value)
{
  if (key == 'g'){
    switch (value) {
      case 0:
        Gscale=GFS_250DPS;
        accelgyro.setFullScaleGyroRange(Gscale);
        Serial.print("RangoGiroscopio: 250dps");
        break;
      case 1:
        Gscale=GFS_500DPS;
        accelgyro.setFullScaleGyroRange(Gscale);
        Serial.print("RangoGiroscopio: 500dps");
        break;
      case 2:
        Gscale=GFS_1000DPS;
        accelgyro.setFullScaleGyroRange(Gscale);
        Serial.print("RangoGiroscopio: 1000dps");
        break;
      case 3:
        Gscale=GFS_2000DPS;
        accelgyro.setFullScaleGyroRange(Gscale);
        Serial.print("RangoGiroscopio: 2000dps");
        break;
      
      default: 
        // if nothing else matches, do the default
        // default is optional
      break;
    }
  }
  if (key == 'a'){
    switch (value) {
      case 0:
        Ascale=AFS_2G;
        accelgyro.setFullScaleAccelRange(Ascale);
        Serial.print("RangoAcelerometro: 2G");
        break;
      case 1:
        Ascale=AFS_4G;
        accelgyro.setFullScaleAccelRange(Ascale);
        Serial.print("RangoAcelerometro: 4G");
        break;
      case 2:
        Ascale=AFS_8G;
        accelgyro.setFullScaleAccelRange(Ascale);
        Serial.print("RangoAcelerometro: 8G");
        break;
      case 3:
        Ascale=AFS_16G;
        accelgyro.setFullScaleAccelRange(Ascale);
        Serial.print("RangoAcelerometro: 16G");
        break;
      
      default: 
        // if nothing else matches, do the default
        // default is optional
      break;
    }
  }  
  return;
}



