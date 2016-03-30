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
#include "MPU6050.h"
#include <math.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif



// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

double angleYZ = 0;
double angleXZ = 0;

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

uint32_t timer;

//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779

double A_R[4]={-16384,-8192,-4096,-2048};
double G_R[4]={-131,-65.5,-32.8,-16.4};

//Angulos
float Acc[2];
float Gy[2];
float Angle[2];


//Configuracion Acelerometro // Opciones 2G, 4G   8G   o  16G
int Ascale=AFS_8G;

//Configuracion Giroscopio // Set the scale below either 250, 500 ,1000  o 2000
int Gscale=GFS_500DPS; 

#define LED_PIN 13
bool blinkState = false;

void setup() {
    
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    
    
    // initialize device
    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    accelgyro.setFullScaleGyroRange(Gscale);
    accelgyro.setFullScaleAccelRange(Ascale);
    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76  -2359 1688  0 0 0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    /*Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */
    Serial.begin(115200);
    timer = micros(); //Para calcular dt    
}

void loop() {
    // read raw accel/gyro measurements from device
    //getAngles();
    getMeasurements();
}

void getMeasurements(){
     accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    
    //Serial.print("Gyro range");Serial.println(accelgyro.getFullScaleGyroRange());
   
      Serial.print(ax/A_R[Ascale]); Serial.print(" ");
      Serial.print(ay/A_R[Ascale]); Serial.print(" ");
      Serial.print(az/A_R[Ascale]); Serial.print(" ");
      Serial.print(gx/G_R[Gscale]); Serial.print(" ");
      Serial.print(gy/G_R[Gscale]); Serial.print(" ");
      Serial.println(gz/G_R[Gscale]);
   
}

void getAngles(){
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      //Se calculan los angulos Y, X respectivamente.
      Acc[1] = atan(-1*(ax/A_R[Ascale])/sqrt(pow((ay/A_R[Ascale]),2) + pow((az/A_R[Ascale]),2)))*RAD_TO_DEG;
      Acc[0] = atan((ay/A_R[Ascale])/sqrt(pow((ax/A_R[Ascale]),2) + pow((az/A_R[Ascale]),2)))*RAD_TO_DEG;
     
     //Aplicar el Filtro Complementario

     double dt=(double)(micros()-timer)/1000000;
     timer = micros();
      
     Angle[0] = 0.98 *(Angle[0]+gx/G_R[Gscale]*dt) + 0.02*Acc[0];
     Angle[1] = 0.98 *(Angle[1]+gy/G_R[Gscale]*dt) + 0.02*Acc[1];
    
     //Mostrar los valores por consola
     Serial.print(Angle[0]); Serial.print(" ");
     Serial.println(Angle[1]);
}


