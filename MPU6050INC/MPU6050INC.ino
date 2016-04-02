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

uint32_t timer;

//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779

double A_R[4]={-16384,-8192,-4096,-2048};
double G_R[4]={-131,-65.5,-32.8,-16.4};

//Angulos
float Acc[2];
float Angle_compl[2];



//Configuracion Acelerometro // Opciones 2G, 4G   8G   o  16G
int Ascale=AFS_2G;

//Configuracion Giroscopio // Set the scale below either 250, 500 ,1000  o 2000
int Gscale=GFS_250DPS; 

#define LED_PIN 13
bool blinkState = false;

void setup() {
    
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
    Serial.begin(115200);
    //Serial.println("Inicio");
    timer = micros(); //Para calcular dt   
}

void loop() {

      //Escuchando para cambiar :)
     if (Serial.available()){
          char key = Serial.read();
          char value= Serial.parseInt();
          changeranges(key,value);
          //Serial.print("Cambios");
          //delay(1000);
    }
    // read raw accel/gyro measurements from device
    getAngles();
    //getMeasurements();
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
      //Se calculan los angulos X, Y respectivamente.
      Acc[0] = atan((ay/A_R[Ascale])/sqrt(pow((ax/A_R[Ascale]),2) + pow((az/A_R[Ascale]),2)))*RAD_TO_DEG;
      Acc[1] = atan(-1*(ax/A_R[Ascale])/sqrt(pow((ay/A_R[Ascale]),2) + pow((az/A_R[Ascale]),2)))*RAD_TO_DEG;
      //Aplicar el Filtro Complementario

      double dt=(double)(micros()-timer)/1000000;
      timer = micros();
      
      Angle_compl[0] = 0.98 *(Angle_compl[0]+gx/G_R[Gscale]*dt) + 0.02*Acc[0];
      Angle_compl[1] = 0.98 *(Angle_compl[1]+gy/G_R[Gscale]*dt) + 0.02*Acc[1];
    
     //Mostrar los valores por consola
     Serial.print(Angle_compl[0]); Serial.print(" ");
     Serial.println(Angle_compl[1]);
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

