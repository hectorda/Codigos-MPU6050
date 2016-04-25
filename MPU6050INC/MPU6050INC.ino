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

MPU6050 MPU;
//MPU6050 MPU(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long start, finished, elapsed;
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

int rate=39;

#define LED_PIN 13
bool blinkState = false;

unsigned long time;//Para medir las cuentas internas
int cuentas=0;

void setup(){
    
    MPU.initialize();
    
    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(MPU.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    MPU.setFullScaleGyroRange(Gscale);
    MPU.setFullScaleAccelRange(Ascale);
    MPU.setRate(rate); // Gyroscope Output Rate / (1 + SMPLRT_DIV) = Sample Rate
    MPU.setDLPFMode(0);
    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76  -2359 1688  0 0 0
    Serial.print(MPU.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(MPU.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(MPU.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(MPU.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(MPU.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(MPU.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
     MPU.setXGyroOffset(220);
    MPU.setYGyroOffset(76);
    MPU.setZGyroOffset(-85);
    */
   
    /*Serial.print(MPU.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(MPU.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(MPU.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(MPU.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(MPU.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(MPU.getZGyroOffset()); Serial.print("\t"); // 0
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
    else{
      if(MPU.getIntStatus()){//Se ve si hay interrupcion
        //prints time since program started
        //obtenerDatos();
        //MPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        //Serial.println("1");
        //Serial.println(MPU.getRate());
        //Serial.println(MPU.getDLPFMode());
        obtenerAngulos();
      }
    }
      
  
  /*
    if(time<1000){
      if(MPU.getIntStatus()){//Se ve si hay interrupcion
        MPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        time = millis();
        cuentas++;
      }      
    }
    else{
        Serial.print("Sample Rate: ");Serial.println(8000/(1+rate));
        Serial.print("Muestras: ");Serial.println(cuentas);
        Serial.print("Tiempo: ");Serial.println(time);
        
        delay(100000);
    }
    */
    
}

void obtenerDatos(){
     MPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // these methods (and a few others) are also available
    //MPU.getAcceleration(&ax, &ay, &az);
    //MPU.getRotation(&gx, &gy, &gz);
    
    //Serial.print("Gyro range");Serial.println(MPU.getFullScaleGyroRange());
   
    Serial.print(ax/A_R[Ascale]); Serial.print(" ");
    Serial.print(ay/A_R[Ascale]); Serial.print(" ");
    Serial.print(az/A_R[Ascale]); Serial.print(" ");
    Serial.print(gx/G_R[Gscale]); Serial.print(" ");
    Serial.print(gy/G_R[Gscale]); Serial.print(" ");
    Serial.println(gz/G_R[Gscale]);
}

void obtenerAngulos(){
      MPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      double Ax,Ay,Az;
      double Gy,Gx,Gz;
      Ax = -1*ax/A_R[Ascale];
      Ay = -1*ay/A_R[Ascale];
      Az = az/A_R[Ascale];
      Gx = -1*gx/G_R[Gscale];
      Gy = -1*gy/G_R[Gscale];
      Gz = gz/G_R[Gscale];
      
      //Se calculan los angulos X, Y respectivamente con la IMU horizontal.
      //Acc[0] = atan((ay/A_R[Ascale])/sqrt(pow((ax/A_R[Ascale]),2) + pow((az/A_R[Ascale]),2)))*RAD_TO_DEG;
      //Acc[1] = atan(-1*(ax/A_R[Ascale])/sqrt(pow((ay/A_R[Ascale]),2) + pow((az/A_R[Ascale]),2)))*RAD_TO_DEG;
      
      //Se calculan los angulos con la IMU vertical.
      Acc[0] = atan(Az/sqrt(pow(Ax,2) + pow(Ay,2)))*RAD_TO_DEG;
      Acc[1] = atan(Ax/sqrt(pow(Az,2) + pow(Ay,2)))*RAD_TO_DEG;
      //Aplicar el Filtro Complementario

      double dt=(double)(micros()-timer)/1000000;
      timer = micros();

      //Angle_compl[1] = 0.98 *(Angle_compl[1]+gy/G_R[Gscale]*dt) + 0.02*Acc[1];
      
      Angle_compl[0] = 0.98 *(Angle_compl[0]+gx/G_R[Gscale]*dt) + 0.02*Acc[0];
      Angle_compl[1] = 0.98 *(Angle_compl[1]+gz/G_R[Gscale]*dt) + 0.02*Acc[1];
      
    
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
        MPU.setFullScaleGyroRange(Gscale);
        Serial.print("RangoGiroscopio: 250dps");
        break;
      case 1:
        Gscale=GFS_500DPS;
        MPU.setFullScaleGyroRange(Gscale);
        Serial.print("RangoGiroscopio: 500dps");
        break;
      case 2:
        Gscale=GFS_1000DPS;
        MPU.setFullScaleGyroRange(Gscale);
        Serial.print("RangoGiroscopio: 1000dps");
        break;
      case 3:
        Gscale=GFS_2000DPS;
        MPU.setFullScaleGyroRange(Gscale);
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
        MPU.setFullScaleAccelRange(Ascale);
        Serial.print("RangoAcelerometro: 2G");
        break;
      case 1:
        Ascale=AFS_4G;
        MPU.setFullScaleAccelRange(Ascale);
        Serial.print("RangoAcelerometro: 4G");
        break;
      case 2:
        Ascale=AFS_8G;
        MPU.setFullScaleAccelRange(Ascale);
        Serial.print("RangoAcelerometro: 8G");
        break;
      case 3:
        Ascale=AFS_16G;
        MPU.setFullScaleAccelRange(Ascale);
        Serial.print("RangoAcelerometro: 16G");
        break;
      
      default: 
        // if nothing else matches, do the default
        // default is optional
      break;
    }
  }
  if (key == 's'){
    rate=value;
    MPU.setRate(rate);
  }
  if (key == 'l'){
    MPU.setDLPFMode(value);
  }
  return;
}

