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

double A_R[4]={16384,8192,4096,2048};
double G_R[4]={131,65.5,32.8,16.4};


/*-------------Configuracion inicial del MPU6050--------------------*/
//Configuracion Acelerometro // Opciones 2G, 4G   8G   o  16G
int Ascale=AFS_2G;

//Configuracion Giroscopio // Set the scale below either 250, 500 ,1000  o 2000
int Gscale=GFS_250DPS;
//Frecuencia de Muestreo
int rate=39; //39 para 200 Hz
//Filtro Pasa-Bajo Digital
int dlpf=0;
int cont=0;

unsigned long time;//Para medir las cuentas internas
int cuentas=0;

void setup(){
    
    MPU.initialize();
    
    MPU.setFullScaleGyroRange(Gscale);
    MPU.setFullScaleAccelRange(Ascale);
    MPU.setRate(rate); // Gyroscope Output Rate / (1 + SMPLRT_DIV) = Sample Rate
    MPU.setDLPFMode(dlpf);
    
    Serial.begin(115200);
    // Verificar Conexion
    //Serial.println("Probando Conexion con IMU...");
    //Serial.println(MPU.testConnection() ? "MPU6050 conectado exitosamente" : "Fallo al conectar MPU6050");
    /*
     use the code below to change accel/gyro offset values
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
    //Serial.println("Inicio");
    //timer = micros(); //Para calcular dt
    */
}

void loop() {
  //Escuchando para cambiar :)  
     if (Serial.available()){
          char key = Serial.read();
          char value= Serial.parseInt();
          cambiarConfiguraciones(key,value);
          cont++;
    }
    
    else{    
      if(MPU.getIntStatus()){//Se ve si hay interrupcion
        //prints time since program started
        obtenerDatos();
        //MPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        //Serial.println("1");
        //Serial.println(MPU.getRate());
        //Serial.println(MPU.getDLPFMode());
      }
    }
    
    /*
    //Para probar las muestras dentro de arduino
    if(time<3000){
      if(MPU.getIntStatus()){//Se ve si hay interrupcion
        //MPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        obtenerDatos();
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
    Serial.print(ax/A_R[Ascale]); Serial.print(" ");
    Serial.print(ay/A_R[Ascale]); Serial.print(" ");
    Serial.print(az/A_R[Ascale]); Serial.print(" ");
    Serial.print(gx/G_R[Gscale]); Serial.print(" ");
    Serial.print(gy/G_R[Gscale]); Serial.print(" ");
    Serial.println(gz/G_R[Gscale]);
}

void cambiarConfiguraciones(char key,int value)
{
  if (key == 'g'){
    switch (value) {
      case 0:
        Gscale=GFS_250DPS;
        MPU.setFullScaleGyroRange(Gscale);
        //Serial.print("RangoGiroscopio: 250dps");
        break;
      case 1:
        Gscale=GFS_500DPS;
        MPU.setFullScaleGyroRange(Gscale);
        //Serial.print("RangoGiroscopio: 500dps");
        break;
      case 2:
        Gscale=GFS_1000DPS;
        MPU.setFullScaleGyroRange(Gscale);
        //Serial.print("RangoGiroscopio: 1000dps");
        break;
      case 3:
        Gscale=GFS_2000DPS;
        MPU.setFullScaleGyroRange(Gscale);
        //Serial.print("RangoGiroscopio: 2000dps");
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
        //Serial.print("RangoAcelerometro: 2G");
        break;
      case 1:
        Ascale=AFS_4G;
        MPU.setFullScaleAccelRange(Ascale);
        //Serial.print("RangoAcelerometro: 4G");
        break;
      case 2:
        Ascale=AFS_8G;
        MPU.setFullScaleAccelRange(Ascale);
        //Serial.print("RangoAcelerometro: 8G");
        break;
      case 3:
        Ascale=AFS_16G;
        MPU.setFullScaleAccelRange(Ascale);
        //Serial.print("RangoAcelerometro: 16G");
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
    dlpf=value;
    MPU.setDLPFMode(dlpf);
  }
  return;
}

