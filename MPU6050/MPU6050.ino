/*
Hardware setup:
 MPU6050 Breakout ------------ Arduino
 5V ------------------------ 5V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
*/

#include<Wire.h>

// Registros del Sensor
#define MPU6050 0x68    // Direccion WHO_AM_I MPU-6050
#define PWR_MGMT_1 0x6B // Registro PWR_MGMT_1
#define CONFIG 0x1A     //Registro de Configuracion DLPF y EXT_SYNC_SET

/* -------------------Registros de Acelerometro----------------  */
#define ACCEL_CONFIG 0x1C //Registro Configuracion Acelerometro

#define ACCEL_XOUT_H 0x3B //Registro Medidas X Acelerometro bits 15-8
#define ACCEL_XOUT_L 0x3C //Registro Medidas X Acelerometro bits 7-0
#define ACCEL_YOUT_H 0x3D //Registro Medidas Y Acelerometro bits 15-8
#define ACCEL_YOUT_L 0x3E //Registro Medidas Y Acelerometro bits 7-0
#define ACCEL_ZOUT_H 0x3F //Registro Medidas Z Acelerometro bits 15-8
#define ACCEL_ZOUT_L 0x40 //Registro Medidas Z Acelerometro bits 7-0

/* -------------------Registros del Gyroscopio----------------  */
#define GYRO_CONFIG 0x1B // Registro Configuracion Gyroscopio

#define GYRO_XOUT_H 0x43 //Registro de Medidas X Gyroscopio bits 15-8
#define GYRO_XOUT_L 0x44 //Registro de Medidas X Gyroscopio bits 7-0
#define GYRO_YOUT_H 0x45 //Registro de Medidas Y Gyroscopio bits 15-8
#define GYRO_YOUT_L 0x46 //Registro de Medidas Y Gyroscopio bits 7-0
#define GYRO_ZOUT_H 0x47 //Registro de Medidas Z Gyroscopio bits 15-8
#define GYRO_ZOUT_L 0x48 //Registro de Medidas Z Gyroscopio bits 7-0

#define INT_STATUS 0x3A //Registro de Estado de Interrupcion
#define SMPRT_DIV 0x19 //Registro Sample Rate Divider
#define FIFO_EN 0x23 //Registro FIFO Enable para Definir que medidas se obtendran
#define MPU6050_CLOCK_PLL_XGYRO 0x01

//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
//Variables donde almacenar muestras
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
//Conversion de Acelerometro y Giroscopio
double A_R[4]={16384,8192,4096,2048};
double G_R[4]={131,65.5,32.8,16.4};

//Angulos
float Acc[2];
float Angle_compl[2];

uint32_t timer;

// Set initial input parameters
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


//Configuracion Acelerometro // Opciones 0,   1    2   o  3
int Ascale=AFS_2G;      // Rango  ±2g, ±4g, ±8g o ±16g

//Configuracion Giroscopio // Set the scale below either 0, 1 ,2  o 3
int Gscale=GFS_250DPS;
#define DLPF_CFG 0 //Configurar Low Pass Filter interno
#define SMPLRT_DIV 39 //Divisor de la frecuencia de salida
#define TEMP_FIFO_EN 0 //Desactivar FIFO TEMP
int cont=0;

void setup() {
  Wire.begin();
  Wire.setClock(400);
  Serial.begin(115200);
  inicializarMPU6050();
}

void loop(){

    if (Serial.available()){ //Escuchando para cambiar :)
          char key = Serial.read();
          char value= Serial.parseInt();
          cambiarRangos(key,value);
          cont++;
    }
    if(cont==4){
      if(leerRegistro(INT_STATUS) & 0x01)  //Comprobamos si hay una interrupcion
        //imprimirAngulos();
        imprimirMuestras();      
    }
}

void inicializarMPU6050(){// Inicializa la configuracion de MPU6050
    escribirRegistro(PWR_MGMT_1, 0);  // Se despierta el MPU-6050
    escribirRegistro(CONFIG, DLPF_CFG);//Configurar Filtro
    escribirRegistro(SMPRT_DIV, SMPLRT_DIV);//Configurar Sample Rate
    escribirRegistro(FIFO_EN, TEMP_FIFO_EN<<7);//Desactivar FIFO de Temperatura
    setFullScaleAccelRange();
    setFullScaleGyroRange();   
}

void leerAcelerometro(){
      Wire.beginTransmission(MPU6050);      
      Wire.write(ACCEL_XOUT_H); // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU6050,6,true); //A partir del 0x3B, se piden 6 registros      
      AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
      AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
}

void leerGyroscopio(){
      //Leer los valores del Giroscopio
      Wire.beginTransmission(MPU6050);
      Wire.write(GYRO_XOUT_H);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU6050,6,true);//A partir del 0x43, se piden 6 registros
      GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void leerMuestras(){

    Wire.beginTransmission(MPU6050);      
    Wire.write(ACCEL_XOUT_H); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050,14,true); //A partir del 0x3B, se piden 6 registros      
    AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    double tmp= Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L);
    GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L) 
    /*
     * 
     //------Lectura Acelerometro---------
    AcX=leerRegistro(ACCEL_XOUT_H)<<8 | leerRegistro(ACCEL_XOUT_L);
    AcY=leerRegistro(ACCEL_YOUT_H)<<8 | leerRegistro(ACCEL_YOUT_L);
    AcZ=leerRegistro(ACCEL_ZOUT_H)<<8 | leerRegistro(ACCEL_ZOUT_L);
    //------Lectura Gyroscopio---------
    GyX=leerRegistro(GYRO_XOUT_H)<<8 | leerRegistro(GYRO_XOUT_L);
    GyY=leerRegistro(GYRO_YOUT_H)<<8 | leerRegistro(GYRO_YOUT_L);
    GyZ=leerRegistro(GYRO_ZOUT_H)<<8 | leerRegistro(GYRO_ZOUT_L);
    
    //leerAcelerometro();
    //leerGyroscopio();
    */
  }


void leerAngulos(){
      double Ax,Ay,Az;
      double Gy,Gx,Gz;
      Ax = -1*AcX/A_R[Ascale];
      Ay = -1*AcY/A_R[Ascale];
      Az = AcZ/A_R[Ascale];
      Gx = -1*GyX/G_R[Gscale];
      Gy = -1*GyY/G_R[Gscale];
      Gz = GyZ/G_R[Gscale];
      
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
      
      Angle_compl[0] = 0.98 *(Angle_compl[0]+GyX/G_R[Gscale]*dt) + 0.02*Acc[0];
      Angle_compl[1] = 0.98 *(Angle_compl[1]+GyZ/G_R[Gscale]*dt) + 0.02*Acc[1];
}

void imprimirMuestras(){    
    leerMuestras();

    //---Imprimir Aceletrometro---
    Serial.print(AcX/A_R[Ascale]);Serial.print(" ");
    Serial.print(AcY/A_R[Ascale]);Serial.print(" "); 
    Serial.print(AcZ/A_R[Ascale]);Serial.print(" ");
        
    //---Imprimir Gyroscopio---
    Serial.print(GyX/G_R[Gscale]);Serial.print(" ");
    Serial.print(GyY/G_R[Gscale]);Serial.print(" ");
    Serial.print(GyZ/G_R[Gscale]);Serial.println();
    
    delay(0);
}

void imprimirAngulos(){
    leerMuestras();
    leerAngulos();
    Serial.print(Angle_compl[0]); Serial.print(" ");
    Serial.println(Angle_compl[1]);
  
  }

byte leerRegistro(byte Registro){// Leer el Byte de un Registro de 8 bits
  Wire.beginTransmission(MPU6050);    //Se inicializa el Tx buffer
  Wire.write(Registro);               //Se pone el registro en el buffer
  Wire.endTransmission(false);        //Se deja de transmitir pero se continua con la coneccion
  Wire.requestFrom(MPU6050, 1,true);  //El maestro pide 1 Byte
  byte lectura=Wire.read();           //Se lee el dato desde el registro
  return lectura;                     //Se retorna el Byte
}

// Writes a single byte (dataToWrite) into addressToWrite
void escribirRegistro(byte Registro, byte Dato) {
  Wire.beginTransmission(MPU6050);
  Wire.write(Registro);
  Wire.write(Dato);
  Wire.endTransmission(); //Stop transmitting
}


  uint8_t leerByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	Wire.requestFrom(address, (uint8_t)1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void cambiarRangos(char key,int value)
{
  if (key == 'g'){
    switch (value) {
      case 0:
        Gscale=GFS_250DPS;
        setFullScaleGyroRange();
        break;
      case 1:
        Gscale=GFS_500DPS;
        setFullScaleGyroRange();
        break;
      case 2:
        Gscale=GFS_1000DPS;
        setFullScaleGyroRange();
        break;
      case 3:
        Gscale=GFS_2000DPS;
        setFullScaleGyroRange();
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
        setFullScaleAccelRange();
        break;
      case 1:
        Ascale=AFS_4G;
        setFullScaleAccelRange();
        break;
      case 2:
        Ascale=AFS_8G;
        setFullScaleAccelRange();
        break;
      case 3:
        Ascale=AFS_16G;
        setFullScaleAccelRange();
        break;
      
      default: 
        // if nothing else matches, do the default
        // default is optional
      break;
    }
  }  
  if (key == 's'){
    escribirRegistro(SMPRT_DIV, value);//Configurar Sample Rate
  }
  if (key == 'l'){
    escribirRegistro(CONFIG, value);//Configurar Filtro
  }
  return;
}

void setFullScaleAccelRange(){
 escribirRegistro(ACCEL_CONFIG, Ascale<<3); //Se escribe en el registro ACCEL_CONFIG con su corrimiento de 3 bits respectivo
}

void setFullScaleGyroRange(){
  escribirRegistro(GYRO_CONFIG, Gscale<<3);   //Se escribe en el registro GYRO_CONFIG con su corrimiento de 3 bits respectivo
}





