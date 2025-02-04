#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define MIN_ABS_SPEED 30 
MPU6050 mpu;

double MotorVelocidadIzq = 1; //Velocidad del motor
double MotorVelocidadDer = 1; //Velocidad del motor
double PuntoEquilibrio = 180.4; //Punto donde se equilibra  

int ENA = 5;
int IN1 = 9;
int IN2 = 8; 
int IN3 = 11;
int IN4 = 10;
int ENB = 6;

double Kp = 72;  
double Kd = 2.4;  
double Ki = 290; 
         
bool dmpReady = false; // Si DMP es exitosa el valor es true
uint8_t mpuIntStatus; // Contiene el byte de estado de interrupción
uint8_t devStatus; // Comprueba el estatus del sensor despues de cada operación
uint16_t packetSize; // Paquete de DMP esperado
uint16_t fifoCount; // Cuenta los bytes en la cola
uint8_t fifoBuffer[64]; // Búfer de almacenamiento


Quaternion q; // [w, x, y, z] Para representar la orientación del objeto tridimencionalmente 
VectorFloat gravity; // [x, y, z] Vector de gravedad
float ypr[3]; // [yaw, pitch, roll] //Tipos de movimiento 
double input, output;
PID pid(&input, &output, &PuntoEquilibrio, Kp, Ki, Kd, DIRECT);//Pasamos los datos del PID a la función de la biblioteca 
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, MotorVelocidadIzq, MotorVelocidadDer);//Se declara un objeto de tipo LMotorController el cual tendra varios parametros como moverse a la izquierda, derecha, para controlarlo de forma sencilla
volatile bool mpuInterrupt = false; // Variable de la interrupción

void dmpDataReady()//Interrupción del sensor
{
 mpuInterrupt = true;
}


void setup()
{
 Serial.begin(9600);    
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE // Para unirse al bus I2C
 Wire.begin();
 TWBR = 24; // 400kHz I2C clock 
 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
 Fastwire::setup(400, true);
 #endif
 mpu.initialize();
 devStatus = mpu.dmpInitialize();//Si regresa un uno el sensor MPU esta funcionando correctamente 
 
 // Calibraciones del giroscopio
 mpu.setXGyroOffset(220);
 mpu.setYGyroOffset(76);
 mpu.setZGyroOffset(-85);
 mpu.setZAccelOffset(1688); 
 

 if (devStatus == 0) //En el caso de que funcione bien el sensor
 {
 
 mpu.setDMPEnabled(true); // Enable de el sensor
 attachInterrupt(0, dmpDataReady, RISING);// Interrupción del sensor
 mpuIntStatus = mpu.getIntStatus();
 dmpReady = true;// DMP esta lista
 packetSize = mpu.dmpGetFIFOPacketSize();// Tamaño de paquete esperado
 
 pid.SetMode(AUTOMATIC);//Para iniciar el control autómatico del PID
 pid.SetSampleTime(10);//Determina la frecuencia con la que se evalúa el algoritmo PID
 pid.SetOutputLimits(-255, 255); 
 }
 else
 {
 Serial.print(F("DMP Initialization failed (code "));
 Serial.print(devStatus);
 Serial.println(F(")"));
 }
}

void loop()
{
 if (!dmpReady) return;
 while (!mpuInterrupt && fifoCount < packetSize)// Espera la interrupción de MPU
 {
 pid.Compute();//Realizando calculos del PID y salida a motores
 motorController.move(output, MIN_ABS_SPEED);//Esta función nos permite controlar la velocidad y direción de los motores
 
 }
 mpuInterrupt = false;// Resetear el indicador de la interupcion
 mpuIntStatus = mpu.getIntStatus();// Obtener el estatus

 fifoCount = mpu.getFIFOCount();// Para obtener la cuenta de la cola de datos
 if ((mpuIntStatus & 0x10) || fifoCount == 1024)
 {
 // Resetear en caso de overflow
 mpu.resetFIFO();
 Serial.println(F("FIFO overflow!"));
 }
 else if (mpuIntStatus & 0x02)
 {
 
 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();// Esperando la longitud de datos disponible

 mpu.getFIFOBytes(fifoBuffer, packetSize); // Lee un paquete de FIFO
 fifoCount -= packetSize;

 mpu.dmpGetQuaternion(&q, fifoBuffer);
 mpu.dmpGetGravity(&gravity, &q);//Obtiene la gravedad
 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);//Tipos de movimiento del sensor
 input = ypr[1] * 180/M_PI + 180;//Conversión a grados 
 }
Serial.println(output);
 
}
