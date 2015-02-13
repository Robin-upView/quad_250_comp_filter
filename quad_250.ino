
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MS561101BA.h"
#include "String.h"

#include <Servo.h>

#define CH1  3  // Pin numbers //av gauche
#define CH2  5  //ar droit
#define CH3  6  //ar gauche
#define CH4  7  //av droit

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

MPU6050 accelgyro;
MS561101BA baro = MS561101BA();

int16_t ax, ay, az;
int16_t gx, gy, gz;

volatile unsigned long ulStartPeriod = 0; // set in the interrupt
volatile boolean bNewThrottleSignal = false; // set in the interrupt and read in the loop
volatile int rc[7];

float pitch, roll;
float G_Dt=0.02;

int led = 13;

long timer=0; //general purpose timer 
long timer_old;

float biasX, biasY, biasZ;

float command_pitch;
float err_pitch;
float pid_pitch;
float pitch_I;
float pitch_D;
float err_pitch_old;

float command_roll;
float err_roll;
float pid_roll;
float roll_I;
float roll_D;
float err_roll_old;

float kp = 3.0;
float ki = 0.9; 
float kd = 0.6;

float pid_yaw;
float err_yaw;
float yaw_I;

float err_altitude;
float err_altitude_old;
float command_altitude;
float altitude_I;
float altitude_D;
float pressure_demand;
float pressure;
float pid_altitude;
bool altitude_init;

float throttle;


float press;


Servo Servo_1;
Servo Servo_2;
Servo Servo_3;
Servo Servo_4;

void setup()
{  
  pinMode(CH1,OUTPUT);
  pinMode(CH2,OUTPUT);
  pinMode(CH3,OUTPUT);
  pinMode(CH4,OUTPUT);
  
  Servo_1.attach (CH1, 1000,1900);  //av gauche
  Servo_2.attach (CH2, 1000,1900);  //ar droit
  Servo_3.attach (CH3, 1000,1900);  //ar gauche
  Servo_4.attach (CH4, 1000,1900);  //av droit
  
  Servo_1.writeMicroseconds(1000);//arrière droit
  Servo_2.writeMicroseconds(1000);//avant gauche
  Servo_3.writeMicroseconds(1000);//arrière gauche
  Servo_4.writeMicroseconds(1000);


  attachInterrupt(0,calcInput,CHANGE);
  Serial.begin(115200);
  Wire.begin();
  
  pinMode(led, OUTPUT); 
  accelgyro.setSleepEnabled(false);
  
 accelgyro.setFullScaleGyroRange(3); //Gyro scale 2000deg/s
  delay(1);
  accelgyro.setFullScaleAccelRange(1);//Accel scale 4g
  delay(1);
  accelgyro.setClockSource(3);// Select GyroZ clock
  delay(1);
  accelgyro.setDLPFMode(4);// set bandwidth of both gyro and accelerometer to ~20 Hz
  delay(1);
  

  calib_gyro();
  
  baro.init(MS561101BA_ADDR_CSB_LOW);

  timer = micros();
  delay(20);
}


void loop()
{
  if((micros()-timer)>=5000)   
  { 
    timer_old = timer;
    timer=micros();
    G_Dt = (timer-timer_old)/1000000.0;      // Real time of loop run 

    fast_Loop();

  }
}


void fast_Loop(){

  imu_Valget (); // read sensors

  press = baro.getPressure(MS561101BA_OSR_4096);
  
  

  //pid pitch = rc1
  command_pitch = -(rc[1]-1200.0)/20;
  err_pitch = command_pitch - pitch;
  pitch_D = (float)(-gx+biasX)*2000.0f/32768.0f;
  pitch_I += (float)err_pitch*G_Dt; 
  //pitch_I = constrain(pitch_I,-50,50);
  pid_pitch = err_pitch*kp+pitch_I*ki+pitch_D*kd; //P=10 I=15 D=5 was good //D=8 the limit //P=15 I=30 D=5
  //pid_pitch = 0;

  //ROLL
  command_roll = (rc[0]-1200.0)/20;
  err_roll = command_roll - roll;
  roll_D = (float)(gy-biasY)*2000.0f/32768.0f;
  roll_I += (float)err_roll*G_Dt; 
  //roll_I = constrain(roll_I,-50,50);
  pid_roll = err_roll*kp+roll_I*ki+roll_D*kd; //P=0.1 I=0.85 D=0.1
  //pid_roll=0; //à supprimer

  //YAW
  err_yaw = (float)(-(gz-biasZ))*2000.0f/32768.0f;
  yaw_I += (float)err_yaw*G_Dt; 
  pid_yaw = err_yaw*6.0+yaw_I*6.0;
  //pid_yaw=0;
  //Throttle
  
  //ALTITUDE
  if(rc[4]>1500)
  {
    if(altitude_init==false)
    {
      pressure_demand = press;
      altitude_init = true;
    }
    
    err_altitude_old = err_altitude;
    err_altitude = - pressure_demand + press;
    //err_altitude = constrain(err_altitude,-60,60);  
    altitude_D = (float)(err_altitude-err_altitude_old)/G_Dt;
    altitude_I += (float)err_altitude*G_Dt;
    //altitude_I = constrain(altitude_I,-150,150);
    pid_altitude = err_altitude*180.0 + altitude_I*50.0 + altitude_D*50.0;
  }
  else
  {
    altitude_init = false; //if we want to enable the altitude hold more than once
    pid_altitude = 0;
    altitude_I = 0;
  }
  
  
  IMU_print();
  

  throttle = constrain(rc[2]*1.20,1000,1900);

  if(throttle < 1100)
  {
    pid_pitch=0;
    pid_roll=0;
    pid_yaw=0;
    pid_altitude = 0;

    pitch_I=0;
    roll_I=0;
    yaw_I=0;
  }

  //IMU_print();

  Servo_2.writeMicroseconds(constrain(throttle+pid_roll-pid_pitch+pid_yaw+pid_altitude,1000,1900));//arrière droit
  Servo_1.writeMicroseconds(constrain(throttle-pid_roll+pid_pitch+pid_yaw+pid_altitude,1000,1900));//avant gauche
  Servo_3.writeMicroseconds(constrain(throttle-pid_roll-pid_pitch-pid_yaw+pid_altitude,1000,1900));//arrière gauche
  Servo_4.writeMicroseconds(constrain(throttle+pid_roll+pid_pitch-pid_yaw+pid_altitude,1000,1900));//avant droit


/*
  Servo_1.writeMicroseconds(throttle); //set servo position 
  Servo_2.writeMicroseconds(throttle);
  Servo_3.writeMicroseconds(throttle); 
  Servo_4.writeMicroseconds(throttle);
*/
}

void calcInput()
{
  static unsigned int nThrottleIn;
  static int channel;

  if(digitalRead(2) == HIGH)
  { 
    ulStartPeriod = micros();
  }
  else
  {
    if(ulStartPeriod)
    {
      nThrottleIn = (int)(micros() - ulStartPeriod);
      ulStartPeriod = 0;

      if(nThrottleIn >2000){
        channel = 0;
      }
      else
      {
        rc[channel]=nThrottleIn;
        channel++;
      }
    }
  }
}



