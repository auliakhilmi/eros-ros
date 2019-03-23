// Credit by Korneliusz Jarzebski 
// Modified by Aulia Khilmi Rizgi
// EROS 2019

#include <Wire.h>
#include <MPU6050.h>
#include <ros.h>
#include <robotcontrol/LowLevel.h>

#define MAX_PUSHB 4

ros::NodeHandle  nh;
robotcontrol::LowLevel lowlevel;
ros::Publisher pub_("lowlevel", &lowlevel);

MPU6050 mpu;
char PUSHB[4]={10,11,12,13};
char led[4]={2,3,4,5};
int i,j;

void reset(){
  for(j=0; j<MAX_PUSHB;j++)
    digitalWrite(led[j],LOW);
}

void setup(){
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(pub_);
  
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    //Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  
  for(i=0; i<MAX_PUSHB;i++){
    pinMode(PUSHB[i],INPUT); digitalWrite(PUSHB[i],HIGH); 
    pinMode(led[i],OUTPUT);  digitalWrite(led[i],LOW);
  }
  
}

void loop(){
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();

  lowlevel.acc_x=normAccel.XAxis;
  lowlevel.acc_y=normAccel.YAxis;
  lowlevel.acc_z=normAccel.ZAxis;

  lowlevel.gyro_x=normGyro.YAxis;
  lowlevel.gyro_y=normGyro.XAxis;
  lowlevel.gyro_z=normGyro.ZAxis;
  
  for(i=0;i<MAX_PUSHB;i++){
    if(digitalRead(PUSHB[i])==LOW){
      delay(60);
      if(digitalRead(PUSHB[i])==LOW){
        reset();
        digitalWrite(led[i],HIGH);
        lowlevel.pushb=i+1;
      }
    }
  }
  
  pub_.publish(&lowlevel);
  nh.spinOnce();
  delay(10);
}

