// Credit by Korneliusz Jarzebski 
// Modified by Aulia Khilmi Rizgi
// EROS 2019

#include <Wire.h>
#include <MPU6050.h>
#include <ros.h>
#include <robotcontrol/LowLevel.h>

ros::NodeHandle  nh;
robotcontrol::LowLevel lowlevel;
ros::Publisher pub_("lowlevel", &lowlevel);

MPU6050 mpu;

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
}

void loop(){
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();

  lowlevel.acc_x=normAccel.XAxis;
  lowlevel.acc_y=normAccel.YAxis;
  lowlevel.acc_z=normAccel.ZAxis;

  lowlevel.gyro_x=normGyro.XAxis;
  lowlevel.gyro_y=normGyro.YAxis;
  lowlevel.gyro_z=normGyro.ZAxis;
  
  pub_.publish(&lowlevel);
  nh.spinOnce();
  delay(10);
}

