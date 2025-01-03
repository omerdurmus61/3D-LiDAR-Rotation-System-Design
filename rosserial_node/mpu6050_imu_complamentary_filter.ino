#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Servo.h>
//#include <imu_gps_servo/imu_gps_servo_msgs.h>

ros::NodeHandle  nh;

sensor_msgs::Imu imu;
int servo_pos=50;
int servo_step;
int count = 3;
std_msgs::Int16 servo_pos_data;
std_msgs::Int16 long_lat[2];

ros::Publisher imu_pub("imu", &imu);
ros::Publisher servo_pos_pub("servo_pos", &servo_pos_data);
ros::Publisher gps_pub("gps", &long_lat[2]);

Servo mg995;

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch, AngleYaw;
float t_start,t_stop;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

// Complementary filter parameters
float alpha = 0.98;  // Weight for gyroscope
float dt = 0.01;    // Time step

void calibration() {
  for (int calibrationIteration = 0; calibrationIteration < 2000; calibrationIteration++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
}

void gyro_signals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);  // Full-scale range: ±2000°/s
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  
  imu.linear_acceleration.x = (float)AccXLSB / 4096;
  imu.linear_acceleration.y = (float)AccYLSB / 4096;
  imu.linear_acceleration.z = (float)AccZLSB / 4096;

  
  imu.orientation.x =  atan(imu.linear_acceleration.y / sqrt(imu.linear_acceleration.x * imu.linear_acceleration.x + imu.linear_acceleration.z * imu.linear_acceleration.z)) * 1 / (3.142 / 180);
  imu.orientation.y = -atan(imu.linear_acceleration.x / sqrt(imu.linear_acceleration.y * imu.linear_acceleration.y + imu.linear_acceleration.z * imu.linear_acceleration.z)) * 1 / (3.142 / 180); 



}

void setup() {
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(servo_pos_pub);
  nh.advertise(gps_pub);
  mg995.attach(13);
  mg995.write(90);
  Serial.begin(256000);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  calibration();

}

void loop() {
  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  
  // Complementary filter for yaw angle
  AngleYaw += RateYaw * dt;
  AngleYaw = alpha * (AngleYaw + RateYaw * dt) + (1 - alpha) * AngleYaw;
  
  
  imu.orientation.z = AngleYaw ;
  imu.header.stamp = nh.now();

 if(count%3==0)
 {
     mg995.write(servo_pos);
     count+=1;
     imu.angular_velocity.z = servo_pos;
     servo_pos_data.data = servo_pos;
     servo_pos = servo_pos + servo_step;
 }
 else
 {
    count+=1;
 }


  if(servo_pos==50)
  servo_step = 1;
  else if(servo_pos==130)
  servo_step=-1;

  imu_pub.publish( &imu );
  servo_pos_pub.publish( &servo_pos_data );

  nh.spinOnce();
  

  
  delay(5); // Adjust delay as needed
}
