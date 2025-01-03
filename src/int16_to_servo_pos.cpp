#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Header.h>
#include <lidar_rotation/servo_pos.h>
#include <cmath>

ros::Publisher header_servo_pos_pub;
lidar_rotation::servo_pos header_servo_pos;
void callback(const std_msgs::Int16& servo_pos)
{  
   header_servo_pos.data = servo_pos.data;
   header_servo_pos.header.stamp = ros::Time::now();
   header_servo_pos_pub.publish(header_servo_pos);

}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "int16_to_servo_pos");
  ros::NodeHandle nh;
  ros::Subscriber servo_pos_sub = nh.subscribe("/servo_pos", 1000, callback);
  header_servo_pos_pub = nh.advertise<lidar_rotation::servo_pos>("/servo_pos_header", 1000);  
  ros::spin();
  

  return 0;
}