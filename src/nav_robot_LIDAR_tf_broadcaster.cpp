#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <cmath>
#include <lidar_rotation/servo_pos.h> 

#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>

float translation_x, translation_y,translation_z;

void servoCallback(const lidar_rotation::servo_pos& servo_pos){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Transform transform2;
  transform.setOrigin  ( tf::Vector3(0.05,-0.01,0.5494) );
  transform2.setOrigin ( tf::Vector3(0.05,-0.01,0.5494) );
  tf::Quaternion q,q2;
  q.setRPY(0, (servo_pos.data-90)*M_PI/180, 0);
  q2.setRPY(0, 0, 0);
  transform.setRotation(q);
  transform2.setRotation(q2);

  br.sendTransform(tf::StampedTransform(transform, servo_pos.header.stamp, "base_link", "LIDAR_MRS"));
  br.sendTransform(tf::StampedTransform(transform2, servo_pos.header.stamp, "base_link", "LIDAR_base"));

}

int main(int argc, char** argv){
  ros::init(argc, argv, "LIDAR_tf_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("servo_pos_header", 10, &servoCallback);
 
  ros::spin();
  return 0;
}