#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

float translation_x, translation_y,translation_z;

void imuCallback(const sensor_msgs::Imu& imu){

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0,-0.25,0) );
  tf::Quaternion q;
  q.setRPY(imu.orientation.x*M_PI/180, imu.orientation.y*M_PI/180, (imu.orientation.z)*M_PI/180);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, imu.header.stamp, "world", "mpu_6050_frame"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "imu_tf_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("imu", 10, &imuCallback);

  ros::spin();
  return 0;
};