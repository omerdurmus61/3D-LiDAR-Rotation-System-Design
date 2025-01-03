#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

ros::Publisher pub;
bool is_program_started = true;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr saved_output (new pcl::PointCloud<pcl::PointXYZ>);

  // Do data processing here...
  output = *input;
  pcl::fromROSMsg(*input, *saved_output);

  // Publish the data.
  pub.publish (output);

  //Save the data,
  if(is_program_started)
  {
    is_program_started = false;
    pcl::io::savePCDFileASCII("/home/omer/Desktop/rotation_system_test1.pcd", *saved_output);
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "rotation_system");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_test", 1);

  // Spin
  ros::spin ();
}