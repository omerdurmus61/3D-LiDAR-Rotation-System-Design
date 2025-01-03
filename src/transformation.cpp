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
// PCL specific includes 

ros::Publisher pub;
float theta = 0;
bool is_program_started = true;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  // Create a container for the data.

  // Do data processing here
  pcl::fromROSMsg(*input, *input_pcl);

  Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();

  // Define a rotation matrix for y axis
/*
        | cos(theta)    0    sin(theta) |       
  R =   |     0         1       0       |
        | -sin(theta)   0    cos(theta) |
*/
  //Transformation matrix 
/*
    |  R   | t |   t is a translation vector,  t = [x,y,z]^T , in our case t = [0,0,0]^T
    |--------- |
    |0 0 0 | 1 |
*/  

  transform_y (0,0) = std::cos (theta*(M_PI/180));         
  transform_y (0,2) = sin(theta*(M_PI/180));                
  transform_y (2,0) = -sin (theta*(M_PI/180)); 
  transform_y (2,2) = std::cos (theta*(M_PI/180));
  transform_y (1,1) = 1;
  transform_y (3,3) = 1;

  //Executing the transform
  pcl::transformPointCloud (*input_pcl, *transformed_pcl, transform_y);
  pcl::toROSMsg(*transformed_pcl,output);

  // Publish the data.
  pub.publish (output);
  
  if(theta ==-45 && is_program_started)
  {
    is_program_started = false;
    pcl::io::savePCDFileASCII("/home/omer/Desktop/mrs_rotated_data.pcd", *transformed_pcl);
  }


  if(theta>-90)
    theta -=1 ;
  else
    theta = 0;

  // Do data processing here
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "rotation_system");
  ros::NodeHandle nh;
  // Initialize ROS

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/cloud", 1, cloud_cb);
  // Create a ROS subscriber for the input point cloud

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/rotated_cloud", 1);
  // Create a ROS publisher for the output point cloud

  // Spin
  ros::spin ();
}