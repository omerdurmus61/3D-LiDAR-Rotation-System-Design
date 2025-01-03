#include <ros/ros.h>
#include <tf/transform_listener.h>
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
tf::TransformListener *listener;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  tf::StampedTransform transform;
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *input_pcl);

  listener->waitForTransform("/LIDAR_base", "/LIDAR_MRS",   ros::Time(0),ros::Duration(0.5));
  listener->lookupTransform("/LIDAR_base", "/LIDAR_MRS",   ros::Time(0), transform);
  
  Eigen::Matrix4f eigen_transform;
  pcl_ros::transformAsMatrix (transform, eigen_transform);
  
  pcl::transformPointCloud (*input_pcl, *transformed_pcl, eigen_transform);
  pcl::toROSMsg(*transformed_pcl,output);
  output.header.frame_id = "LIDAR_base";
  output.header.stamp = input->header.stamp;
  pub.publish (output);

}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "transformation_system");
  ros::NodeHandle nh;
  listener = new tf::TransformListener();
  ros::Subscriber sub = nh.subscribe ("/cloud", 10, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/transformed_cloud", 10);

  ros::spin ();
}



