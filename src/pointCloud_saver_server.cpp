#include "ros/ros.h"
#include "std_srvs/Empty.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (input->data.empty())
  {
    ROS_WARN("Received an empty point cloud message. Skipping processing.");
    return;
  }

  // Convert the input point cloud to PCL format
  pcl::fromROSMsg(*input, *map);

}

bool pointCloud_saver(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  pcl::io::savePCDFileBinary("map.pcd", *map);
  ROS_INFO("Concatenated point cloud saved!");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointCloud_saver_server");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/scanning", 1, cloud_cb);

  ros::ServiceServer service = n.advertiseService("pointCloud_saver", pointCloud_saver);

  ros::spin();

  return 0;
}