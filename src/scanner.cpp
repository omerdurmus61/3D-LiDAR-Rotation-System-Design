#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
bool flag = true;
int counter = 0;
pcl::PointCloud<pcl::PointXYZ>::Ptr concatenated_pcl(new pcl::PointCloud<pcl::PointXYZ>);

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (input->data.empty())
  {
    ROS_WARN("Received an empty point cloud message. Skipping processing.");
    return;
  }

  // Convert the input point cloud to PCL format
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *input_pcl);

  // Voxel Grid Filtering (Downsampling)
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  voxel_grid_filter.setInputCloud(input_pcl);
  voxel_grid_filter.setLeafSize(0.05, 0.05, 0.05);  // Set the voxel size (adjust as needed)
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_filter.filter(*downsampled_pcl);

  // Concatenate the input point cloud with the concatenated point cloud
  *concatenated_pcl += *downsampled_pcl;


  // Publish the concatenated point cloud
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*concatenated_pcl, output);
  output.header = input->header;
  pub.publish(output);
}



int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "scanner");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/transformed_cloud", 1, cloud_cb);

  // Create a ROS publisher for the concatenated point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("/scanning", 1);

  // Spin
  ros::spin();

  return 0;
}
