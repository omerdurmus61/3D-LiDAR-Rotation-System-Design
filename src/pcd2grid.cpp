#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <limits>

ros::Publisher pcl_pub;
ros::Publisher map_pub;
nav_msgs::OccupancyGrid occupancy_grid;

void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";
    output.header.stamp = ros::Time::now();
    pcl_pub.publish(output);
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    int grid_size_x = 380;  
    int grid_size_y = 450; 
    double grid_resolution = 0.05; 
    std::vector<int8_t> grid_data(grid_size_x * grid_size_y, 0); 
    double origin_x_m = -2.92;
    double origin_y_m = -4.58;

    for (const auto& point : cloud.points)
    {
        int x = static_cast<int>((point.x - origin_x_m) / grid_resolution);
        int y = static_cast<int>((point.y - origin_y_m) / grid_resolution);

        if (x >= 0 && x < grid_size_x && y >= 0 && y < grid_size_y)
        {
            grid_data[y * grid_size_x + x] = 100; 
        }
    }

    occupancy_grid.header.stamp = ros::Time::now();
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.resolution = grid_resolution;
    occupancy_grid.info.width = grid_size_x;
    occupancy_grid.info.height = grid_size_y;
    occupancy_grid.info.origin.position.x = origin_x_m;
    occupancy_grid.info.origin.position.y = origin_y_m;
    occupancy_grid.info.origin.orientation.w = 1.0;
    occupancy_grid.data = grid_data;

    map_pub.publish(occupancy_grid);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;

    std::string pcd_file_path;
    nh.param<std::string>("pcd_file_path", pcd_file_path, "/home/omer/lidar_rotation_system/maps/clean_map_pcd2grid.pcd");

    if (pcd_file_path.empty())
    {
        ROS_ERROR("PCD file path not provided! Please set the pcd_file_path parameter.");
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1)
    {
        ROS_ERROR("Couldn't read the PCD file. Please check the file path.");
        return -1;
    }

    ROS_INFO("Loaded PCD file with %zu points.", cloud->points.size());

    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcd_topic", 1);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid_map", 1);

    ros::Subscriber pcl_sub = nh.subscribe("pcd_topic", 1, pointCloudCallback);

    ros::Rate loop_rate(0.5); // 0.5 Hz

    while (ros::ok())
    {
        publishPointCloud(cloud);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
