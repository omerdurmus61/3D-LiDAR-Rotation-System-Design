#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

int main( int argc, char** argv )
{   
    ros::init(argc, argv, "LIDAR_maker_broadcaster");
    ros::NodeHandle node;

    ros::Publisher lidar_marker_pub = node.advertise<visualization_msgs::Marker>("LIDAR_MRS_marker", 100);
   
    ros::Rate rate(40);
    visualization_msgs::Marker lidar_marker;
    lidar_marker.header.frame_id = "LIDAR_MRS";
    lidar_marker.id = 0;
    lidar_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    lidar_marker.action = visualization_msgs::Marker::ADD;

    lidar_marker.scale.x = 1;
    lidar_marker.scale.y = 1;
    lidar_marker.scale.z = 1;

    lidar_marker.color.r = 0.6f;
    lidar_marker.color.g = 0.6f;
    lidar_marker.color.b = 0.6f;
    lidar_marker.color.a = 1.0;


    lidar_marker.pose.orientation.x = 0;
    lidar_marker.pose.orientation.y = 0;
    lidar_marker.pose.orientation.z = 0;
    lidar_marker.pose.orientation.w = 1;

    lidar_marker.pose.orientation.z = 0 * (M_PI)/180;
    lidar_marker.pose.orientation.y = 0 * (M_PI)/180;
    lidar_marker.pose.orientation.x = 0 * (M_PI)/180;


    lidar_marker.lifetime = ros::Duration();
    lidar_marker.mesh_resource ="package://lidar_rotation/meshes/LIDAR_MRS_origin.stl";
    

    while(ros::ok()) 
    {
        lidar_marker.header.stamp = ros::Time::now();
        lidar_marker.lifetime = ros::Duration();
        lidar_marker_pub.publish(lidar_marker);
        ros::spinOnce();
        rate.sleep();
    }
}
