#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

int main( int argc, char** argv )
{   
    ros::init(argc, argv, "platform_maker_broadcaster");
    ros::NodeHandle node;

    ros::Publisher platform_marker_pub = node.advertise<visualization_msgs::Marker>("platform_marker", 100);
   
    ros::Rate rate(40);
    visualization_msgs::Marker platform_marker;
    platform_marker.header.frame_id = "mpu_6050_frame";
    platform_marker.id = 0;
    platform_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    platform_marker.action = visualization_msgs::Marker::ADD;

    platform_marker.scale.x = 1;
    platform_marker.scale.y = 1;
    platform_marker.scale.z = 1;

    platform_marker.color.r = 0.6f;
    platform_marker.color.g = 0.6f;
    platform_marker.color.b = 0.6f;
    platform_marker.color.a = 1.0;


    platform_marker.pose.orientation.x = 0;
    platform_marker.pose.orientation.y = 0;
    platform_marker.pose.orientation.z = 0;
    platform_marker.pose.orientation.w = 1;

    platform_marker.pose.orientation.z = 0 * (M_PI)/180;
    platform_marker.pose.orientation.y = 0 * (M_PI)/180;
    platform_marker.pose.orientation.x = 0 * (M_PI)/180;


    platform_marker.lifetime = ros::Duration();
    platform_marker.mesh_resource ="package://lidar_rotation/meshes/base_link_origin.stl";
    

    while(ros::ok()) 
    {
        platform_marker.header.stamp = ros::Time::now();
        platform_marker.lifetime = ros::Duration();
        platform_marker_pub.publish(platform_marker);
        ros::spinOnce();
        rate.sleep();
    }
}
