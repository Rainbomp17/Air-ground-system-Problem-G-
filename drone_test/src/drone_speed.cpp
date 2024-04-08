#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <unistd.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <ros/duration.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Imu.h>
tf2_ros::Buffer tfBuffer;
ros::Publisher mag_pub;
ros::Publisher imu_pub;
using namespace std;

void Publish_mag(void)
{
    geometry_msgs::Point Result;
    Result.x = 0;
    Result.y = 0;
    Result.z = 0;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PointStamped initial_pt, transformed_pt;
    try
    {
        transformStamped = tfBuffer.lookupTransform("YYY_see_body", "camera_init", ros::Time(0)); // target,source
        initial_pt.point.x = 0;                                                                   // f(aliened with E at startup,thus mag=0)
        initial_pt.point.y = 80000;                                                               // l(aliened with N at startup,thus mag=800)
        initial_pt.point.z = 10000;                                                               // u
        tf2::doTransform(initial_pt, transformed_pt, transformStamped);
        Result = transformed_pt.point;
        mag_pub.publish(Result);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(0.1).sleep();
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_node");
    ros::NodeHandle nh;
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);
    // ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/vision_pose/pose", 1, odom_cb);
    tf2_ros::TransformListener tfListener(tfBuffer);
    mag_pub = nh.advertise<geometry_msgs::Point>("/mavros/virtual_compass/virtual_compass", 1);
    // imu_pub = nh.advertise<sensor_msgs::Imu>("/mavros/imu/data", 1);
    // allow the subscribers to initialize
    ROS_INFO("INITILIZING...");
    for (int i = 0; i < 100; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    while (ros::ok())
    {
        Publish_mag();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
