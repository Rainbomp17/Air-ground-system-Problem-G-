#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher camera_speed_publisher;

geometry_msgs::TwistWithCovariance camera_twist;
void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    camera_twist = (*msg).twist;
    static geometry_msgs::TwistWithCovarianceStamped twist_send;
    twist_send.header.stamp = ros::Time::now();
    twist_send.twist.twist.linear.x = -1.0 * camera_twist.twist.linear.x;
    twist_send.twist.twist.linear.y = -1.0 * camera_twist.twist.linear.y;
    twist_send.twist.twist.linear.z = 1.0 * (camera_twist.twist.linear.z);
    twist_send.twist.twist.angular.x = -1.0 * (camera_twist.twist.angular.x);
    twist_send.twist.twist.angular.y = -1.0 * (camera_twist.twist.angular.y);
    twist_send.twist.twist.angular.z = 1.0 * (camera_twist.twist.angular.z);
    camera_speed_publisher.publish(twist_send);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // 2.初始化 ROS 节点
    ros::init(argc, argv, "dynamic_tf_pub");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建订阅对象
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1, odom_cb);
    camera_speed_publisher = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/mavros/vision_speed/speed_twist_cov", 1);
    // ros::Publisher camera_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // tf2_ros::Buffer tf_buffer;
    // tf2_ros::TransformListener tf_listener(tf_buffer);
    // geometry_msgs::TransformStamped transform_stamped;
    // geometry_msgs::PoseStamped msg_body_pose;
    // std::string target_frame_id = "YYY_see_body";
    // std::string source_frame_id = "YYY_see_body_takeoff";

    while (ros::ok())
    {

        // try
        // {
        //     transform_stamped = tf_buffer.lookupTransform(target_frame_id, source_frame_id, ros::Time(0));
        //     // Only publish pose messages when we have new transform data.

        //     // 将ROS的geometry_msgs转换为tf2的数据类型
        //     tf2::Transform transform;
        //     tf2::fromMsg(transform_stamped.transform, transform);
        //     tf2::Vector3 position_orig = transform.getOrigin();
        //     tf2::Quaternion quat_lidar = transform.getRotation();

        //     // Create PoseStamped message to be sent
        //     msg_body_pose.header.stamp = transform_stamped.header.stamp;
        //     msg_body_pose.header.frame_id = transform_stamped.header.frame_id;
        //     msg_body_pose.pose.position.x = position_orig.getX();
        //     msg_body_pose.pose.position.y = position_orig.getY();
        //     msg_body_pose.pose.position.z = position_orig.getZ();
        //     msg_body_pose.pose.orientation.x = quat_lidar.getX();
        //     msg_body_pose.pose.orientation.y = quat_lidar.getY();
        //     msg_body_pose.pose.orientation.z = quat_lidar.getZ();
        //     msg_body_pose.pose.orientation.w = quat_lidar.getW();

        //     // Publish pose of body frame in world frame
        //     camera_pose_publisher.publish(msg_body_pose);
        //     ROS_INFO("TFCPPPPP");
        // }
        // catch (tf2::TransformException &ex)
        // {
        //     ROS_WARN("Failed to lookup transform: %s", ex.what());
        // }
    }
    return 0;
}
