#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "drone_test/Y_Serial_port.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry current_Alt;
double mixed_alt = 0;
static drone_test::Y_Serial_port uart_control_rx;
int flag_takeoffed = 0;
int tf_flag = 1;
geometry_msgs::PoseStamped current_pose;

void doPerson(const drone_test::Y_Serial_port::ConstPtr &person_p) // 接收串口数据
{
    uart_control_rx = *person_p;
    if ((int)(person_p->if_takeoff) == 1)
    {
        flag_takeoffed = 1;
    }
    // ROS_INFO("%d, %d, %d, %d", (int)person_p->mode, (int)person_p->tar1, (int)person_p->tar2, (int)person_p->if_takeoff);
}

void get_Alt(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_Alt = *msg;
    mixed_alt = current_Alt.twist.twist.linear.y;
    // ROS_INFO("now_position[2]:%f", now_position[2]);
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    try
    {
        /* code */
        static tf2_ros::TransformBroadcaster broadcaster_body;
        //  5-2.创建 广播的数据(通过 pose 设置)
        static geometry_msgs::TransformStamped tfs_body;
        static geometry_msgs::PoseStamped msg_body_pose;
        //  |----头设置
        tfs_body.header.stamp = current_pose.header.stamp;
        tfs_body.header.frame_id = "camera_init";

        //  |----坐标系 ID
        tfs_body.child_frame_id = "YYY_see_body";

        //  |----坐标系相对信息设置
        tfs_body.transform.translation.x = current_pose.pose.position.x;
        tfs_body.transform.translation.y = current_pose.pose.position.y;
        tfs_body.transform.translation.z = current_pose.pose.position.z;
        tfs_body.transform.rotation.x = current_pose.pose.orientation.x;
        tfs_body.transform.rotation.y = current_pose.pose.orientation.y;
        tfs_body.transform.rotation.z = current_pose.pose.orientation.z;
        tfs_body.transform.rotation.w = current_pose.pose.orientation.w;
        //  5-3.广播器发布数据
        broadcaster_body.sendTransform(tfs_body);
        ROS_INFO("tf published");
    }
    catch (const std::exception &e)
    {
        // std::cerr << e.what() << '\n';
    }

    //  5-1.创建 TF 广播器
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // 2.初始化 ROS 节点
    ros::init(argc, argv, "tf_takeoff_pub");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建订阅对象
    ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10, pose_cb);
    ros::Subscriber sub = nh.subscribe<drone_test::Y_Serial_port>("serial_Chip", 10, doPerson);
    ros::Subscriber currentAlt = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 1, get_Alt);
    ROS_INFO("tf published");
    // 5.广播器发布坐标系信息

    while (ros::ok())
    {
        ros::spinOnce();

        if (tf_flag == 1)
        {
            static tf2_ros::StaticTransformBroadcaster broadcaster;
            static tf2_ros::StaticTransformBroadcaster broadcaster_world;
            //  5-2.创建 广播的数据(通过 pose 设置)
            geometry_msgs::TransformStamped tfs;
            geometry_msgs::TransformStamped tfs_world;
            //  |----头设置
            tfs.header.frame_id = "camera_init";

            tfs.header.stamp = ros::Time::now();

            //  |----坐标系 ID
            tfs.child_frame_id = "YYY_see_body_takeoff";

            //  |----坐标系相对信息设置
            ros::spinOnce();
            tfs.transform.translation.x = current_pose.pose.position.x;
            tfs.transform.translation.y = current_pose.pose.position.y;
            tfs.transform.translation.z = current_pose.pose.position.z;
            tfs.transform.rotation.x = current_pose.pose.orientation.x;
            tfs.transform.rotation.y = current_pose.pose.orientation.y;
            tfs.transform.rotation.z = current_pose.pose.orientation.z;
            tfs.transform.rotation.w = current_pose.pose.orientation.w;
            ROS_INFO("current_pose.pose.position.x:%f", current_pose.pose.position.x);
            ROS_INFO("current_pose.pose.position.y:%f", current_pose.pose.position.y);
            ROS_INFO("current_pose.pose.position.z:%f", current_pose.pose.position.z);
            ROS_INFO("current_pose.pose.orientation.w:%f", current_pose.pose.orientation.w);

            tfs_world.header.frame_id = "camera_init";
            tfs_world.header.stamp = ros::Time::now();
            tfs_world.child_frame_id = "world";
            tfs_world.transform.translation.x = current_pose.pose.position.x - 0.4;
            tfs_world.transform.translation.y = current_pose.pose.position.y + 0.4;
            tfs_world.transform.translation.z = current_pose.pose.position.z;
            tfs_world.transform.rotation.x = current_pose.pose.orientation.x;
            tfs_world.transform.rotation.y = current_pose.pose.orientation.y;
            tfs_world.transform.rotation.z = current_pose.pose.orientation.z;
            tfs_world.transform.rotation.w = current_pose.pose.orientation.w;
            //  5-3.广播器发布数据
            if (current_pose.pose.orientation.x && current_pose.pose.orientation.y && current_pose.pose.orientation.z && current_pose.pose.orientation.w)
            {

                broadcaster_world.sendTransform(tfs_world);
                broadcaster.sendTransform(tfs);
                tf_flag = 0;
            }
            else
            {
                ROS_INFO("Publliing tf failed");
            }
        }
    }

    return 0;
}