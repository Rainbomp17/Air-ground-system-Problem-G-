#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandHome.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Byte.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include <nav_msgs/Odometry.h>
#include <sys/time.h>
#include <pthread.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <mavros_msgs/DebugValue.h>
#include <drone_test/detection.h>
#include <tf/tf.h>
#include "std_msgs/Float64.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "drone_test/Y_Serial_port.h"
#include <typeinfo>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

using namespace cv;
using namespace std;
using namespace mavros_msgs;

/*----------------------------------------------------------------------------------------------------------------------------------- */

enum Position_ControlMode// 飞控状态机
{

    Position_ControlMode_Null = 255,

    // Position_ControlMode_VelocityTrack = 16 ,
    Position_ControlMode_Position = 12,
    Position_ControlMode_Velocity = 11,
    Position_ControlMode_Locking = 10,

    // 2D
    Position_ControlMode_Takeoff = 20,
    Position_ControlMode_RouteLine = 22,

    // 3D
    Position_ControlMode_RouteLine3D = 52,
};

class PIDController// PID控制器
{
public:
    PIDController() : Kp(0), Ki(0), Kd(0), min_output(0), max_output(0), last_error(0), integral(0) {}

    void setParameters(double kp, double ki, double kd, double minoutput, double maxoutput)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        min_output = minoutput;
        max_output = maxoutput;
    }

    double update(double target, double feedback)
    {

        double dt = 1; // PID控制器采样时间
        double error = target - feedback;
        integral += error * dt;

        double derivative = (error - last_error) / dt;

        double output = Kp * error + Ki * integral + Kd * derivative;
        last_error = error;
        output = std::max(output, min_output);
        output = std::min(output, max_output);
        return output;
    }

    void clear()
    {
        last_error = 0;
        integral = 0;
    }

private:
    double Kp,
        Ki,
        Kd;
    double min_output, max_output;
    double last_error;
    double integral;
};

/*----------------------------------------------------------------------------------------------------------------------------------- */
ros::Publisher set_raw_pub;
ros::Publisher uart_control_pub;
geometry_msgs::PoseStamped current_pose;
nav_msgs::Odometry current_speed;
nav_msgs::Odometry current_Alt;
mavros_msgs::State current_state;
mavros_msgs::DebugValue PosAlt_atate;

PIDController pid_controller_set_break_speed;
PIDController pid_controller_set_xy_pose_local;
PIDController pid_controller_set_contiue_pose;
PIDController pid_controller_set_yaw_pose_local;
PIDController pid_controller_set_z_pose_local;
ros::Publisher camera_speed_publisher;
static drone_test::Y_Serial_port uart_control_rx;
/*----------------------------------------------------------------------------------------------------------------------------------- */
// 极限速度
double max_vel_yaw = 36;
double max_vel_xy = 0.4;
double max_vel_dis_xy = 1;
double max_vel_z = 0.3;
double min_stop_error = 0.1;

// 飞控状态
int Alt_mode;
int Pos_mode;
int is_RC_control;
double now_yaw;
double now_position[3] = {0};
double mixed_alt = 0;
double now_speed[3] = {0};

// 图像消息_速度
double opencv_erro_x = 0;
double opencv_erro_y = 0;
double opencv_erro_z = 0;
// 图像消息_状态
int opencv_flag = 0;
int start_opencv = 0;

double yaw_takeoff = 0;
int flag_tf = 0;

int fly_flag = 0;// 
int send_flag = 0;// 把坐标发送给小车的标志位
double x_pos[12] = {3.6, 3.6, 1.3, 1.3, 3.6, 3.6, 1.3, 1.3, 3.6, 3.6, 0.7, 0.5};
double y_pos[12] = {-0.5, -1.3, -1.3, -2, -2, -2.8, -2.8, -3.7, -3.7, -4.5, -4.5, -0.5};

// 核心点坐标
double x_pos_neat[6] = {1.6, 2.4, 1.6, 2.4, 1.6, 2.4};
double y_pos_neat[6] = {-1.6, -1.6, -2.4, -2.4, -3.2, -3.2};

/*----------------------------------------------------------------------------------------------------------------------------------- */

void uart_pub(int mode, int tar1, int tar2, int if_takeoff);

int takeoff(ros::NodeHandle &nh, double height); // meters

int land(ros::NodeHandle &nh);

int set_pose_body(double x, double y, double z, double yaw); // flu meters rad. yaw = 0 when not used. x=y=z=0 when not used.

int set_speed_body(double x, double y, double z, double yaw_rate); // flu meter/s rad/s, must set continously, or the vehicle stops after a few seconds(failsafe feature). yaw_rate = 0 when not used.

int set_speed_enu(double x, double y, double z, double yaw_rate); // flu meter / s rad / s,must set continously, or the vehicle stops after a few seconds(failsafe feature).yaw_rate = 0 when not used.

int set_break();

// 上锁
int arm_hand(ros::NodeHandle &nh);

// 解锁
int arm_drone(ros::NodeHandle &nh);

// 舵机
int set_servo(ros::NodeHandle &nh, int check_code, int state);

// 切换OFFBOARD
void offboard_drone(ros::NodeHandle &nh);

// void set_break_2D_speed()

void void_seeker_pose(double x, double y, double z, double yaw);

void void_seeker_speed(double x, double y, double z, double yaw);

void void_seeker(double x, double y, double z, double yaw);

void void_seeker_continus_pose(double x, double y, double z);

void void_seeker_takeoff();

void void_seeker_land(double x, double y, double z, double yaw);

void doPerson(const drone_test::Y_Serial_port::ConstPtr &person_p) // 接收串口数据
{
    uart_control_rx = *person_p;
    ROS_INFO("%d, %d, %d, %d", (int)person_p->mode, (int)person_p->tar1, (int)person_p->tar2, (int)person_p->if_takeoff);
}

void doOpencv_sub(const drone_test::detection::ConstPtr &msg)
{
    opencv_erro_x = (double)(msg->erro_x);
    opencv_erro_y = (double)(msg->erro_y);

    opencv_erro_x = opencv_erro_x / 300.0 * max_vel_xy; // 相当于归一化
    opencv_erro_y = opencv_erro_y / 300.0 * max_vel_xy;

    // 限幅输出
    if (opencv_erro_x >= max_vel_xy)
    {
        opencv_erro_x = max_vel_xy;
    }
    if (opencv_erro_x <= -1.0 * max_vel_xy)
    {
        opencv_erro_x = -1.0 * max_vel_xy;
    }
    if (opencv_erro_y >= max_vel_xy)
    {
        opencv_erro_y = max_vel_xy;
    }
    if (opencv_erro_y <= -1.0 * max_vel_xy)
    {
        opencv_erro_y = -1.0 * max_vel_xy;
    }
    opencv_flag = msg->flag;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    if (!flag_tf)
    {
        now_position[0] = current_pose.pose.position.x;
        now_position[1] = current_pose.pose.position.y;
        now_position[2] = current_pose.pose.position.z;
        now_yaw = tf::getYaw(current_pose.pose.orientation) * 180.0 / 3.1415926;
        // ROS_INFO("NOT CHECK x_pos=%f, y_pos=%f, z_pos=%f, yaw=%f", now_position[0], now_position[1], now_position[2], now_yaw);
    }
    else
    {
        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener listener(buffer);

        geometry_msgs::PointStamped point_laser;
        geometry_msgs::PointStamped point_base;

        point_laser.header.frame_id = "camera_init";
        point_laser.point.x = current_pose.pose.position.x;
        point_laser.point.y = current_pose.pose.position.y;
        point_laser.point.z = current_pose.pose.position.z;
        int tf_inside = 1;
        while (tf_inside && ros::ok())
        {
            try
            {
                /* code */
                point_base = buffer.transform(point_laser, "world");
                now_position[0] = point_base.point.x;
                now_position[1] = point_base.point.y;
                now_position[2] = point_base.point.z;
                now_yaw = tf::getYaw(current_pose.pose.orientation) * 180.0 / 3.1415926;
                tf_inside = 0;
                
                if (send_flag == 1)// 回传坐标给小车，其实一开始我是写在投放那边的，但是一直有别的地方的问题误以为是这个的问题，所以放过来了
                {
                    geometry_msgs::TwistWithCovarianceStamped twist_send;

                    twist_send.twist.twist.linear.x = now_position[0];
                    twist_send.twist.twist.linear.y = now_position[1];
                    twist_send.twist.twist.linear.z = 1;

                    ROS_INFO("x_now=%.2lf,y_now=%2.lf", now_position[0], now_position[1]);
                    ROS_INFO("x_now=%.2lf,y_now=%2.lf", now_position[0], now_position[1]);
                    ROS_INFO("x_now=%.2lf,y_now=%2.lf", now_position[0], now_position[1]);

                    camera_speed_publisher.publish(twist_send);
                    camera_speed_publisher.publish(twist_send);
                    camera_speed_publisher.publish(twist_send);
                    camera_speed_publisher.publish(twist_send);
                    send_flag = 0;
                }

                //ROS_INFO("CHECKED x_pos=%f, y_pos=%f, z_pos=%f, yaw=%f", now_position[0], now_position[1], now_position[2], now_yaw);
            }
            catch (const std::exception &e)
            {
                // std::cerr << e.what() << '\n';
            }
        }
    }
}

void get_speed(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_speed = *msg;
    now_speed[0] = current_speed.twist.twist.linear.x;
    now_speed[1] = current_speed.twist.twist.linear.y;
    now_speed[2] = current_speed.twist.twist.linear.z;
    // ROS_INFO("now_speed_x:%f,now_speed_y:%f", now_speed[0], now_speed[1]);
}

void get_Alt(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_Alt = *msg;
    mixed_alt = current_Alt.twist.twist.linear.y;
    // ROS_INFO("now_position[2]:%f", now_position[2]);
}

void PosAlt_state_cb(const mavros_msgs::DebugValue::ConstPtr &msg) // 状态机
{
    PosAlt_atate = *msg;
    Pos_mode = (int)PosAlt_atate.data[0];
    Alt_mode = (int)PosAlt_atate.data[1];
    is_RC_control = (int)PosAlt_atate.data[2];
    // ROS_INFO("Pos_mode::%d,Alt_mode::%d", Pos_mode, Alt_mode);
}

void state_cb(const mavros_msgs::State::ConstPtr &msg) // 飞控状态
{
    current_state = *msg;
}

int check_path()
{
    for (int i = 0; i < 6; i++)
    {
        ros::spinOnce();
        if (abs(now_position[0] - x_pos_neat[i]) < 1 && abs(now_position[1] - y_pos_neat[i]) < 1)
        {
            // ROS_INFO("X=%.2lf,Y=%.2lf", x_pos_neat[i], y_pos_neat[i]);
            return i;
        }
    }
    return -1;
}

/*----------------------------------------------------------------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "star_drone");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    /*----------------------------------------------------pid例化------------------------------------------------------------------------ */
    // pid_controller_set_break_speed.setParameters(1.0, 0.0, 0.0, -1 * max_vel_xy, max_vel_xy);
    pid_controller_set_xy_pose_local.setParameters(1.5, 0.0, 0.3, -1 * max_vel_xy, max_vel_xy);
    pid_controller_set_contiue_pose.setParameters(1.0, 0.0, 0.1, -1 * max_vel_dis_xy, max_vel_dis_xy);
    pid_controller_set_yaw_pose_local.setParameters(1.0, 0.0, 0.0, -1 * max_vel_yaw, max_vel_yaw);
    pid_controller_set_z_pose_local.setParameters(1, 0.0, 0.0, -1 * max_vel_z, max_vel_z);
    /*----------------------------------------------------------------------------------------------------------------------------------- */
    // 飞控基础信息订阅
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);
    ros::Subscriber PA_state_sub = nh.subscribe<mavros_msgs::DebugValue>("/mavros/debug_value/debug_vector", 1, PosAlt_state_cb);
    ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1, pose_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1, get_speed);
    ros::Subscriber currentAlt = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 1, get_Alt);
    /*----------------------------------------------------------------------------------------------------------------------------------- */
    // 图像信息订阅
    ros::Subscriber sub = nh.subscribe<drone_test::Y_Serial_port>("serial_Chip", 10, doPerson);
    ros::Subscriber Opencv_sub = nh.subscribe<drone_test::detection>("detection_fire", 1, doOpencv_sub);
    /*----------------------------------------------------------------------------------------------------------------------------------- */
    // 发布信息
    ros::Publisher dete_pub = nh.advertise<drone_test::detection>("detection", 10);
    uart_control_pub = nh.advertise<drone_test::Y_Serial_port>("Nuc", 100);
    set_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    camera_speed_publisher = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/mavros/vision_speed/speed_twist_cov", 1);
    /*----------------------------------------------------------------------------------------------------------------------------------- */
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    /*----------------------------------------------------------------------------------------------------------------------------------- */

    ros::AsyncSpinner spinner(0);//自适应创建多线程刷新回调函数
    spinner.start();

    ROS_INFO("Prepareing...");

    while (ros::ok && uart_control_rx.if_takeoff != 1)// 等待按键输入起飞
    {
        ros::spinOnce();
        rate.sleep();
    }

    flag_tf = 1;
    ros::Duration(10).sleep();

    /*------------------------------------------------------飞控自检----------------------------------------------------------------------- */
    if (1) // WANGING!!!
    {
        do
        {
            offboard_drone(nh);
            ros::Duration(1).sleep();
            ros::spinOnce();
        } while (current_state.mode != "OFFBOARD");
        ROS_INFO("Suc OFFBOARD");
        do
        {
            ros::Duration(1).sleep();
            ros::spinOnce();
        } while (ros::ok() && !current_state.connected);
        ROS_INFO("Suc Connect");
        do
        {
            arm_drone(nh);
            ros::Duration(1).sleep();
            ros::spinOnce();
        } while (!current_state.armed);
        ROS_INFO("Suc Armed");
    }
    ROS_INFO("Ready to Takeoff");
    
    /*-----------------------保存起飞时的位置------------------------ */
    ros::spinOnce();
    double x_takeoff = now_position[0];
    double y_takeoff = now_position[1];
    yaw_takeoff = now_yaw; 

    /*-------------------------------------------------------起飞--------------------------------------------------------------------------- */
    ros::spinOnce();
    takeoff(nh, 1.6); // meters
    ros::Duration(0.5).sleep();
    do
    {
        rate.sleep();
        ros::spinOnce();
    } while (ros::ok() && Alt_mode != Position_ControlMode_Position);

    ros::spinOnce();
    double z_takeoff = now_position[2];
    ROS_INFO("Z=%.2lf", z_takeoff);
    ROS_INFO("Suc Takeoff");

    if (0)
    {
    }
    else
    {
        for (int ii = 0; ii < 12; ii++)
        {
            ros::spinOnce();
            ROS_ERROR("ii=%d", ii);
            ROS_ERROR("flag=%d", opencv_flag);
            void_seeker_pose(x_pos[ii] - 0.1, y_pos[ii], 0.0, yaw_takeoff);

            if (fly_flag == 1)
            {   
                /*----------------------图像接手飞行----------------------------- */
                ros::spinOnce();
                ROS_ERROR("OPENCV!!!!!!!!!!");
                ROS_ERROR("OPENCV!!!!!!!!!!");
                ROS_ERROR("OPENCV!!!!!!!!!!");
                ROS_ERROR("OPENCV!!!!!!!!!!");
                ROS_ERROR("OPENCV!!!!!!!!!!");
                ii--;
                int flag_speed = 0;
                do
                {
                    ros::spinOnce();
                    set_speed_body(-opencv_erro_y, -opencv_erro_x, 0.0, 0.0);
                    rate.sleep();
                    if (abs(opencv_erro_y) < 0.1 && abs(opencv_erro_x) < 0.1)
                        flag_speed++;
                    else
                        flag_speed = 0;

                } while (flag_speed < 10);

                fly_flag = 2;
                set_pose_body(0, 0, 1 - mixed_alt, 0);// 下降高度，准备投放
                ros::Duration(0.5).sleep();
                do
                {
                    rate.sleep();
                    ros::spinOnce();
                } while (ros::ok() && Alt_mode != Position_ControlMode_Position);

                set_pose_body(0.5, 0, 0, 0);
                ros::Duration(0.5).sleep();// 往前飞0.5m的补偿距离，使得投放仓对准火源
                do
                {
                    rate.sleep();
                    ros::spinOnce();
                } while (ros::ok() && Pos_mode != Position_ControlMode_Position);

                send_flag = 1;// 发送坐标给小车，被改到坐标回调函数里面了

                /*--------------------------投放------------------------------- */
                ros::spinOnce();
                ros::Duration(2).sleep();
                uart_pub(13, 0, 0, 0);
                ros::Duration(0.5).sleep();
                uart_pub(13, 0, 0, 0);、
                /*------------------------------------------------------------- */

                set_pose_body(0, 0, 1.6 - mixed_alt, 0);// 回到巡航高度，继续后续任务
                ros::Duration(0.5).sleep();
                do
                {
                    rate.sleep();
                    ros::spinOnce();
                } while (ros::ok() && Alt_mode != Position_ControlMode_Position);
            }
        }

        ros::spinOnce();
        void_seeker_land(0.5, -0.5, 0.05, yaw_takeoff);// 贴地虚空索敌校准起飞点

    }

    /*---------------------------------------------------------降落--------------------------------------------------------------------- */
    ROS_INFO("Start to Land");
    land(nh);
    ros::Duration(0.5).sleep();
    do
    {
        rate.sleep();
        ros::spinOnce();
    } while (ros::ok() && Alt_mode != Position_ControlMode_Position);

    /*---------------------------------------------------------上锁--------------------------------------------------------------------- */
    do
    {
        arm_hand(nh);
        rate.sleep();
        ros::spinOnce();
    } while (ros::ok() && current_state.armed);

    return 0;
}


int takeoff(ros::NodeHandle &nh, double height) //起飞函数 meters
{
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = height;
    if (takeoff_cl.call(srv_takeoff))
    {
        ROS_INFO("Takeoff sent %d", srv_takeoff.response.success);
        flag_tf = 1;
    }
    else
    {
        ROS_ERROR("Failed Takeoff");
        return -1;
    }
    return 0;
}

int land(ros::NodeHandle &nh)// 原地降落
{
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    if (land_client.call(srv_land) && srv_land.response.success)
        ROS_INFO("land sent %d", srv_land.response.success);
    else
    {
        ROS_ERROR("Landing failed");
        ros::shutdown();
        return -1;
    }
    return 0;
}

int set_pose_body(double x, double y, double z, double yaw) // body坐标系相对位置飞行
{
    // flu meters rad. yaw = 0 when not used. x=y=z=0 when not used.
    ROS_INFO("Set_Pos: X=%.2lf,Y=%.2lf,Z=%.2lf", x, y, z);
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
    raw_target.type_mask = PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW_RATE;

    if (fabs(yaw) < 1e-6)
        raw_target.type_mask |= PositionTarget::IGNORE_YAW;
    else
        raw_target.yaw = yaw * 0.01745329;

    raw_target.position.x = x;
    raw_target.position.y = y;
    raw_target.position.z = z;

    raw_target.header.stamp = ros::Time::now();
    set_raw_pub.publish(raw_target);
    ROS_INFO("Set_Pos: 2");
    ros::spinOnce();
    ROS_INFO("Set_Pos: 3");
    return 0;
}

int set_speed_body(double x, double y, double z, double yaw_rate) // body坐标系相对速度飞行
{
    // flu meter/s rad/s, must set continously, or the vehicle stops after a few seconds(failsafe feature). yaw_rate = 0 when not used.
    if (true)// 最大速度限幅
    {
        if (x > max_vel_xy) x = max_vel_xy;
        if (x < -1*max_vel_xy) x = -1*max_vel_xy;
        if (y > max_vel_xy) y = max_vel_xy;
        if (y < -1*max_vel_xy) y = -1*max_vel_xy;
        if (z > max_vel_z) z = max_vel_z;
        if (z < -1*max_vel_z) z = -1*max_vel_z;
        if (yaw_rate > max_vel_yaw) yaw_rate = max_vel_yaw;
        if (yaw_rate < -1*max_vel_yaw) yaw_rate = -1*max_vel_yaw;
    }
    
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
    raw_target.type_mask = PositionTarget::IGNORE_PX | PositionTarget::IGNORE_PY | PositionTarget::IGNORE_PZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW;
    if (fabs(yaw_rate) < 1e-6)
        raw_target.type_mask |= PositionTarget::IGNORE_YAW_RATE;
    raw_target.velocity.x = x;
    raw_target.velocity.y = y;
    raw_target.velocity.z = z;
    raw_target.yaw_rate = yaw_rate * 0.01745329;
    // ROS_INFO("Set_Vel x=%.2lf,y=%.2lf,z=%.2lf,yaw=%.2lf", x, y, z, yaw_rate);
    set_raw_pub.publish(raw_target);
    return 0;
}

int set_speed_enu(double x, double y, double z, double yaw_rate)// 全局enu坐标系速度飞行 
{
    // flu meter / s rad / s,must set continously, or the vehicle stops after a few seconds(failsafe feature).yaw_rate = 0 when not used.
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
    raw_target.type_mask = PositionTarget::IGNORE_PX | PositionTarget::IGNORE_PY | PositionTarget::IGNORE_PZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW;
    if (fabs(yaw_rate) < 1e-6)
        raw_target.type_mask |= PositionTarget::IGNORE_YAW_RATE;
    raw_target.velocity.x = x;
    raw_target.velocity.y = y;
    raw_target.velocity.z = z;
    raw_target.yaw_rate = yaw_rate * 0.01745329;
    set_raw_pub.publish(raw_target);
    return 0;
}

int set_break()// 刹车
{
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
    raw_target.type_mask = PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW_RATE;
    raw_target.position.x = 0;
    raw_target.position.y = 0;
    raw_target.position.z = 0;
    raw_target.yaw = 0;
    set_raw_pub.publish(raw_target);

    ros::spinOnce();
    ros::Duration(0.1).sleep();
    return 0;
}

int arm_hand(ros::NodeHandle &nh)// 上锁
{
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false; // true解锁 false上锁
    arming_client.call(arm_cmd);
    if (arm_cmd.response.success)
        ROS_INFO("Vehicle armed");
    return arm_cmd.response.success;
}

int arm_drone(ros::NodeHandle &nh)// 解锁
{
    // arming
    ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool srv_arm_i;
    srv_arm_i.request.value = true;
    if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
        ROS_INFO("ARM sent %d", srv_arm_i.response.success);
    else
    {
        ROS_ERROR("Failed arming");
        return -1;
    }
    return 0;
}

int set_servo(ros::NodeHandle &nh, int check_code, int state)// 舵机
{
    ros::ServiceClient servo_client_i = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
    mavros_msgs::CommandLong msg;
    msg.request.command = mavros_msgs::CommandCode::DO_SET_SERVO;
    msg.request.param1 = check_code;
    msg.request.param2 = state;

    if (servo_client_i.call(msg) && msg.response.success)
        ROS_INFO("servo msg sent");
    else
    {
        ROS_ERROR("Failed servo");
        return -1;
    }
    return 0;
}

void offboard_drone(ros::NodeHandle &nh)// 切换OFFBOARD
{
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD"; // 设置模式为 OFFBOARD
    ros::spinOnce();
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("Offboard enabled"); // 如果请求服务成功且服务返回执行成功打印消息
    }
}

void uart_pub(int mode, int tar1, int tar2, int if_takeoff)//接受按键扩展板串口数据
{
    drone_test::Y_Serial_port uart_control_tx;
    uart_control_tx.if_takeoff = if_takeoff;
    uart_control_tx.mode = mode;
    uart_control_tx.tar1 = tar1;
    uart_control_tx.tar2 = tar2;
    ROS_INFO("Haved send");
    uart_control_pub.publish(uart_control_tx);
}

// void set_break_2D_speed()// 2D刹车
// {
//     int flag_stop = 0;
//     do
//     {
//         ros::spinOnce();
//         double break_speed_x = pid_controller_set_break_speed.update(0, now_speed[0]);
//         double break_speed_y = pid_controller_set_break_speed.update(0, now_speed[1]);
//         set_speed_body(break_speed_x, break_speed_y, 0, 0);
//         ros::Duration(0.1).sleep();
//         ros::spinOnce();
//         if (abs(now_speed[0]) < 0.05 && abs(now_speed[1]) < 0.05)
//         {
//             flag_stop++;
//         }
//         else
//         {
//             flag_stop = 0;
//         }
//     } while (flag_stop < 10 && ros::ok());
//     set_break();
//     do
//     {
//         ros::Duration(0.05).sleep();
//         ros::spinOnce();
//     } while (ros::ok() && (Pos_mode != Position_ControlMode_Position || Alt_mode != Position_ControlMode_Position));
// }

void void_seeker_land(double x, double y, double z, double yaw)// 其实和后面的void_seeker_pose一样，，以前的话是把目标点对准起飞点，就是做坐标变换的目标点写死了，国赛改了而已
{
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    static geometry_msgs::PointStamped point_laser;
    point_laser.header.frame_id = "world";
    point_laser.header.stamp = ros::Time();
    point_laser.point.x = x;
    point_laser.point.y = y;
    point_laser.point.z = z;
    static geometry_msgs::PointStamped point_base;
    int flag_tf_seeker_pose = 1;// 等待坐标变换
    while (flag_tf_seeker_pose && ros::ok())
    {
        try
        {
            /* code */
            point_base = buffer.transform(point_laser, "YYY_see_body");
            flag_tf_seeker_pose = 0;
            //ROS_INFO("(%.2f,%.2f,%.2f)", point_base.point.x, point_base.point.y, point_base.point.z);
            set_pose_body(point_base.point.x, point_base.point.y, point_base.point.z, 0.0);
            ROS_INFO("Void_seeker_Pos");
            ros::Duration(0.5).sleep();
            do
            {
                ros::Duration(0.1).sleep();
                ros::spinOnce();
            } while (ros::ok() && (Pos_mode != Position_ControlMode_Position || Alt_mode != Position_ControlMode_Position));
        }
        catch (const std::exception &e)
        {
            // std::cerr << e.what() << '\n';
        }
    }
}

void void_seeker_pose(double x, double y, double z, double yaw)// 航点飞行的虚空索敌，每两个航点之间的最大偏差大概是10m左右偏离10cm，yaw角的补偿我感觉已经尽力了，真正飞行过程就算转圈也不会影响真正的飞行，相对坐标的补偿是一个航点一算
{
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    static geometry_msgs::PointStamped point_laser;
    point_laser.header.frame_id = "world";
    point_laser.header.stamp = ros::Time();
    point_laser.point.x = x;
    point_laser.point.y = y;
    point_laser.point.z = z;
    static geometry_msgs::PointStamped point_base;
    int flag_tf_seeker_pose = 1;
    while (flag_tf_seeker_pose && ros::ok())
    {
        ros::spinOnce();
        try
        {
            /* code */
            ros::spinOnce();
            point_base = buffer.transform(point_laser, "YYY_see_body");
            flag_tf_seeker_pose = 0;
            ROS_INFO("(%.2f,%.2f,%.2f)", point_base.point.x, point_base.point.y, point_base.point.z);
            set_pose_body(point_base.point.x, point_base.point.y, 0.0, yaw - now_yaw);
            ROS_INFO("Void_seeker_Pos");
            ros::Duration(0.5).sleep();
            do
            {
                ros::spinOnce();
                if (fly_flag == 0 && check_path() > -1)
                {
                    //ROS_INFO("error_x=%.2lf,error_y=%.2lf,opencv_flag=%d", opencv_erro_x, opencv_erro_y, opencv_flag);
                    if (opencv_flag == 1 && abs(opencv_erro_x) > 0.1 && abs(opencv_erro_y) > 0.1)
                    {
                        set_break();// 这个函数其实刹不住车的，真想虚空悬停得用那个2D刹车，但是没调好，需要校准一会，没必要浪费这个时间，只为了让下位机切一下状态机
                        do
                        {
                            ros::spinOnce();
                            ROS_INFO("Start_OP!!!!!!!!");
                            ros::Duration(0.1).sleep();
                            ros::spinOnce();
                            fly_flag = 1;
                        } while (ros::ok() && (Pos_mode != Position_ControlMode_Position || Alt_mode != Position_ControlMode_Position));
                        uart_pub(12, 0, 0, 0);// 亮灯
                        ros::Duration(0.5).sleep();
                        uart_pub(12, 0, 0, 0);
                        return;
                    }
                }
                ros::Duration(0.1).sleep();
                ros::spinOnce();
            } while (ros::ok() && (Pos_mode != Position_ControlMode_Position) && (abs(now_position[0] - x) > 0.05 || abs(now_position[1] - y) > 0.05));
        }
        catch (const std::exception &e)
        {
            // std::cerr << e.what() << '\n';
        }
    }
}

void void_seeker_speed(double x, double y, double z, double yaw)// 真正的虚空索敌，但是没写压线飞行，所以存在飞行轨迹是曲线，以及pid没调好的问题。校准时间太久了，大概得3~5s校准一个航点，但是理论上没有距离限制，最大速度没限制，调好了无敌
{
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    static geometry_msgs::PointStamped point_laser;
    point_laser.header.frame_id = "YYY_see_body_takeoff";
    point_laser.header.stamp = ros::Time();
    point_laser.point.x = x;
    point_laser.point.y = y;
    point_laser.point.z = z;
    static geometry_msgs::PointStamped point_base;
    int flag_stop = 0;
    do
    {
        try
        {
            /* code */
            ros::spinOnce();
            point_base = buffer.transform(point_laser, "YYY_see_body");
            ROS_INFO("xxx=%.2f,yyy=%.2f", point_base.point.x, point_base.point.y);
            double break_speed_x = -pid_controller_set_xy_pose_local.update(0, point_base.point.x);
            double break_speed_y = -pid_controller_set_xy_pose_local.update(0, point_base.point.y);
            // double break_speed_z = -pid_controller_set_z_pose_local.update(0, point_base.point.z);
            // double break_speed_yaw = pid_controller_set_yaw_pose_local.update(yaw, now_yaw);
            ROS_INFO("Void_seeker_Vel");
            set_speed_body(break_speed_x, break_speed_y, 0, 0);
            // set_speed_body(break_speed_xy / sqrt(xxx * xxx + yyy * yyy) * xxx, break_speed_xy / sqrt(xxx * xxx + yyy * yyy) * yyy, 0.0, 0.0);
            ros::Duration(0.05).sleep();
            ros::spinOnce();
            if (abs(point_base.point.x) < min_stop_error && abs(point_base.point.y) < min_stop_error) flag_stop++;
            else flag_stop = 0;
        }
        catch (const std::exception &e)
        {
            // std::cerr << e.what() << '\n';
        }
    } while (flag_stop < 20 && ros::ok());

    pid_controller_set_xy_pose_local.clear();
    // pid_controller_set_yaw_pose_local.clear();
    // pid_controller_set_z_pose_local.clear();
}

void void_seeker(double x, double y, double z, double yaw)// 位置飞压线，速度校准最后30cm，是投机取巧，但可以解决长距离飞行轨迹的问题，校准也很快，因为进入速度较准时候，pid值不会被截断式的限制
{

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    static geometry_msgs::PointStamped point_laser;
    point_laser.header.frame_id = "YYY_see_body_takeoff";
    point_laser.header.stamp = ros::Time();
    point_laser.point.x = x;
    point_laser.point.y = y;
    point_laser.point.z = z;
    static geometry_msgs::PointStamped point_base;
    int flag_tf_seeker = 1;
    while (flag_tf_seeker && ros::ok())
    {
        try
        {
            /* code */
            point_base = buffer.transform(point_laser, "YYY_see_body");
            ROS_INFO("(%.2f,%.2f,%.2f)", point_base.point.x, point_base.point.y, point_base.point.z);
            ROS_INFO("Void_seeker_Pos");
            flag_tf_seeker = 0;
            set_pose_body(point_base.point.x, point_base.point.y, point_base.point.z, 0.0);
            ros::Duration(0.5).sleep();
            do
            {
                ros::Duration(0.01).sleep();
                ros::spinOnce();
            } while (ros::ok() && ((abs(point_base.point.x) > 0.3) || (abs(point_base.point.y) > 0.3) || abs(point_base.point.z) > 0.3) && (Alt_mode != Position_ControlMode_Position || Pos_mode != Position_ControlMode_Position));
        }
        catch (const std::exception &e)
        {
            // std::cerr << e.what() << '\n';
        }
    }

    int flag_stop = 0;
    do
    {
        try
        {
            /* code */
            ros::spinOnce();
            point_base = buffer.transform(point_laser, "YYY_see_body");
            double xxx = point_base.point.x;
            double yyy = point_base.point.y;
            double break_speed_xy = -pid_controller_set_xy_pose_local.update(0, sqrt(xxx * xxx + yyy * yyy));

            set_speed_body(break_speed_xy / sqrt(xxx * xxx + yyy * yyy) * xxx, break_speed_xy / sqrt(xxx * xxx + yyy * yyy) * yyy, 0.0, 0.0);
            ros::Duration(0.01).sleep();
            ros::spinOnce();
            if (abs(x - now_position[0]) < min_stop_error && abs(y - now_position[1]) < min_stop_error) flag_stop++;
            else flag_stop = 0;
        }
        catch (const std::exception &e)
        {
            // std::cerr << e.what() << '\n';
        }
    } while (flag_stop < 5 && ros::ok());

    pid_controller_set_xy_pose_local.clear();
}

void void_seeker_takeoff()// 起飞，后来单独写了一个代码tfcpp_takeoff.cpp，这个没用了
{
    // ros::spinOnce();
    // static tf2_ros::StaticTransformBroadcaster broadcaster;
    // static tf2_ros::StaticTransformBroadcaster broadcaster_world;
    // //  5-2.创建 广播的数据(通过 pose 设置)
    // geometry_msgs::TransformStamped tfs;
    // geometry_msgs::TransformStamped tfs_world;
    // //  |----头设置
    // tfs.header.frame_id = "camera_init";

    // tfs.header.stamp = ros::Time::now();

    // //  |----坐标系 ID
    // tfs.child_frame_id = "YYY_see_body_takeoff";

    // //  |----坐标系相对信息设置
    // // ros::spinOnce();
    // tfs.transform.translation.x = current_pose.pose.position.x;
    // tfs.transform.translation.y = current_pose.pose.position.y;
    // tfs.transform.translation.z = current_pose.pose.position.z;
    // tfs.transform.rotation.x = current_pose.pose.orientation.x;
    // tfs.transform.rotation.y = current_pose.pose.orientation.y;
    // tfs.transform.rotation.z = current_pose.pose.orientation.z;
    // tfs.transform.rotation.w = current_pose.pose.orientation.w;
    // broadcaster.sendTransform(tfs);

    // tfs_world.header.frame_id = "camera_init";
    // tfs_world.header.stamp = ros::Time::now();
    // tfs_world.child_frame_id = "world";
    // tfs_world.transform.translation.x = current_pose.pose.position.x - 0.4;
    // tfs_world.transform.translation.y = current_pose.pose.position.y + 0.4;
    // tfs_world.transform.translation.z = current_pose.pose.position.z;
    // tfs_world.transform.rotation.x = current_pose.pose.orientation.x;
    // tfs_world.transform.rotation.y = current_pose.pose.orientation.y;
    // tfs_world.transform.rotation.z = current_pose.pose.orientation.z;
    // tfs_world.transform.rotation.w = current_pose.pose.orientation.w;
    // broadcaster_world.sendTransform(tfs_world);
    // flag_tf = 1;// 开始坐标变换，原地留一个坐标系，用于虚空索敌的参考
    // ROS_INFO("tf_takeoff");
}

void void_seeker_continus_pose(double x, double y, double z)// 连续的pose型虚空索敌，可压线飞行
{
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    static geometry_msgs::PointStamped point_laser;
    point_laser.header.frame_id = "YYY_see_body_takeoff";
    point_laser.header.stamp = ros::Time();
    point_laser.point.x = x;
    point_laser.point.y = y;
    point_laser.point.z = z;
    static geometry_msgs::PointStamped point_base;
    int flag_stop = 0;
    do
    {
        try
        {
            /* code */
            ros::spinOnce();
            point_base = buffer.transform(point_laser, "YYY_see_body");
            double break_pose_x = -pid_controller_set_contiue_pose.update(0, point_base.point.x);
            double break_pose_y = -pid_controller_set_contiue_pose.update(0, point_base.point.y);
            double break_pose_z = -pid_controller_set_contiue_pose.update(0, point_base.point.z);
            set_pose_body(break_pose_x, break_pose_y, break_pose_z, 0);
            ros::Duration(1).sleep();
            ros::spinOnce();
            if (abs(break_pose_x) < min_stop_error && abs(break_pose_y) < min_stop_error && abs(break_pose_z) < min_stop_error)
            {
                flag_stop++;
            }
            else
            {
                flag_stop = 0;
            }
        }
        catch (const std::exception &e)
        {
            // std::cerr << e.what() << '\n';
        }
    } while (flag_stop < 10 && ros::ok());
    pid_controller_set_contiue_pose.clear();
}
