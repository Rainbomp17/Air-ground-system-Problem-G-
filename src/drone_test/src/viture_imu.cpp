#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

ros::Publisher mavros_imu_pub;

void t265ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensor_msgs::Imu mavros_imu;
    mavros_imu.header = msg->header;
    mavros_imu.linear_acceleration = msg->linear_acceleration;
    mavros_imu.angular_velocity = msg->angular_velocity;

    mavros_imu_pub.publish(mavros_imu);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "t265_imu_publisher");
    ros::NodeHandle nh;

    mavros_imu_pub = nh.advertise<sensor_msgs::Imu>("/mavros/imu/data_raw", 10);

    ros::Subscriber t265_imu_sub = nh.subscribe<sensor_msgs::Imu>("/camera/imu", 10, t265ImuCallback);

    ros::spin();

    return 0;
}
