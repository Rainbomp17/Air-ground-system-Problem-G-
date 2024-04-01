#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/LandingTarget.h>
#include <nav_msgs/Odometry.h>

#include <string.h>

#define TF2_EULER_DEFAULT_ZYX
geometry_msgs::TwistWithCovariance camera_twist;
void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
  camera_twist = (*msg).twist;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mid360_to_mavros");

  ros::NodeHandle node;

  //////////////////////////////////////////////////
  // Variables for precision navigation
  //////////////////////////////////////////////////

  ros::Publisher camera_pose_publisher = node.advertise<geometry_msgs::PoseStamped>("mid360_pose", 1);

  // ros::Publisher camera_speed_publisher = node.advertise<geometry_msgs::TwistWithCovarianceStamped>("mid360_speed", 1);

  ros::Subscriber odom_sub = node.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1, odom_cb);

  ros::Publisher body_path_pubisher = node.advertise<nav_msgs::Path>("body_frame/path", 1);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::TransformStamped transform_stamped;

  geometry_msgs::PoseStamped msg_body_pose;

  nav_msgs::Path body_path;

  std::string target_frame_id = "base_link"; // 底座坐标系

  std::string source_frame_id = "camera_init"; // 激光雷达坐标系

  double output_rate = 10, roll_lidar = 0.0, pitch_lidar = 0.0, yaw_lidar = 0.0;
  double x_lidar = 0.0, y_lidar = 0.0, z_lidar = 0.0, gamma_world = -1.5707963;

  // Read parameters from launch file, including: target_frame_id, source_frame_id, output_rate
  {
    // The frame in which we find the transform into, the original "world" frame
    if (node.getParam("target_frame_id", target_frame_id))
    {
      ROS_INFO("Get target_frame_id parameter: %s", target_frame_id.c_str());
    }
    else
    {
      ROS_WARN("Using default target_frame_id: %s", target_frame_id.c_str());
    }

    // The frame for which we find the tranform to target_frame_id, the original "lidar" frame
    if (node.getParam("source_frame_id", source_frame_id))
    {
      ROS_INFO("Get source_frame_id parameter: %s", source_frame_id.c_str());
    }
    else
    {
      ROS_WARN("Using default source_frame_id: %s", source_frame_id.c_str());
    }

    // The rate at which we wish to publish final pose data
    if (node.getParam("output_rate", output_rate))
    {
      ROS_INFO("Get output_rate parameter: %f", output_rate);
    }
    else
    {
      ROS_WARN("Using default output_rate: %f", output_rate);
    }

    // The roll angle around lidar's own axis to align with body frame
    if (node.getParam("roll_lidar", roll_lidar))
    {
      ROS_INFO("Get roll_lidar parameter: %f", roll_lidar);
    }
    else
    {
      ROS_WARN("Using default roll_lidar: %f", roll_lidar);
    }

    // The pitch angle around lidar's own axis to align with body frame
    if (node.getParam("pitch_lidar", pitch_lidar))
    {
      ROS_INFO("Get pitch_lidar parameter: %f", pitch_lidar);
    }
    else
    {
      ROS_WARN("Using default pitch_lidar: %f", pitch_lidar);
    }

    // The yaw angle around lidar's own axis to align with body frame
    if (node.getParam("yaw_lidar", yaw_lidar))
    {
      ROS_INFO("Get yaw_lidar parameter: %f", yaw_lidar);
    }
    else
    {
      ROS_WARN("Using default yaw_lidar: %f", yaw_lidar);
    }

    // The lidar's X axis offset to align with body frame
    if (node.getParam("x_lidar", x_lidar))
    {
      ROS_INFO("Get x_lidar parameter: %f", x_lidar);
    }
    else
    {
      ROS_WARN("Using default x_lidar: %f", x_lidar);
    }

    // The lidar's Y axis offset to align with body frame
    if (node.getParam("y_lidar", y_lidar))
    {
      ROS_INFO("Get y_lidar parameter: %f", y_lidar);
    }
    else
    {
      ROS_WARN("Using default y_lidar: %f", y_lidar);
    }

    // The lidar's Z axis offset to align with body frame
    if (node.getParam("z_lidar", z_lidar))
    {
      ROS_INFO("Get z_lidar parameter: %f", z_lidar);
    }
    else
    {
      ROS_WARN("Using default z_lidar: %f", z_lidar);
    }

    // The rotation around z axis between original world frame and target world frame, assuming the z axis needs not to be changed
    // In this case, target world frame has y forward, x to the right and z upwards (ENU as ROS dictates)
    if (node.getParam("gamma_world", gamma_world))
    {
      ROS_INFO("Get gamma_world parameter: %f", gamma_world);
    }
    else
    {
      ROS_WARN("Using default gamma_world: %f", gamma_world);
    }
  }

  //////////////////////////////////////////////////
  // Wait for the first transform to become available.
  //////////////////////////////////////////////////
  // tf_listener.waitForTransform(target_frame_id, source_frame_id, ros::Time::now(), ros::Duration(3.0));
  transform_stamped = tf_buffer.lookupTransform(target_frame_id, source_frame_id, ros::Time(0), ros::Duration(3.0));

  ros::Time last_tf_time = ros::Time::now();

  // Limit the rate of publishing data, otherwise the other telemetry port might be flooded
  ros::Rate rate(output_rate);

  while (node.ok())
  {
    // For tf, Time(0) means "the latest available" transform in the buffer.
    ros::Time now = ros::Time(0);
    ros::Duration timeout(1.0);

    //////////////////////////////////////////////////
    // Publish vision_position_estimate message if transform is available
    //////////////////////////////////////////////////
    try
    {
      // 获取变换信息
      transform_stamped = tf_buffer.lookupTransform(target_frame_id, source_frame_id, now, timeout);

      // Only publish pose messages when we have new transform data.
      if (last_tf_time < transform_stamped.header.stamp)
      {
        ros::spinOnce();
        last_tf_time = transform_stamped.header.stamp;

        // 将ROS的geometry_msgs转换为tf2的数据类型
        tf2::Transform transform;
        tf2::fromMsg(transform_stamped.transform, transform);

        // 进行坐标系变换
        tf2::Vector3 position_body;
        tf2::Vector3 position_orig = transform.getOrigin();
        position_body.setX(cos(gamma_world) * (position_orig.getX() + x_lidar) + sin(gamma_world) * (position_orig.getY() + y_lidar));
        position_body.setY(-sin(gamma_world) * (position_orig.getX() + x_lidar) + cos(gamma_world) * (position_orig.getY() + y_lidar));
        position_body.setZ(position_orig.getZ() + z_lidar);

        // 读取旋转偏移并进行旋转
        tf2::Quaternion quat_lidar = transform.getRotation();
        tf2::Quaternion quat_body;
        quat_body.setRPY(roll_lidar, pitch_lidar, yaw_lidar);

        // Rotate body frame 90 degree (align body x with world y at launch)
        tf2::Quaternion quat_rot_z;
        quat_rot_z.setRPY(0, 0, -gamma_world);

        quat_body = quat_rot_z * quat_lidar * quat_body;

        // 归一化
        quat_body.normalize();

        // Create PoseStamped message to be sent
        msg_body_pose.header.stamp = transform_stamped.header.stamp;
        msg_body_pose.header.frame_id = transform_stamped.header.frame_id;
        msg_body_pose.pose.position.x = position_body.getX();
        msg_body_pose.pose.position.y = position_body.getY();
        msg_body_pose.pose.position.z = position_body.getZ();
        msg_body_pose.pose.orientation.x = quat_body.getX();
        msg_body_pose.pose.orientation.y = quat_body.getY();
        msg_body_pose.pose.orientation.z = quat_body.getZ();
        msg_body_pose.pose.orientation.w = quat_body.getW();

        // Publish pose of body frame in world frame
        camera_pose_publisher.publish(msg_body_pose);

        // Publish trajectory path for visualization
        body_path.header.stamp = msg_body_pose.header.stamp;
        body_path.header.frame_id = msg_body_pose.header.frame_id;
        body_path.poses.push_back(msg_body_pose);
        body_path_pubisher.publish(body_path);

        // geometry_msgs::TwistWithCovarianceStamped twist_send;
        // twist_send.header.stamp = msg_body_pose.header.stamp;
        // twist_send.header.frame_id = msg_body_pose.header.frame_id;
        // twist_send.twist.twist.linear.x = -1.0 * camera_twist.twist.linear.x;
        // twist_send.twist.twist.linear.y = -1.0 * camera_twist.twist.linear.y;
        // twist_send.twist.twist.linear.z = 1.0 * (camera_twist.twist.linear.z);
        // twist_send.twist.twist.angular.x = -1.0 * (camera_twist.twist.angular.x);
        // twist_send.twist.twist.angular.y = -1.0 * (camera_twist.twist.angular.y);
        // twist_send.twist.twist.angular.z = 1.0 * (camera_twist.twist.angular.z);
        // camera_speed_publisher.publish(twist_send);
      }
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Failed to lookup transform: %s", ex.what());
      ros::Duration(1.0).sleep();
    }

    //////////////////////////////////////////////////
    // Repeat
    //////////////////////////////////////////////////
    rate.sleep();
  }
  return 0;
}