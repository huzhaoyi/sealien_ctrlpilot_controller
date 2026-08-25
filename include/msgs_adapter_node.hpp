#ifndef _MSGS_ADAPTER_NODE_HPP
#define _MSGS_ADAPTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sealien_ctrlpilot_msgmanagement/msg/sonar_altimeter_status.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/elb105_shzr04.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/depth_status.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include "nav_msgs/msg/odometry.hpp"

class MsgAdapter : public rclcpp::Node {
public:
  MsgAdapter();

private:
  void timer_callback();
  void sonar_callback(const sealien_ctrlpilot_msgmanagement::msg::SonarAltimeterStatus& msg);
  void imu_callback(const sealien_ctrlpilot_msgmanagement::msg::Elb105Shzr04& msg);
  void depth_callback(const sealien_ctrlpilot_msgmanagement::msg::DepthStatus& msg);
  void resetRef_callback(const std_msgs::msg::Bool& msg);
  void get_params();
  double Trans2LocatCoordinate(const double& angle);

  rclcpp::TimerBase::SharedPtr timer_;
  GeographicLib::LocalCartesian origin_ref;   //创建一个LocalCartesian对象，用于将经纬度转换为局部笛卡尔坐标
  bool restRef_flag; //重置参考位置标志量
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  double lat; //参考精度
  double lon; //参考精度
  double alt; //参考精度

  sensor_msgs::msg::Imu imu_data;
  nav_msgs::msg::Odometry rov_odom;
  geometry_msgs::msg::Vector3 cur_imu_pos;   //当前imu相对位置
  geometry_msgs::msg::TwistWithCovarianceStamped dvl_data;
  geometry_msgs::msg::PoseWithCovarianceStamped depth_data;
  geometry_msgs::msg::PoseWithCovarianceStamped sonar_data;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr RovOdom_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sonar_publisher;    //发布sonar/pose
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_origin_publisher;    //发布航向角

  rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::SonarAltimeterStatus>::SharedPtr sonar_subscriber;   //订阅声呐数据
  rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::Elb105Shzr04>::SharedPtr imu_subscriber;   //订阅imu数据,包括dvl
  rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::DepthStatus>::SharedPtr depth_subscriber;   //订阅深度计数据
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resetRef_subscriber;   //订阅深度计数据

}; 


#endif // _MSGS_ADAPTER_NODE_HPP