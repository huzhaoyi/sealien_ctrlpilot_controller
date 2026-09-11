/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****
* File name          : msgs_adapter_node.cpp
* Author             : Yi Lu
* Brief              : 
********************************************************************************
* modify
* Version   Date                Author              Described
* V1.00     2026/8/25            Yi Lu               Created
* V1.01     2026/9/9             JoeyHU                INS体轴映射修正(错误Y前假设)
* V1.02     2026/9/10            JoeyHU                INS(x左/y前/z下)->base_link(x前/y左/z上); DVL恢复前/右/下
*******************************************************************************/

#include "msgs_adapter_node.hpp"

#define DEG2RAD  (M_PI / 180.0)

using std::placeholders::_1;

/*
 * base_link: x前 / y左 / z上（不变）
 * 新INS体轴: x左 / y前 / z下（右手系）
 * v_base = R * v_ins, R = [[0,1,0],[1,0,0],[0,0,-1]]  <=>  setRPY(pi, 0, pi/2)
 * DVL字段为船体语义(前/右/下)，与旧版一致直通到 twist（前->x）
 */

/********************************************************************************
 * @brief  :构造函数
 * @param  NONE
 * @return NONE
*********************************************************************************/
MsgAdapter::MsgAdapter() : Node("msg_adapter_node") {
  get_params();

  origin_ref.Reset(lat, lon, alt);  //重置原点

  sonar_subscriber = this->create_subscription<sealien_ctrlpilot_msgmanagement::msg::SonarAltimeterStatus>(
    "/SonarAltimeterStatus", 10, std::bind(&MsgAdapter::sonar_callback, this, _1));       //订阅高度计数据

  imu_subscriber = this->create_subscription<sealien_ctrlpilot_msgmanagement::msg::Elb105Shzr04>(
    "/elb105/shzr04", 10, std::bind(&MsgAdapter::imu_callback, this, _1));       //订阅imu数据，这里默认包含DVL数据，如果dvl与IMU数据是分离的，需要增加订阅dvl的内容

  depth_subscriber = this->create_subscription<sealien_ctrlpilot_msgmanagement::msg::DepthStatus>(
    "/DepthStatus", 10, std::bind(&MsgAdapter::depth_callback, this, _1));       //订阅深度计数据

  resetRef_subscriber = this->create_subscription<std_msgs::msg::Bool>("~/resetRef", 10,
     std::bind(&MsgAdapter::resetRef_callback, this, _1));       //订阅重置参考点指令

  RovOdom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/msg_adapter/rov_odom", 10);
  sonar_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/msg_adapter/sonar/pose", 10);
  yaw_origin_publisher = this->create_publisher<std_msgs::msg::Float32>("/msg_adapter/yaw_origin", 10); 


  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);  //


  timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&MsgAdapter::timer_callback, this));

  restRef_flag = true;
}

void MsgAdapter::timer_callback() {

}


void MsgAdapter::get_params() {
  //获取参考经纬度及高度
  this->declare_parameter<double>("lat", 22.80169);
  lat = this->get_parameter("lat").as_double();

  this->declare_parameter<double>("lon", 113.52497);
  lon = this->get_parameter("lon").as_double();

  this->declare_parameter<double>("alt", 0);
  alt = this->get_parameter("alt").as_double();

  // RCLCPP_INFO(this->get_logger(),"covar_sonar_alt[%f]",covar_sonar_alt);
  
}

/********************************************************************************
 * @brief  :测距声呐回调函数，目前只有一个声呐
 * @param  msg:消息
 * @return NONE
*********************************************************************************/
void MsgAdapter::sonar_callback(const sealien_ctrlpilot_msgmanagement::msg::SonarAltimeterStatus& msg){
  sonar_data.header.stamp = this->get_clock()->now();
  sonar_data.header.frame_id = "sensor_sonar";
  sonar_data.pose.pose.position.z = 0.01 * msg.near_dist_cm[0]; //转换成单位m

  sonar_publisher->publish(sonar_data);
}

/********************************************************************************
 * @brief  :imu回调函数
 * @param  msg:消息数据
 * @return NONE
*********************************************************************************/
void MsgAdapter::imu_callback(const sealien_ctrlpilot_msgmanagement::msg::Elb105Shzr04& msg)
{
  tf2::Quaternion q_ins;
  tf2::Quaternion q_offset;
  tf2::Quaternion q_base;
  std_msgs::msg::Float32 yaw1_pub;
  double locate_yaw = 0.0;
  double dvl_front_mps = 0.0;
  double dvl_right_mps = 0.0;
  double dvl_down_mps = 0.0;

  if (msg.alignment_status != 3)
  {
    return;
  }

  /* Z下航向 -> Z上yaw：取反并归一化到 (-pi, pi] */
  locate_yaw = Trans2LocatCoordinate(msg.heading_deg);

  yaw1_pub.data = msg.heading_deg;
  yaw_origin_publisher->publish(yaw1_pub);

  q_ins.setRPY(msg.roll_deg * DEG2RAD, msg.pitch_deg * DEG2RAD, locate_yaw);
  /* INS(x左/y前/z下) -> base_link(x前/y左/z上) */
  q_offset.setRPY(M_PI, 0.0, M_PI / 2.0);
  q_base = q_ins * q_offset;

  if (restRef_flag)
  {
    restRef_flag = false;
    origin_ref.Reset(msg.latitude_deg, msg.longitude_deg, rov_odom.pose.pose.position.z);
  }

  origin_ref.Forward(msg.latitude_deg, msg.longitude_deg, rov_odom.pose.pose.position.z,
      cur_imu_pos.x, cur_imu_pos.y, cur_imu_pos.z);

  rov_odom.pose.pose.orientation = tf2::toMsg(q_base);

  /* 陀螺：INS(x左/y前/z下) -> base(x前/y左/z上): (gyro_y, gyro_x, -gyro_z) */
  rov_odom.twist.twist.angular.x = msg.gyro_y_degps * DEG2RAD;
  rov_odom.twist.twist.angular.y = msg.gyro_x_degps * DEG2RAD;
  rov_odom.twist.twist.angular.z = -msg.gyro_z_degps * DEG2RAD;

  dvl_front_mps = msg.dvl_water_front_mps;
  dvl_right_mps = msg.dvl_water_right_mps;
  dvl_down_mps = msg.dvl_water_down_mps;

  if (msg.dvl_valid_flags == 7)  // 对底有效
  {
    dvl_front_mps = msg.dvl_bottom_front_mps;
    dvl_right_mps = msg.dvl_bottom_right_mps;
    dvl_down_mps = msg.dvl_bottom_down_mps;
  }

  /* DVL 船体语义：与旧版一致 前->x / 右->y / 下->z（供 velx=linear.x） */
  rov_odom.twist.twist.linear.x = dvl_front_mps;
  rov_odom.twist.twist.linear.y = dvl_right_mps;
  rov_odom.twist.twist.linear.z = dvl_down_mps;

  rov_odom.pose.pose.position.x = cur_imu_pos.x;
  rov_odom.pose.pose.position.y = cur_imu_pos.y;

  rov_odom.header.stamp = this->get_clock()->now();
  rov_odom.header.frame_id = "odom";
  rov_odom.child_frame_id = "base_link";
  RovOdom_publisher->publish(rov_odom);

  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = rov_odom.pose.pose.position.x;
  transformStamped.transform.translation.y = rov_odom.pose.pose.position.y;
  transformStamped.transform.translation.z = rov_odom.pose.pose.position.z;
  transformStamped.transform.rotation.x = rov_odom.pose.pose.orientation.x;
  transformStamped.transform.rotation.y = rov_odom.pose.pose.orientation.y;
  transformStamped.transform.rotation.z = rov_odom.pose.pose.orientation.z;
  transformStamped.transform.rotation.w = rov_odom.pose.pose.orientation.w;
  broadcaster_->sendTransform(transformStamped);
}

/********************************************************************************
 * @brief  :深度计回调函数
 * @param  msg:消息数据
 * @return NONE
*********************************************************************************/
void MsgAdapter::depth_callback(const sealien_ctrlpilot_msgmanagement::msg::DepthStatus& msg){
  rov_odom.pose.pose.position.z = -msg.depth_m[0];
}


/********************************************************************************
 * @brief  :重置参考点回调函数
 * @param  msg:消息数据
 * @return NONE
*********************************************************************************/
void MsgAdapter::resetRef_callback(const std_msgs::msg::Bool& msg){
  if(msg.data){
    restRef_flag = true;
  }
}

/********************************************************************************
 * @brief  :惯导航向(deg, Z下)转到控制系 yaw(rad, Z上)，并归一化到 (-pi, pi]
 * @param  heading_deg: 惯导航向角 [deg]
 * @return yaw [rad]
*********************************************************************************/
double MsgAdapter::Trans2LocatCoordinate(const double& heading_deg)
{
  double yaw_rad = -heading_deg * DEG2RAD;

  while (yaw_rad > M_PI)
  {
    yaw_rad -= 2.0 * M_PI;
  }
  while (yaw_rad < -M_PI)
  {
    yaw_rad += 2.0 * M_PI;
  }

  return yaw_rad;
}

/********************************************************************************
 * @brief  :节点函数
 * @param  argc:参数
 * @param  argv:参数
 * @return NONE
*********************************************************************************/
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MsgAdapter>());
  rclcpp::shutdown();
}