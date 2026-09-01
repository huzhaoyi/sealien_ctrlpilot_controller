/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****
* File name          : msgs_adapter_node.cpp
* Author             : Yi Lu
* Brief              : 
********************************************************************************
* modify
* Version   Date                Author              Described
* V1.00     2026/8/25            Yi Lu               Created
*******************************************************************************/

#include "msgs_adapter_node.hpp" 

#define RAD2DEG  (180.0/M_PI)

using std::placeholders::_1;

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
void MsgAdapter::imu_callback(const sealien_ctrlpilot_msgmanagement::msg::Elb105Shzr04& msg){
  geometry_msgs::msg::PoseStamped imu_pose;
  tf2::Quaternion quat;
  std_msgs::msg::Float32 yaw1_pub;
  double locate_yaw;

  if(msg.alignment_status != 3){
    return;
  }

  locate_yaw = Trans2LocatCoordinate(msg.heading_deg);//转换坐标系，并转换成rad

  yaw1_pub.data = msg.heading_deg;  
  yaw_origin_publisher->publish(yaw1_pub);

  quat.setRPY(msg.roll_deg/RAD2DEG, msg.pitch_deg/RAD2DEG, locate_yaw);


  //判断是否要重置参考点
  if(restRef_flag){
    restRef_flag = false;
    origin_ref.Reset(msg.latitude_deg, msg.longitude_deg, rov_odom.pose.pose.position.z);
  }


  origin_ref.Forward(msg.latitude_deg, msg.longitude_deg, rov_odom.pose.pose.position.z,  //相对位置
      cur_imu_pos.x, cur_imu_pos.y, cur_imu_pos.z);

  rov_odom.pose.pose.orientation = tf2::toMsg(quat);
  
  rov_odom.twist.twist.angular.x = msg.gyro_x_degps/RAD2DEG;
  rov_odom.twist.twist.angular.y = msg.gyro_y_degps/RAD2DEG;
  rov_odom.twist.twist.angular.z = msg.gyro_z_degps/RAD2DEG;

  rov_odom.twist.twist.linear.x = msg.dvl_water_front_mps;
  rov_odom.twist.twist.linear.y = msg.dvl_water_right_mps;
  rov_odom.twist.twist.linear.z = msg.dvl_water_down_mps;

  if(msg.dvl_valid_flags == 7){ //对底有效
    rov_odom.twist.twist.linear.x = msg.dvl_bottom_front_mps;
    rov_odom.twist.twist.linear.y = msg.dvl_bottom_right_mps;
    rov_odom.twist.twist.linear.z = msg.dvl_bottom_down_mps;
  }

  rov_odom.pose.pose.position.x = cur_imu_pos.x;
  rov_odom.pose.pose.position.y = cur_imu_pos.y;


  rov_odom.header.stamp = this->get_clock()->now();
  rov_odom.header.frame_id = "odom";
  rov_odom.child_frame_id = "base_link";
  RovOdom_publisher->publish(rov_odom);


  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = "odom"; // Source frame
  transformStamped.child_frame_id = "base_link"; // Target frame
  transformStamped.transform.translation.x = rov_odom.pose.pose.position.x; // Example translation from odom to base_link frame (x, y, z)
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
 * @brief  :转换到控制坐标系
 * @param  angle:消息数据
 * @return NONE
*********************************************************************************/
double MsgAdapter::Trans2LocatCoordinate(const double& angle){
  float yaw_tmp = 2*M_PI - angle;   //imu角度相反
  // float yaw_tmp = msg.yaw_deg;  

  if(yaw_tmp > M_PI){
    yaw_tmp = yaw_tmp - 2*M_PI;
  }
  
  return yaw_tmp;
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