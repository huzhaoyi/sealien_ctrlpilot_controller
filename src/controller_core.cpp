/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****
* File name          : rov_vel_ctr_core.cpp
* Author             : Yi Lu
* Brief              : 
********************************************************************************
* modify
* Version   Date                Author              Described
* V1.00     2025/11/28            Yi Lu               Created
*******************************************************************************/

#include "controller_core.hpp"

namespace ControllerNS{

using std::placeholders::_1;
/********************************************************************************
 * @brief  :构造函数
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
Controller::Controller(std::string node_name):Node(node_name){
  controller_init();

  TwistCmd_subscriber = this->create_subscription<sealien_ctrlpilot_msgmanagement::msg::TwistCmd>(
    "/obc/twist_cmd", 10, std::bind(&Controller::TwistCmd_callback, this, _1));       //订阅控制指令

  RovOdom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
    "/msg_adapter/rov_odom", 10, std::bind(&Controller::RovOdom_callback, this, _1));       //订阅状态数据

  track_cmd_subscriber = this->create_subscription<msg_FollowCmd>("/pure_pursuit_node/follow_cmd", 10,
    std::bind(&Controller::trackCmd_callback, this, _1));       

  thru_cmd_publisher = this->create_publisher<sealien_ctrlpilot_msgmanagement::msg::ThrusterCmd>("~/thruster_cmd", 10); 
  gs_cmd_publisher = this->create_publisher<sealien_ctrlpilot_msgmanagement::msg::GsCmd>("~/gs_cmd", 10); 


  timer_cycle_20HZ = this->create_wall_timer(std::chrono::milliseconds((uint32_t)(1000*dt)), std::bind(&Controller::timer_20HZ_callback, this));
  timer_cycle_1HZ = this->create_wall_timer(std::chrono::milliseconds((uint32_t)(1000)), std::bind(&Controller::timer_1HZ_callback, this));

}

Controller::~Controller(){

}

void Controller::timer_20HZ_callback(){
	Run();

}

void Controller::timer_1HZ_callback(){


}

/********************************************************************************
 * @brief  :主函数
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::Run(){
  controller_mode_sw();  //每次计算都要先判断控制模式，如果需要就切换
  controller_step();  //控制更新
  control_output();    //控制输出
  
  // RCLCPP_INFO(this->get_logger(), "get_status[%d]", status.get_status);

}


/********************************************************************************
 * @brief  :参数初始化
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::controller_init(){
  dt = 0.05;
  twist_cmd.ctrl_mode   = DEFUALT_PILOT_MODE;  //初始控制模式
  twist_cmd.lock_status = DEFUALT_LOCK_STATUS; //默认上锁
  status.get_status = false; //未获得状态量

  this->declare_parameter<int>("gs1_dir", 1);
  this->declare_parameter<int>("gs2_dir", 1);
  this->declare_parameter<int>("gs3_dir", 1);
  this->declare_parameter<int>("gs4_dir", 1);

  gs1_dir = this->get_parameter("gs1_dir").as_int(); 
  gs2_dir = this->get_parameter("gs2_dir").as_int();
  gs3_dir = this->get_parameter("gs3_dir").as_int();
  gs4_dir = this->get_parameter("gs4_dir").as_int();

  status.angle.x = 0;
  status.angle.y = 0;
  status.angle.z = 0;

  status.rate.x = 0;
  status.rate.y = 0;
  status.rate.z = 0;

  status.pos.x = 0;
  status.pos.y = 0;
  status.pos.z = 0;

  status.vel.x = 0;
  status.vel.y = 0;
  status.vel.z = 0;

  this->declare_parameter<double>("angle_pitch_integration_limit", PID_ANGLE_PITCH_INTEGRATION_LIMIT);
  this->declare_parameter<double>("rate_pitch_integration_limit", PID_RATE_PITCH_INTEGRATION_LIMIT);
  this->declare_parameter<double>("rate_yaw_integration_limit", PID_RATE_YAW_INTEGRATION_LIMIT);

  this->declare_parameter<double>("angle_pitch_output_limit", PID_ANGLE_PITCH_OUTPUT_LIMIT);
  this->declare_parameter<double>("rate_pitch_output_limit", PID_RATE_PITCH_OUTPUT_LIMIT);
  this->declare_parameter<double>("rate_yaw_output_limit", PID_RATE_YAW_OUTPUT_LIMIT);


  this->declare_parameter<double>("angle_pitch_pid_P", 0.0f);
  this->declare_parameter<double>("angle_pitch_pid_I", 0.0f);
  this->declare_parameter<double>("angle_pitch_pid_D", 0.0f);
  this->declare_parameter<double>("rate_pitch_pid_P", 0.0f);
  this->declare_parameter<double>("rate_pitch_pid_I", 0.0f);
  this->declare_parameter<double>("rate_pitch_pid_D", 0.0f);  

  this->declare_parameter<double>("rate_yaw_pid_P", 1.0f);
  this->declare_parameter<double>("rate_yaw_pid_I", 0.0f);
  this->declare_parameter<double>("rate_yaw_pid_D", 0.0f);  

  float angle_pitch_integration_limit = this->get_parameter("angle_pitch_integration_limit").as_double(); 
  float rate_pitch_integration_limit = this->get_parameter("rate_pitch_integration_limit").as_double(); 
  float rate_yaw_integration_limit   = this->get_parameter("rate_yaw_integration_limit").as_double(); 

  float angle_pitch_output_limit = this->get_parameter("angle_pitch_output_limit").as_double(); 
  float rate_pitch_output_limit = this->get_parameter("rate_pitch_output_limit").as_double(); 
  float rate_yaw_output_limit   = this->get_parameter("rate_yaw_output_limit").as_double(); 

  float angle_pitch_pid_P   = this->get_parameter("angle_pitch_pid_P").as_double(); 
  float angle_pitch_pid_I   = this->get_parameter("angle_pitch_pid_I").as_double(); 
  float angle_pitch_pid_D   = this->get_parameter("angle_pitch_pid_D").as_double(); 

  float rate_pitch_pid_P  = this->get_parameter("rate_pitch_pid_P").as_double(); 
  float rate_pitch_pid_I  = this->get_parameter("rate_pitch_pid_I").as_double(); 
  float rate_pitch_pid_D  = this->get_parameter("rate_pitch_pid_D").as_double(); 

  float rate_yaw_pid_P  = this->get_parameter("rate_yaw_pid_P").as_double(); 
  float rate_yaw_pid_I = this->get_parameter("rate_yaw_pid_I").as_double(); 
  float rate_yaw_pid_D   = this->get_parameter("rate_yaw_pid_D").as_double(); 


  pid_angle_pitch.init(angle_pitch_pid_P , angle_pitch_pid_I, angle_pitch_pid_D, 
                    angle_pitch_integration_limit, angle_pitch_output_limit, dt);
  pid_rate_pitch.init(rate_pitch_pid_P , rate_pitch_pid_I, rate_pitch_pid_D, 
                  rate_pitch_integration_limit, rate_pitch_output_limit, dt);
  pid_rate_yaw.init(rate_yaw_pid_P , rate_yaw_pid_I, rate_yaw_pid_D, 
                  rate_yaw_integration_limit, rate_yaw_output_limit, dt);


  this->declare_parameter<double>("vel_x_integration_limit", PID_VELOCITY_X_INTEGRATION_LIMIT);
  float vel_x_integration_limit  = this->get_parameter("vel_x_integration_limit").as_double(); 

  this->declare_parameter<double>("vel_x_output_limit", PID_VELOCITY_X_OUTPUT_LIMIT);
  float vel_x_output_limit  = this->get_parameter("vel_x_output_limit").as_double(); 

  this->declare_parameter<double>("vel_x_pid_P", 0.0f);
  this->declare_parameter<double>("vel_x_pid_I", 0.0f);
  this->declare_parameter<double>("vel_x_pid_D", 0.0f);
  float vel_x_pid_P   = this->get_parameter("vel_x_pid_P").as_double(); 
  float vel_x_pid_I   = this->get_parameter("vel_x_pid_I").as_double(); 
  float vel_x_pid_D   = this->get_parameter("vel_x_pid_D").as_double(); 

  pid_vx.init(vel_x_pid_P, vel_x_pid_I, vel_x_pid_D, vel_x_integration_limit, vel_x_output_limit, dt);

  //实例化控制模式
  ModeMap[PILOT_MODE_NONE] = std::make_shared<PilotNone>(this);
  ModeMap[PILOT_MODE_MANUAL] = std::make_shared<PilotManual>(this);
  ModeMap[PILOT_MODE_MISSION] = std::make_shared<PilotMission>(this);
}

/********************************************************************************
 * @brief  : Controller mode switch function
            ONE模式断开Controller的数据发布，此时可以用上位机调试接口控制推进器，
            手动模式独立存在，自稳模式、定深模式可以共存
            自稳模式稳定稳定自身姿态以及航向
            定深模式稳定当前深度
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::controller_mode_sw(void){
  static uint32_t last_mode = DEFUALT_PILOT_MODE;

  if (twist_cmd.ctrl_mode != last_mode){  //当前模式不等于上一个模式，说明有模式切换
    ModeMap[twist_cmd.ctrl_mode]->reset();  //重置当前模式相关参数

    last_mode = twist_cmd.ctrl_mode;  //将上一次的模式更改为当前模式
  }else{

  }
}

/********************************************************************************
 * @brief  :运行一步PID计算，需要周期执行
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
void Controller::controller_step(void){
  if (twist_cmd.lock_status){  //遥控器上锁，输出置零，不进行控制更新
    clear_output();
    return;
  }

  ModeMap[twist_cmd.ctrl_mode]->update();

}


/********************************************************************************
 * @brief  :选择控制量，并发布
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
void Controller::control_output(void){
  if(twist_cmd.lock_status != 1){  //不上锁的时候输出控制器计算值
    ModeMap[twist_cmd.ctrl_mode]->output();
  }else{
    clear_output();
  }
  

  Thru_Cmd_Mix();  //动力分配
}

/********************************************************************************
 * @brief  :清除PID输出
 * @param  NONE
 * @return NONE
*********************************************************************************/
void Controller::clear_output(void){
  output.x = 0.0;
  output.y = 0.0;
  output.z = 0.0;
  output.roll = 0.0;
  output.roll = 0.0;
  output.yaw = 0.0;
}

/********************************************************************************
 * @brief  :遥控器指令回调函数
 * @param  msg:消息
 * @return NONE
*********************************************************************************/
void Controller::TwistCmd_callback(const sealien_ctrlpilot_msgmanagement::msg::TwistCmd& msg){
  //判断模式是否合法，不合法就强制为NONE模式，防止内存访问越界
  if(isModelegal(msg.ctrl_mode)){  
    twist_cmd.x = (msg.x+1)/2.0;  //规范到0-1
    twist_cmd.y = msg.y;
    twist_cmd.z = msg.z;
    twist_cmd.roll  = msg.roll;
    twist_cmd.pitch = msg.pitch;
    twist_cmd.yaw   = msg.yaw;
    twist_cmd.lock_status = msg.lock_status;
    twist_cmd.ctrl_mode   = msg.ctrl_mode;
  }else{
    twist_cmd.x = 0.0;
    twist_cmd.y = 0.0;
    twist_cmd.z = 0.0;
    twist_cmd.yaw = 0.0;
    twist_cmd.lock_status = 1;
    twist_cmd.ctrl_mode   = PILOT_MODE_NONE;
  }
}

void Controller::trackCmd_callback(const msg_FollowCmd& msg){
  target_cmd.velx         = msg.twist.twist.linear.x;
  target_cmd.pitch_angle  = msg.twist.twist.linear.z;
  target_cmd.yaw_rate     = msg.twist.twist.angular.z;
}

void Controller::RovOdom_callback(const nav_msgs::msg::Odometry& msg){

  double roll, pitch, yaw;  //rad
  // 使用tf2进行转换
  tf2::Quaternion tf_quat;
  tf2::fromMsg(msg.pose.pose.orientation, tf_quat);

  tf2::Matrix3x3 m(tf_quat);
  m.getRPY(roll, pitch, yaw);

  status.angle.x = roll;  
  status.angle.y = pitch;  
  status.angle.z = yaw; 


  status.rate.x = msg.twist.twist.angular.x;
  status.rate.y = msg.twist.twist.angular.y;
  status.rate.z = msg.twist.twist.angular.z;
 

  status.vel.x = msg.twist.twist.linear.x;
  status.vel.y = msg.twist.twist.linear.y;
  status.vel.z = msg.twist.twist.linear.z;

  //因为dvl+IMU的z轴位置其实不准确
  //这里的z轴位置是深度计位置，在adapter节点里会处理。
  status.pos.x = msg.pose.pose.position.x;
  status.pos.y = msg.pose.pose.position.y;
  status.pos.z = msg.pose.pose.position.z;
  
  status.get_status = true;
}


/********************************************************************************
 * @brief  :动力分配
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
void Controller::Thru_Cmd_Mix(void){
  sealien_ctrlpilot_msgmanagement::msg::ThrusterCmd thru;
  sealien_ctrlpilot_msgmanagement::msg::GsCmd gs_send;
  float gscmd[4]; //0,1是水平舵机，2、3是垂直舵机

  if(twist_cmd.lock_status){  //上锁后油门归中位。解锁才计算
    thru.thru1 = 1500;    //范围1000-2000，其中1500是中位
    gscmd[0] = 0.0;
    gscmd[1] = 0.0;
    gscmd[2] = 0.0;
    gscmd[3] = 0.0;
  }else{
    thru.thru1 = (uint16_t)(output.x*500 + 1500);

    gscmd[0] =  gs1_dir*output.pitch * MAX_GS_ANGLE;
    gscmd[1] =  gs2_dir*output.pitch * MAX_GS_ANGLE;

    gscmd[2] =  gs3_dir*output.yaw * MAX_GS_ANGLE;
    gscmd[3] =  gs4_dir*output.yaw * MAX_GS_ANGLE;

  }

  thru_cmd_publisher->publish(thru);

  for(int i=0; i<4; i++){
    gs_send.index = i;
    gs_send.angle_deg = gscmd[gs_send.index];
    gs_cmd_publisher->publish(gs_send);
  }

}

/********************************************************************************
 * @brief  :判断模式是否合理
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
bool Controller::isModelegal(int ctrlmod){ //判断模式是否合法，是返回true
  if(ctrlmod == PILOT_MODE_NONE || ctrlmod == PILOT_MODE_MANUAL || ctrlmod == PILOT_MODE_MISSION ){
    return true;
  }else{
    return false;
  }
}

} //end namespace ControllerNS

