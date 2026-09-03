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

using namespace std::placeholders;
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
  // Task 编排下发的 MISSION 目标（与 planner 同结构，独立话题避免抢控）
  task_mission_subscriber_ = this->create_subscription<msg_FollowCmd>(
      "/task/mission_cmd",
      10,
      std::bind(&Controller::trackCmd_callback, this, _1));

  task_status_subscriber_ = this->create_subscription<sealien_ctrlpilot_msgmanagement::msg::TaskStatus>(
      "/task_status",
      10,
      std::bind(&Controller::TaskStatus_callback, this, _1));
  task_stage_subscriber_ = this->create_subscription<sealien_ctrlpilot_msgmanagement::msg::TaskStage>(
      "/task_stage",
      10,
      std::bind(&Controller::TaskStage_callback, this, _1));
    
  displacement_status_subscriber = this->create_subscription<sealien_ctrlpilot_msgmanagement::msg::WireDisplacementStatus>("/WireDisplacementStatus", 10,
  std::bind(&Controller::displacement_callback, this, _1)); 

  valve_status_subscriber = this->create_subscription<sealien_ctrlpilot_msgmanagement::msg::SwitchStatus>("/Switch", 10,
  std::bind(&Controller::Switchs_callback, this, _1)); 

 
  // 创建Action服务器
  oilBladder_server_= rclcpp_action::create_server<PercentTarget>(this,"/oil_bladder_target",
  std::bind(&Controller::oilBladder_handle_goal, this, _1, _2),
  std::bind(&Controller::oilBladder_handle_cancel, this, _1),
  std::bind(&Controller::oilBladder_handle_accepted, this, _1));

  pitchMotor_server_= rclcpp_action::create_server<PercentTarget>(this,"/pitch_motor_target",
  std::bind(&Controller::pitchMotor_handle_goal, this, _1, _2),
  std::bind(&Controller::pitchMotor_handle_cancel, this, _1),
  std::bind(&Controller::pitchMotor_handle_accepted, this, _1));

  // 话题对齐 communicationservice / mavlink_bridge_node
  thru_cmd_publisher = this->create_publisher<sealien_ctrlpilot_msgmanagement::msg::ThrusterCommand>("/thruster_command", 10); 
  gs_cmd_publisher = this->create_publisher<sealien_ctrlpilot_msgmanagement::msg::GsCmd>("/obc/gs_cmd", 10); 
  pitch_cmd_publisher   = this->create_publisher<sealien_ctrlpilot_msgmanagement::msg::PitchMotorCmd>("/obc/pitch_cmd", 10); 
  pump_cmd_publisher    = this->create_publisher<sealien_ctrlpilot_msgmanagement::msg::PlungerPumpCmd>("/obc/plunger_pump_cmd", 10); 
  switch_cmd_publisher  = this->create_publisher<sealien_ctrlpilot_msgmanagement::msg::SwitchCmd>("/obc/switch_cmd", 10); 
  pid_output_publisher  = this->create_publisher<sealien_ctrlpilot_msgmanagement::msg::TaskPosCmd>("~/pid_output_cmd", 10); 
  gs_output_publisher   = this->create_publisher<std_msgs::msg::Float32MultiArray>("~/gs_cmd_output", 10); 

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
  pid_debug_log_sample();
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

  status.sensor_displace_oilbladder = 0.0;  //油囊拉线传感器位移，单位%
  status.sensor_displace_pitchmotor = 0.0;  //俯仰电机拉线传感器位移，单位%
  status.valve1_status = false;   //阀1状态，0:关， 1:开
  status.valve2_status = false;   //阀2状态，0:关， 1:开

  this->declare_parameter<int>("gs1_dir", 1);
  this->declare_parameter<int>("gs2_dir", 1);
  this->declare_parameter<int>("gs3_dir", 1);
  this->declare_parameter<int>("gs4_dir", 1);
  // "cross" = 十字解耦（实艇默认）；"x" = 叉型四路混控
  this->declare_parameter<std::string>("gs_mix_layout", "cross");
  this->declare_parameter<double>("gs_mix_x_scale", 0.70710678);

  gs1_dir = this->get_parameter("gs1_dir").as_int(); 
  gs2_dir = this->get_parameter("gs2_dir").as_int();
  gs3_dir = this->get_parameter("gs3_dir").as_int();
  gs4_dir = this->get_parameter("gs4_dir").as_int();

  {
    const std::string layout = this->get_parameter("gs_mix_layout").as_string();
    if (layout == "cross")
    {
      gs_mix_layout = 0;
    }
    else
    {
      gs_mix_layout = 1;  // default / "x"
    }
  }
  gs_mix_x_scale = static_cast<float>(this->get_parameter("gs_mix_x_scale").as_double());
  if (gs_mix_x_scale < 0.1f)
  {
    gs_mix_x_scale = 0.1f;
  }
  if (gs_mix_x_scale > 1.0f)
  {
    gs_mix_x_scale = 1.0f;
  }

  this->declare_parameter<bool>("pid_debug_log_enable", true);
  this->declare_parameter<std::string>("pid_debug_log_dir", "");
  {
    const bool log_enable = this->get_parameter("pid_debug_log_enable").as_bool();
    const std::string log_dir = this->get_parameter("pid_debug_log_dir").as_string();
    pid_debug_logger_.configure(this->get_logger(), log_enable, log_dir);
  }

  RCLCPP_INFO(
      this->get_logger(),
      "gs_mix_layout=%s gs_mix_x_scale=%.3f dirs=[%d,%d,%d,%d] mission_vel=pid_vx",
      (gs_mix_layout == 0) ? "cross" : "x",
      gs_mix_x_scale,
      gs1_dir,
      gs2_dir,
      gs3_dir,
      gs4_dir);

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

  RCLCPP_INFO(this->get_logger(), "rate_yawP[%f], rate_yawI[%f], rate_yawD[%f]", pid_rate_yaw.kp,pid_rate_yaw.ki,pid_rate_yaw.kd);


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

  gs_rr_index_ = 0;
  for(int i = 0; i < 4; i++){
    last_gs_cmd_sent_[i] = 1.0e6f;  // 强制首帧下发
    gs_cycles_since_sent_[i] = GS_CMD_HEARTBEAT_CYCLES;
    last_gscmd_[i] = 0.0f;
  }
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
  output.pitch = 0.0;
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
    // twist_cmd.x = (msg.x+1)/2.0;  //规范到0-1
    twist_cmd.x = msg.x;  //规范到0-1
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
  target_cmd.pitch_angle  = msg.twist.twist.linear.z; //这里msg.twist.twist.linear.z表示的是俯仰目标角度
  target_cmd.yaw_rate     = msg.twist.twist.angular.z;
}

void Controller::TaskStatus_callback(const sealien_ctrlpilot_msgmanagement::msg::TaskStatus& msg){
  pid_debug_logger_.on_task_status(msg);
}

void Controller::TaskStage_callback(const sealien_ctrlpilot_msgmanagement::msg::TaskStage& msg){
  pid_debug_logger_.on_task_stage(msg);
}

void Controller::pid_debug_log_sample(void){
  if (!pid_debug_logger_.is_recording())
  {
    return;
  }

  pid_debug_sample_t sample;
  sample.t_sec = this->now().seconds();
  sample.task_id = pid_debug_logger_.task_id();
  sample.script_id = pid_debug_logger_.script_id();
  sample.ctrl_mode = static_cast<int>(twist_cmd.ctrl_mode);
  sample.lock_status = static_cast<int>(twist_cmd.lock_status);
  sample.depth_m = static_cast<float>(status.pos.z);
  sample.velx_sp = target_cmd.velx;
  sample.velx = static_cast<float>(status.vel.x);
  sample.velx_err = sample.velx_sp - sample.velx;
  sample.thrust_out = output.x;
  sample.pitch_sp = target_cmd.pitch_angle;
  sample.pitch = static_cast<float>(status.angle.y);
  sample.pitch_err = sample.pitch_sp - sample.pitch;
  sample.pitch_rate_sp = pid_angle_pitch.out;
  sample.pitch_rate = static_cast<float>(status.rate.y);
  sample.pitch_rate_out = pid_rate_pitch.out;
  sample.yaw_rate_sp = target_cmd.yaw_rate;
  sample.yaw_rate = static_cast<float>(status.rate.z);
  sample.yaw_rate_out = pid_rate_yaw.out;
  sample.out_x = output.x;
  sample.out_pitch = output.pitch;
  sample.out_yaw = output.yaw;
  sample.gs1 = last_gscmd_[0];
  sample.gs2 = last_gscmd_[1];
  sample.gs3 = last_gscmd_[2];
  sample.gs4 = last_gscmd_[3];
  sample.angle_pitch_kp = pid_angle_pitch.kp;
  sample.angle_pitch_ki = pid_angle_pitch.ki;
  sample.angle_pitch_kd = pid_angle_pitch.kd;
  sample.rate_pitch_kp = pid_rate_pitch.kp;
  sample.rate_pitch_ki = pid_rate_pitch.ki;
  sample.rate_pitch_kd = pid_rate_pitch.kd;
  sample.rate_yaw_kp = pid_rate_yaw.kp;
  sample.rate_yaw_ki = pid_rate_yaw.ki;
  sample.rate_yaw_kd = pid_rate_yaw.kd;
  sample.vel_x_kp = pid_vx.kp;
  sample.vel_x_ki = pid_vx.ki;
  sample.vel_x_kd = pid_vx.kd;
  pid_debug_logger_.write_sample(sample);
}

void Controller::displacement_callback(const sealien_ctrlpilot_msgmanagement::msg::WireDisplacementStatus& msg){
  status.sensor_displace_oilbladder = msg.displacement_mm[0]*100/250;
  status.sensor_displace_pitchmotor = msg.displacement_mm[1]*100/250;
  //转换成百分比
}

void Controller::Switchs_callback(const sealien_ctrlpilot_msgmanagement::msg::SwitchStatus& msg){
  status.valve1_status = msg.switch_status.at(0)? true:false;   //阀1状态，false:关， true:开
  status.valve2_status = msg.switch_status.at(1)? true:false;   //阀2状态，false:关， true:开
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
 * @brief  :舵机指令轮转下发，降低 /obc/gs_cmd 高频丢包率（原 80Hz 连发易丢包）
 * @param  gscmd: 四路目标角 deg
 * @return :NONE
 *********************************************************************************/
void Controller::publish_gs_cmd_round_robin(const float gscmd[4]){
  for(int i = 0; i < 4; i++){
    gs_cycles_since_sent_[i]++;
  }

  const int idx = gs_rr_index_;
  gs_rr_index_ = (gs_rr_index_ + 1) % 4;

  const float target = LIMIT(gscmd[idx], -MAX_GS_ANGLE, MAX_GS_ANGLE);
  const float delta = fabsf(target - last_gs_cmd_sent_[idx]);
  const bool changed = (delta >= GS_CMD_CHANGE_DEG);
  const bool heartbeat = (gs_cycles_since_sent_[idx] >= GS_CMD_HEARTBEAT_CYCLES);

  if(!changed && !heartbeat){
    return;
  }

  sealien_ctrlpilot_msgmanagement::msg::GsCmd gs_send;
  gs_send.index = static_cast<uint8_t>(idx);
  gs_send.cmd_type = 0;  // 角度控制
  gs_send.angle_deg = target;
  // 仅在真正下发时带转速；避免每帧 4 路重复刷速度字段
  gs_send.forward_speed = GS_CMD_SPEED_DPS;
  gs_send.reverse_speed = GS_CMD_SPEED_DPS;
  gs_cmd_publisher->publish(gs_send);

  last_gs_cmd_sent_[idx] = target;
  gs_cycles_since_sent_[idx] = 0;
}

/********************************************************************************
 * @brief  :动力分配
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
void Controller::Thru_Cmd_Mix(void){
  sealien_ctrlpilot_msgmanagement::msg::ThrusterCommand thru;
  sealien_ctrlpilot_msgmanagement::msg::TaskPosCmd pid_output;
  std_msgs::msg::Float32MultiArray gs_cmd_output;

  // cross: [0,1] 水平跟 pitch，[2,3] 垂直跟 yaw
  // x(叉型): 四路均混 pitch+yaw，姿态耦合更匀
  float gscmd[4];

  // AUV 只用 thrusts[0]；其余保持中位，避免把 0 当油门下发
  for(size_t i = 0; i < thru.thrusts.size(); i++){
    thru.thrusts[i] = 1500;
  }

  // TwistCmd.lock_status: 0解锁 / 1上锁；网关 thruster_unlocked: true解锁 / false上锁
  thru.thruster_unlocked = (twist_cmd.lock_status == 0);

  if(twist_cmd.lock_status){  //上锁后油门归中位。解锁才计算
    thru.thrusts[0] = 1500;    //范围1000-2000，其中1500是中位
    gscmd[0] = 0.0f;
    gscmd[1] = 0.0f;
    gscmd[2] = 0.0f;
    gscmd[3] = 0.0f;
  }else{
    float thrust_cmd = LIMIT(output.x * 500.0f + 1500.0f, 1000.0f, 2000.0f);
    thru.thrusts[0] = (uint16_t)thrust_cmd;

    if(fabs(output.x)< 0.05){
      gscmd[0] =  0.0f;
      gscmd[1] =  0.0f;
      gscmd[2] =  0.0f;
      gscmd[3] =  0.0f;

      //积分清零，防止停止时舵面还有积分量。
      pid_angle_pitch.integ = 0.0;
      pid_rate_pitch.integ  = 0.0;
      pid_rate_yaw.integ    = 0.0;

    }else if (gs_mix_layout == 0){
      // 十字：轴解耦。未用轴命令为 0（回中），不保持上次偏角。
      gscmd[0] =  gs1_dir * output.pitch * MAX_GS_ANGLE;
      gscmd[1] =  gs2_dir * output.pitch * MAX_GS_ANGLE;
      gscmd[2] =  gs3_dir * output.yaw * MAX_GS_ANGLE;
      gscmd[3] =  gs4_dir * output.yaw * MAX_GS_ANGLE;
    }else{
      // 叉型：四路舵同时参与 pitch(深) 与 yaw(向)
      // 基准混控（安装反向用 gsN_dir 翻转）：
      //   ch0 = +pitch + yaw
      //   ch1 = +pitch - yaw
      //   ch2 = -pitch + yaw
      //   ch3 = -pitch - yaw
      const float pitch_cmd = output.pitch;
      const float yaw_cmd = output.yaw;
      const float scale = gs_mix_x_scale * MAX_GS_ANGLE;
      gscmd[0] = gs1_dir * ( pitch_cmd + yaw_cmd) * scale;
      gscmd[1] = gs2_dir * ( pitch_cmd - yaw_cmd) * scale;
      gscmd[2] = gs3_dir * (-pitch_cmd + yaw_cmd) * scale;
      gscmd[3] = gs4_dir * (-pitch_cmd - yaw_cmd) * scale;
    }
  }

  for(int i = 0; i < 4; i++){
    gscmd[i] = LIMIT(gscmd[i], -MAX_GS_ANGLE, MAX_GS_ANGLE);
    last_gscmd_[i] = gscmd[i];
  }

  thru_cmd_publisher->publish(thru);
  publish_gs_cmd_round_robin(gscmd);

  pid_output.x = output.x;
  pid_output.y = output.y;
  pid_output.z = output.z;
  pid_output.roll   = output.roll;
  pid_output.pitch  = output.pitch;
  pid_output.yaw    = output.yaw;

  pid_output_publisher->publish(pid_output);

  for(int i = 0; i < 4; i++){
    gs_cmd_output.data.push_back(gscmd[i]);
  }

  gs_output_publisher->publish(gs_cmd_output);
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

