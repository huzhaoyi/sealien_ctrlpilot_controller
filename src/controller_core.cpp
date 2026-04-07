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

#define PRINT_PARAMS     0    //0： 开始时不打印参数，1:开始时打印参数
#define PUB_THRUSTER     0    //0： 发布twist_cmd，1:发布thruster_cmd

namespace ControllerNS{

//动力分配矩阵，顺序 x,y,z,roll,pitch,yaw
const float actuator_mixer[8][6] ={
  {1, 1,  0,  0,  0,  -1 },
  {1, -1, 0,  0,  0,  1 },
  {1, -1, 0,  0,  0,  -1 },
  {1, 1,  0,  0,  0,  1},
  {0, 0, -1,  1,  1,  0 },
  {0, 0,  1,  1,  -1, 0 },
  {0, 0,  1,  -1, 1,  0 },
  {0, 0,  -1, -1, -1, 0 }
};

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

  imuPos_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/msg_adapter/imuPose", 10, std::bind(&Controller::ImuPos_callback, this, _1));       //订阅状态数据

  imuData_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
    "/msg_adapter/imu_data", 10, std::bind(&Controller::ImuData_callback, this, _1));       //订阅状态数据

  dvl_subscriber = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/msg_adapter/dvl/twist", 10, std::bind(&Controller::Dvl_callback, this, _1));       //订阅状态数据

  depth_subscriber = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/msg_adapter/depth/pose", 10, std::bind(&Controller::Depth_callback, this, _1));       //订阅深度数据

  height_subscriber = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/msg_adapter/sonar/pose", 10, std::bind(&Controller::Height_callback, this, _1));       //订阅高度数据

  odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10,
    std::bind(&Controller::odom_callback, this, _1));       //订阅重置参考点指令

  track_cmd_subscriber = this->create_subscription<msg_FollowCmd>("/pure_pursuit_node/follow_cmd", 10,
    std::bind(&Controller::trackCmd_callback, this, _1));       //订阅重置参考点指令

  pathTrackStatus_subscriber =  this->create_subscription<std_msgs::msg::Bool>(
    "/pure_pursuit_node/path_track_status", 10, std::bind(&Controller::PathTrackStatus_callback, this, _1));       //订阅路径跟踪状态

  imu_twist_subscriber =  this->create_subscription<geometry_msgs::msg::Twist>(
    "/msg_adapter/imu/twist", 10, std::bind(&Controller::imu_twist_callback, this, _1));       //订阅imu速度

  taskPosCmd_subscriber =  this->create_subscription<sealien_ctrlpilot_msgmanagement::msg::TaskPosCmd>(
    "/task/pose_cmd", 10, std::bind(&Controller::taskPoseCmd_callback, this, _1));       //订阅任务

  target_angle_publisher  = this->create_publisher<geometry_msgs::msg::Point>("~/target_angle", 10);
  target_pos_publisher    = this->create_publisher<geometry_msgs::msg::Point>("~/target_pos", 10);
  task_finish_publisher   = this->create_publisher<std_msgs::msg::Bool>("/task_finish", 10);
  // test_publisher    = this->create_publisher<std_msgs::msg::Float32>("~/test_data", 10);

#if PUB_THRUSTER
  thru_cmd_publisher = this->create_publisher<sealien_ctrlpilot_msgmanagement::msg::ThrusterCmd>("~/thruster_cmd", 10); 
#else
  control_output_publisher = this->create_publisher<sealien_ctrlpilot_msgmanagement::msg::TwistCmd>("~/controller_output", 10); 
#endif



  timer_cycle_20HZ = this->create_wall_timer(std::chrono::milliseconds((uint32_t)(config.dt*1000)), std::bind(&Controller::timer_20HZ_callback, this));
  timer_cycle_10HZ = this->create_wall_timer(std::chrono::milliseconds((uint32_t)(100)), std::bind(&Controller::timer_10HZ_callback, this));
  timer_cycle_1HZ = this->create_wall_timer(std::chrono::milliseconds((uint32_t)(1000)), std::bind(&Controller::timer_1HZ_callback, this));

}

Controller::~Controller(){

}

void Controller::timer_20HZ_callback(){
	Run();

}

void Controller::timer_10HZ_callback(){
  // RCLCPP_INFO(this->get_logger(), "angle_target.z[%f]", angle_target.z);
  // RCLCPP_INFO(this->get_logger(), "status.yaw_base[%f]", status.yaw_base);
  // RCLCPP_INFO(this->get_logger(), "yaw_tar[%f], yaw_cur[%f]", angle_target.z, status.angle.z);

}

void Controller::timer_1HZ_callback(){


}

/********************************************************************************
 * @brief  :主函数
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::Run(){
  if(!init_flag.isInitFinish){
    if(init_flag.isPosCtrl){
      if(init_flag.posCount <= 0){
        init_flag.isInitFinish = true;
        attitude_controller_reset();
        position_controller_reset(status.depth);
      }

    }else{
      init_flag.isInitFinish = true;
    }
  }


  if(status.get_status && init_flag.isInitFinish){  //只有获取状态量后才开始计算
    setpoint_mapping();  //遥控器数据处理
    controller_mode_sw();  //每次计算都要先判断控制模式，如果需要就切换
    controller_step();  //控制更新
    control_output();    //控制输出
  }


  // RCLCPP_INFO(this->get_logger(), "get_status[%d]", status.get_status);

}


/********************************************************************************
 * @brief  :参数初始化
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::controller_init(){
  // 声明参数，如果不存在则使用默认值
  this->declare_parameter<double>("dt", DEFUALT_DT);
  this->declare_parameter<double>("yaw_gain", DEFUALT_YAW_GAIN);
  this->declare_parameter<double>("xy_gain", DEFUALT_XY_GAIN);
  this->declare_parameter<double>("z_gain", DEFUALT_Z_GAIN);
  this->declare_parameter<double>("yaw_limit", DEFUALT_YAW_LIMIT);
  this->declare_parameter<double>("depth_min", DEFUALT_DEPTH_MIN);
  this->declare_parameter<double>("depth_max", DEFUALT_DEPTH_MAN);
  this->declare_parameter<double>("x_min", DEFUALT_X_MIN);
  this->declare_parameter<double>("x_max", DEFUALT_X_MAN);
  this->declare_parameter<double>("y_min", DEFUALT_Y_MIN);
  this->declare_parameter<double>("y_max", DEFUALT_Y_MAN);
  this->declare_parameter<int>("alt_source", DEFUALT_ALT_SOURCE);
  this->declare_parameter<double>("height_min", DEFUALT_ALT_MIN);
  this->declare_parameter<double>("height_max", DEFUALT_ALT_MAX);
  this->declare_parameter<double>("ctrl_accuracy", DEFUALT_ACCURACY);
  this->declare_parameter<bool>("use_rollpitch_ctrl", DEFUALT_USE_ROLLPITCH);
  this->declare_parameter<double>("ref_lat", DEFUALT_REF_LAT);
  this->declare_parameter<double>("ref_lon", DEFUALT_REF_LON);
  this->declare_parameter<double>("ref_alt", DEFUALT_REF_ALT);
  this->declare_parameter<bool>("use_imu2navi", DEFUALT_USE_IMU2NAVI);
  this->declare_parameter<int>("track_alt_depth", DEFUALT_TRACK_ALT_DEPTH);
  this->declare_parameter<bool>("pos_use_brake", false);
  this->declare_parameter<double>("brake_pos_threshold", 0.05);
  this->declare_parameter<double>("brake_vel_threshold", 0.05);
  this->declare_parameter<double>("brake_kcoef", 3.0);

  config.dt = this->get_parameter("dt").as_double();
  config.yaw_gain = this->get_parameter("yaw_gain").as_double(); 
  config.xy_gain =  this->get_parameter("xy_gain").as_double();
  config.z_gain = this->get_parameter("z_gain").as_double(); 
  config.yaw_limit = this->get_parameter("yaw_limit").as_double(); 
  config.depth_min = this->get_parameter("depth_min").as_double(); 
  config.depth_max = this->get_parameter("depth_max").as_double(); 
  config.alt_source = this->get_parameter("alt_source").as_int();
  config.height_min = this->get_parameter("height_min").as_double();
  config.height_max = this->get_parameter("height_max").as_double();
  config.ctrl_accuracy = this->get_parameter("ctrl_accuracy").as_double();
  config.use_rollpitch_ctrl = this->get_parameter("use_rollpitch_ctrl").as_bool();
  config.use_imu2navi = this->get_parameter("use_imu2navi").as_bool();
  config.track_alt_depth = this->get_parameter("track_alt_depth").as_int();

  config.ref_alt = this->get_parameter("ref_lat").as_double();
  config.ref_lon = this->get_parameter("ref_lon").as_double();
  config.x_min = this->get_parameter("x_min").as_double();
  config.x_max = this->get_parameter("x_max").as_double();
  config.y_min = this->get_parameter("y_min").as_double();
  config.y_max = this->get_parameter("y_max").as_double();
  config.pos_use_brake = this->get_parameter("pos_use_brake").as_bool();
  config.brake_pos_threshold = this->get_parameter("brake_pos_threshold").as_double();
  config.brake_vel_threshold = this->get_parameter("brake_vel_threshold").as_double();
  config.brake_kcoef = this->get_parameter("brake_kcoef").as_double();


  config.ref_alt = this->get_parameter("ref_alt").as_double();
  
  origin_ref.Reset(config.ref_alt, config.ref_lon, config.ref_alt);  //重置原点

  config.thrust_base = DEFUALT_THRUST_BASE;

#if PRINT_PARAMS
  RCLCPP_INFO(this->get_logger(), "dt[%f]", config.dt);
  RCLCPP_INFO(this->get_logger(), "yaw_gain[%f]", config.yaw_gain);
  RCLCPP_INFO(this->get_logger(), "z_gain[%f]", config.z_gain);
  RCLCPP_INFO(this->get_logger(), "yaw_limit[%f]", config.yaw_limit);
  RCLCPP_INFO(this->get_logger(), "depth_min[%f]", config.depth_min);
  RCLCPP_INFO(this->get_logger(), "depth_max[%f]", config.depth_max);
  RCLCPP_INFO(this->get_logger(), "alt_source[%d]", config.alt_source);
  RCLCPP_INFO(this->get_logger(), "height_min[%f]", config.height_min);
  RCLCPP_INFO(this->get_logger(), "height_max[%f]", config.height_max);
  RCLCPP_INFO(this->get_logger(), "ctrl_accuracy[%f]", config.ctrl_accuracy);
  RCLCPP_INFO(this->get_logger(), "use_rollpitch_ctrl[%d]", config.use_rollpitch_ctrl);
  RCLCPP_INFO(this->get_logger(), "ref_lat[%lf]", config.ref_alt);
  RCLCPP_INFO(this->get_logger(), "ref_lon[%lf]", config.ref_lon);
  RCLCPP_INFO(this->get_logger(), "ref_alt[%lf]", config.ref_alt);
#endif

  have_new_track_status = false;

  twist_cmd.ctrl_mode   = DEFUALT_PILOT_MODE;  //初始控制模式
  twist_cmd.lock_status = DEFUALT_LOCK_STATUS; //默认上锁
  status.get_status = 0; //未获得状态量
  status.yaw_base   = DEFUALT_YAW_BASE; //给定一个很大的基准航向，即不限制航向角
  status.sonar_height = 0;
  status.depth = 0;
  status.dvl_alt = 0;

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

  status.reset_target_yaw_flag = 0;  //
  status.track_status = false;  //默认跟踪结束
  status.angle_add = 0.0;

  got_follow_target = false;  //是否获得跟踪目标；路径跟踪时使用
  got_task_target = false;    //是否获得任务目标；任务模式时使用

  x_target_base   = 0.0;  //x轴位置基础值，切换到位置模式时置为当前值
  x_target_delta  = 0.0;  //x轴位置遥控器增量值，累加
  y_target_base   = 0.0;   //y轴位置基础值，切换到位置模式时置为当前值
  y_target_delta  = 0.0; //y轴位置遥控器增量值，累加

  thru_cmd.thru1  = 1500;  //推进器1500表示转速0，转速范围是1000~2000
  thru_cmd.thru2  = 1500;
  thru_cmd.thru3  = 1500;
  thru_cmd.thru4  = 1500;
  thru_cmd.thru5  = 1500;
  thru_cmd.thru6  = 1500;
  thru_cmd.thru7  = 1500;
  thru_cmd.thru8  = 1500;
  thru_cmd.thru9  = 1500;
  thru_cmd.thru10 = 1500;
  thru_cmd.thru11 = 1500;
  thru_cmd.thru12 = 1500;


  attitude_controller_init();   //姿态控制器初始化
  position_controller_init();   //位置控制器初始化

  init_flag.isInitFinish = false;  //初始时置false，需要经过初始化
  init_flag.isPosCtrl  = true;  //初始默认遥控器给的是位置控制
  init_flag.posCount = 10;

  //实例化控制模式
  ModeMap[PILOT_MODE_NONE] = std::make_shared<PilotNone>(this);
  ModeMap[PILOT_MODE_MANUAL] = std::make_shared<PilotManual>(this);
  ModeMap[PILOT_MODE_STABILIZE1] = std::make_shared<PilotStablize1>(this);
  ModeMap[PILOT_MODE_STABILIZE2] = std::make_shared<PilotStablize2>(this);
  ModeMap[PILOT_MODE_AUTODEPTH] = std::make_shared<PilotAutodepth>(this);
  ModeMap[PILOT_MODE_AUTODHIGHT] = std::make_shared<PilotAutoheight>(this);
  ModeMap[PILOT_MODE_AUTODIRCETION] = std::make_shared<PilotAutoDirection>(this);
  ModeMap[PILOT_MODE_AUTOHOLD1] = std::make_shared<PilotAutoHold1>(this);
  ModeMap[PILOT_MODE_AUTOHOLD2] = std::make_shared<PilotAutoHold2>(this);
  ModeMap[PILOT_MODE_MISSION] = std::make_shared<PilotMission>(this);
}

/********************************************************************************
 * @brief  :姿态控制器初始化
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::attitude_controller_init(void){
  this->declare_parameter<double>("angle_roll_integration_limit", PID_ANGLE_ROLL_INTEGRATION_LIMIT);
  this->declare_parameter<double>("angle_pitch_integration_limit", PID_ANGLE_PITCH_INTEGRATION_LIMIT);
  this->declare_parameter<double>("angle_yaw_integration_limit", PID_ANGLE_YAW_INTEGRATION_LIMIT);

  this->declare_parameter<double>("rate_roll_integration_limit", PID_RATE_ROLL_INTEGRATION_LIMIT);
  this->declare_parameter<double>("rate_pitch_integration_limit", PID_RATE_PITCH_INTEGRATION_LIMIT);
  this->declare_parameter<double>("rate_yaw_integration_limit", PID_RATE_YAW_INTEGRATION_LIMIT);

  this->declare_parameter<double>("angle_roll_output_limit", PID_ANGLE_ROLL_OUTPUT_LIMIT);
  this->declare_parameter<double>("angle_pitch_output_limit", PID_ANGLE_PITCH_OUTPUT_LIMIT);
  this->declare_parameter<double>("angle_yaw_output_limit", PID_ANGLE_YAW_OUTPUT_LIMIT);

  this->declare_parameter<double>("rate_roll_output_limit", PID_RATE_ROLL_OUTPUT_LIMIT);
  this->declare_parameter<double>("rate_pitch_output_limit", PID_RATE_PITCH_OUTPUT_LIMIT);
  this->declare_parameter<double>("rate_yaw_output_limit", PID_RATE_YAW_OUTPUT_LIMIT);

  this->declare_parameter<double>("angle_roll_pid_P", 0.0f);
  this->declare_parameter<double>("angle_roll_pid_I", 0.0f);
  this->declare_parameter<double>("angle_roll_pid_D", 0.0f);
  this->declare_parameter<double>("rate_roll_pid_P", 0.0f);
  this->declare_parameter<double>("rate_roll_pid_I", 0.0f);
  this->declare_parameter<double>("rate_roll_pid_D", 0.0f);  

  this->declare_parameter<double>("angle_pitch_pid_P", 0.0f);
  this->declare_parameter<double>("angle_pitch_pid_I", 0.0f);
  this->declare_parameter<double>("angle_pitch_pid_D", 0.0f);
  this->declare_parameter<double>("rate_pitch_pid_P", 0.0f);
  this->declare_parameter<double>("rate_pitch_pid_I", 0.0f);
  this->declare_parameter<double>("rate_pitch_pid_D", 0.0f);  

  this->declare_parameter<double>("angle_yaw_pid_P", 0.01f);
  this->declare_parameter<double>("angle_yaw_pid_I", 0.01f);
  this->declare_parameter<double>("angle_yaw_pid_D", 0.024f);
  this->declare_parameter<double>("rate_yaw_pid_P", 1.0f);
  this->declare_parameter<double>("rate_yaw_pid_I", 0.0f);
  this->declare_parameter<double>("rate_yaw_pid_D", 0.0f);  

  float angle_roll_integration_limit  = this->get_parameter("angle_roll_integration_limit").as_double(); 
  float angle_pitch_integration_limit = this->get_parameter("angle_pitch_integration_limit").as_double(); 
  float angle_yaw_integration_limit   = this->get_parameter("angle_yaw_integration_limit").as_double(); 

  float rate_roll_integration_limit  = this->get_parameter("rate_roll_integration_limit").as_double(); 
  float rate_pitch_integration_limit = this->get_parameter("rate_pitch_integration_limit").as_double(); 
  float rate_yaw_integration_limit   = this->get_parameter("rate_yaw_integration_limit").as_double(); 

  float angle_roll_output_limit  = this->get_parameter("angle_roll_output_limit").as_double(); 
  float angle_pitch_output_limit = this->get_parameter("angle_pitch_output_limit").as_double(); 
  float angle_yaw_output_limit   = this->get_parameter("angle_yaw_output_limit").as_double(); 

  float rate_roll_output_limit  = this->get_parameter("rate_roll_output_limit").as_double(); 
  float rate_pitch_output_limit = this->get_parameter("rate_pitch_output_limit").as_double(); 
  float rate_yaw_output_limit   = this->get_parameter("rate_yaw_output_limit").as_double(); 

  float angle_roll_pid_P  = this->get_parameter("angle_roll_pid_P").as_double(); 
  float angle_roll_pid_I  = this->get_parameter("angle_roll_pid_I").as_double(); 
  float angle_roll_pid_D  = this->get_parameter("angle_roll_pid_D").as_double(); 

  float rate_roll_pid_P   = this->get_parameter("rate_roll_pid_P").as_double(); 
  float rate_roll_pid_I   = this->get_parameter("rate_roll_pid_I").as_double(); 
  float rate_roll_pid_D   = this->get_parameter("rate_roll_pid_D").as_double(); 

  float angle_pitch_pid_P   = this->get_parameter("angle_pitch_pid_P").as_double(); 
  float angle_pitch_pid_I   = this->get_parameter("angle_pitch_pid_I").as_double(); 
  float angle_pitch_pid_D   = this->get_parameter("angle_pitch_pid_D").as_double(); 

  float rate_pitch_pid_P  = this->get_parameter("rate_pitch_pid_P").as_double(); 
  float rate_pitch_pid_I  = this->get_parameter("rate_pitch_pid_I").as_double(); 
  float rate_pitch_pid_D  = this->get_parameter("rate_pitch_pid_D").as_double(); 

  float angle_yaw_pid_P  = this->get_parameter("angle_yaw_pid_P").as_double(); 
  float angle_yaw_pid_I = this->get_parameter("angle_yaw_pid_I").as_double(); 
  float angle_yaw_pid_D   = this->get_parameter("angle_yaw_pid_D").as_double(); 

  float rate_yaw_pid_P  = this->get_parameter("rate_yaw_pid_P").as_double(); 
  float rate_yaw_pid_I = this->get_parameter("rate_yaw_pid_I").as_double(); 
  float rate_yaw_pid_D   = this->get_parameter("rate_yaw_pid_D").as_double(); 

#if PRINT_PARAMS
  RCLCPP_INFO(this->get_logger(), "angle_roll_integration_limit[%f]", angle_roll_integration_limit);
  RCLCPP_INFO(this->get_logger(), "angle_pitch_integration_limit[%f]", angle_pitch_integration_limit);
  RCLCPP_INFO(this->get_logger(), "angle_yaw_integration_limit[%f]", angle_yaw_integration_limit);

  RCLCPP_INFO(this->get_logger(), "rate_roll_integration_limit[%f]", rate_roll_integration_limit);
  RCLCPP_INFO(this->get_logger(), "rate_pitch_integration_limit[%f]", rate_pitch_integration_limit);
  RCLCPP_INFO(this->get_logger(), "rate_yaw_integration_limit[%f]", rate_yaw_integration_limit);

  RCLCPP_INFO(this->get_logger(), "angle_roll_output_limit[%f]", angle_roll_output_limit);
  RCLCPP_INFO(this->get_logger(), "angle_pitch_output_limit[%f]", angle_pitch_output_limit);
  RCLCPP_INFO(this->get_logger(), "angle_yaw_output_limit[%f]", angle_yaw_output_limit);

  RCLCPP_INFO(this->get_logger(), "rate_roll_output_limit[%f]", rate_roll_output_limit);
  RCLCPP_INFO(this->get_logger(), "rate_pitch_output_limit[%f]", rate_pitch_output_limit);
  RCLCPP_INFO(this->get_logger(), "rate_yaw_output_limit[%f]", rate_yaw_output_limit);

  RCLCPP_INFO(this->get_logger(), "angle_roll_pid_P[%f]", angle_roll_pid_P);
  RCLCPP_INFO(this->get_logger(), "angle_roll_pid_I[%f]", angle_roll_pid_I);
  RCLCPP_INFO(this->get_logger(), "angle_roll_pid_D[%f]", angle_roll_pid_D);

  RCLCPP_INFO(this->get_logger(), "rate_roll_pid_P[%f]", rate_roll_pid_P);
  RCLCPP_INFO(this->get_logger(), "rate_roll_pid_I[%f]", rate_roll_pid_I);
  RCLCPP_INFO(this->get_logger(), "rate_roll_pid_D[%f]", rate_roll_pid_D);

  RCLCPP_INFO(this->get_logger(), "angle_pitch_pid_P[%f]", angle_pitch_pid_P);
  RCLCPP_INFO(this->get_logger(), "angle_pitch_pid_I[%f]", angle_pitch_pid_I);
  RCLCPP_INFO(this->get_logger(), "angle_pitch_pid_D[%f]", angle_pitch_pid_D);

  RCLCPP_INFO(this->get_logger(), "rate_pitch_pid_P[%f]", rate_pitch_pid_P);
  RCLCPP_INFO(this->get_logger(), "rate_pitch_pid_I[%f]", rate_pitch_pid_I);
  RCLCPP_INFO(this->get_logger(), "rate_pitch_pid_D[%f]", rate_pitch_pid_D);

  RCLCPP_INFO(this->get_logger(), "angle_yaw_pid_P[%f]", angle_yaw_pid_P);
  RCLCPP_INFO(this->get_logger(), "angle_yaw_pid_I[%f]", angle_yaw_pid_I);
  RCLCPP_INFO(this->get_logger(), "angle_yaw_pid_D[%f]", angle_yaw_pid_D);

  RCLCPP_INFO(this->get_logger(), "rate_yaw_pid_P[%f]", rate_yaw_pid_P);
  RCLCPP_INFO(this->get_logger(), "rate_yaw_pid_I[%f]", rate_yaw_pid_I);
  RCLCPP_INFO(this->get_logger(), "rate_yaw_pid_D[%f]", rate_yaw_pid_D);
#endif


  pid_angle_roll.init(angle_roll_pid_P , angle_roll_pid_I, angle_roll_pid_D,
                     angle_roll_integration_limit, angle_roll_output_limit, config.dt);
  pid_angle_pitch.init(angle_pitch_pid_P , angle_pitch_pid_I, angle_pitch_pid_D, 
                    angle_pitch_integration_limit, angle_pitch_output_limit, config.dt);
  pid_angle_yaw.init(angle_yaw_pid_P , angle_yaw_pid_I, angle_yaw_pid_D, 
                    angle_yaw_integration_limit, angle_yaw_output_limit, config.dt);

  pid_rate_roll.init(rate_roll_pid_P , rate_roll_pid_I, rate_roll_pid_D, 
                    rate_roll_integration_limit, rate_roll_output_limit, config.dt);
  pid_rate_pitch.init(rate_pitch_pid_P , rate_pitch_pid_I, rate_pitch_pid_D, 
                  rate_pitch_integration_limit, rate_pitch_output_limit, config.dt);
  pid_rate_yaw.init(rate_yaw_pid_P , rate_yaw_pid_I, rate_yaw_pid_D, 
                  rate_yaw_integration_limit, rate_yaw_output_limit, config.dt);
}

/********************************************************************************
 * @brief  :姿态控制器重置
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::attitude_controller_reset(void){
  pid_angle_roll.reset();
  pid_angle_pitch.reset();
  pid_angle_yaw.reset();

  pid_rate_roll.reset();
  pid_rate_pitch.reset();
  pid_rate_yaw.reset();

  status.yaw_base = status.angle.z;  //重置姿态控制器后，即重新进入姿态控制，yaw_base设定为当前值。
  status.reset_target_yaw_flag = 1;
}

/********************************************************************************
 * @brief  :位置控制器初始化
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::position_controller_init(void){
  this->declare_parameter<double>("pos_x_integration_limit", PID_POS_X_INTEGRATION_LIMIT);
  this->declare_parameter<double>("pos_y_integration_limit", PID_POS_Y_INTEGRATION_LIMIT);
  this->declare_parameter<double>("pos_z_integration_limit", PID_POS_Z_INTEGRATION_LIMIT);
  float pos_x_integration_limit  = this->get_parameter("pos_x_integration_limit").as_double(); 
  float pos_y_integration_limit = this->get_parameter("pos_y_integration_limit").as_double(); 
  float pos_z_integration_limit   = this->get_parameter("pos_z_integration_limit").as_double(); 

  this->declare_parameter<double>("vel_x_integration_limit", PID_VELOCITY_X_INTEGRATION_LIMIT);
  this->declare_parameter<double>("vel_y_integration_limit", PID_VELOCITY_Y_INTEGRATION_LIMIT);
  this->declare_parameter<double>("vel_z_integration_limit", PID_VELOCITY_Z_INTEGRATION_LIMIT);
  float vel_x_integration_limit  = this->get_parameter("vel_x_integration_limit").as_double(); 
  float vel_y_integration_limit = this->get_parameter("vel_y_integration_limit").as_double(); 
  float vel_z_integration_limit   = this->get_parameter("vel_z_integration_limit").as_double(); 

  this->declare_parameter<double>("pos_x_output_limit", PID_POS_X_OUTPUT_LIMIT);
  this->declare_parameter<double>("pos_y_output_limit", PID_POS_Y_OUTPUT_LIMIT);
  this->declare_parameter<double>("pos_z_output_limit", PID_POS_Z_OUTPUT_LIMIT);
  float pos_x_output_limit  = this->get_parameter("pos_x_output_limit").as_double(); 
  float pos_y_output_limit = this->get_parameter("pos_y_output_limit").as_double(); 
  float pos_z_output_limit   = this->get_parameter("pos_z_output_limit").as_double(); 

  this->declare_parameter<double>("vel_x_output_limit", PID_VELOCITY_X_OUTPUT_LIMIT);
  this->declare_parameter<double>("vel_y_output_limit", PID_VELOCITY_Y_OUTPUT_LIMIT);
  this->declare_parameter<double>("vel_z_output_limit", PID_VELOCITY_Z_OUTPUT_LIMIT);
  float vel_x_output_limit  = this->get_parameter("vel_x_output_limit").as_double(); 
  float vel_y_output_limit = this->get_parameter("vel_y_output_limit").as_double(); 
  float vel_z_output_limit   = this->get_parameter("vel_z_output_limit").as_double(); 

  this->declare_parameter<double>("pos_x_pid_P", 0.0f);
  this->declare_parameter<double>("pos_x_pid_I", 0.0f);
  this->declare_parameter<double>("pos_x_pid_D", 0.0f);
  float pos_x_pid_P   = this->get_parameter("pos_x_pid_P").as_double(); 
  float pos_x_pid_I   = this->get_parameter("pos_x_pid_I").as_double(); 
  float pos_x_pid_D   = this->get_parameter("pos_x_pid_D").as_double(); 

  this->declare_parameter<double>("vel_x_pid_P", 0.0f);
  this->declare_parameter<double>("vel_x_pid_I", 0.0f);
  this->declare_parameter<double>("vel_x_pid_D", 0.0f);
  float vel_x_pid_P   = this->get_parameter("vel_x_pid_P").as_double(); 
  float vel_x_pid_I   = this->get_parameter("vel_x_pid_I").as_double(); 
  float vel_x_pid_D   = this->get_parameter("vel_x_pid_D").as_double(); 

  this->declare_parameter<double>("pos_y_pid_P", 0.0f);
  this->declare_parameter<double>("pos_y_pid_I", 0.0f);
  this->declare_parameter<double>("pos_y_pid_D", 0.0f);
  float pos_y_pid_P   = this->get_parameter("pos_y_pid_P").as_double(); 
  float pos_y_pid_I   = this->get_parameter("pos_y_pid_I").as_double(); 
  float pos_y_pid_D   = this->get_parameter("pos_y_pid_D").as_double(); 

  this->declare_parameter<double>("vel_y_pid_P", 0.0f);
  this->declare_parameter<double>("vel_y_pid_I", 0.0f);
  this->declare_parameter<double>("vel_y_pid_D", 0.0f);
  float vel_y_pid_P   = this->get_parameter("vel_y_pid_P").as_double(); 
  float vel_y_pid_I   = this->get_parameter("vel_y_pid_I").as_double(); 
  float vel_y_pid_D   = this->get_parameter("vel_y_pid_D").as_double(); 

  this->declare_parameter<double>("pos_z_pid_P", 0.0f);
  this->declare_parameter<double>("pos_z_pid_I", 0.f);
  this->declare_parameter<double>("pos_z_pid_D", 0.0f);
  float pos_z_pid_P  = this->get_parameter("pos_z_pid_P").as_double(); 
  float pos_z_pid_I = this->get_parameter("pos_z_pid_I").as_double(); 
  float pos_z_pid_D   = this->get_parameter("pos_z_pid_D").as_double(); 

  this->declare_parameter<double>("vel_z_pid_P",  1.0f);
  this->declare_parameter<double>("vel_z_pid_I", 0.0f);
  this->declare_parameter<double>("vel_z_pid_D", 0.05f);
  float vel_z_pid_P  = this->get_parameter("vel_z_pid_P").as_double(); 
  float vel_z_pid_I = this->get_parameter("vel_z_pid_I").as_double(); 
  float vel_z_pid_D   = this->get_parameter("vel_z_pid_D").as_double(); 

  pid_x.init(pos_x_pid_P, pos_x_pid_I, pos_x_pid_D, pos_x_integration_limit, pos_x_output_limit, config.dt);
  pid_y.init(pos_y_pid_P, pos_y_pid_I, pos_y_pid_D, pos_y_integration_limit, pos_y_output_limit, config.dt);
  pid_z.init(pos_z_pid_P, pos_z_pid_I, pos_z_pid_D, pos_z_integration_limit, pos_z_output_limit, config.dt);

  pid_vx.init(vel_x_pid_P, vel_x_pid_I, vel_x_pid_D, vel_x_integration_limit, vel_x_output_limit, config.dt);
  pid_vy.init(vel_y_pid_P, vel_y_pid_I, vel_y_pid_D, vel_y_integration_limit, vel_y_output_limit, config.dt);
  pid_vz.init(vel_z_pid_P, vel_z_pid_I, vel_z_pid_D, vel_z_integration_limit, vel_z_output_limit, config.dt);


#if PRINT_PARAMS
  RCLCPP_INFO(this->get_logger(), "pos_x_integration_limit[%f]", pos_x_integration_limit);
  RCLCPP_INFO(this->get_logger(), "pos_y_integration_limit[%f]", pos_y_integration_limit);
  RCLCPP_INFO(this->get_logger(), "pos_z_integration_limit[%f]", pos_z_integration_limit);

  RCLCPP_INFO(this->get_logger(), "vel_x_integration_limit[%f]", vel_x_integration_limit);
  RCLCPP_INFO(this->get_logger(), "vel_y_integration_limit[%f]", vel_y_integration_limit);
  RCLCPP_INFO(this->get_logger(), "vel_z_integration_limit[%f]", vel_z_integration_limit);

  RCLCPP_INFO(this->get_logger(), "pos_x_output_limit[%f]", pos_x_output_limit);
  RCLCPP_INFO(this->get_logger(), "pos_y_output_limit[%f]", pos_y_output_limit);
  RCLCPP_INFO(this->get_logger(), "pos_z_output_limit[%f]", pos_z_output_limit);

  RCLCPP_INFO(this->get_logger(), "vel_x_output_limit[%f]", vel_x_output_limit);
  RCLCPP_INFO(this->get_logger(), "vel_y_output_limit[%f]", vel_y_output_limit);
  RCLCPP_INFO(this->get_logger(), "vel_z_output_limit[%f]", vel_z_output_limit);

  RCLCPP_INFO(this->get_logger(), "pos_x_pid_P[%f]", pos_x_pid_P);
  RCLCPP_INFO(this->get_logger(), "pos_x_pid_I[%f]", pos_x_pid_I);
  RCLCPP_INFO(this->get_logger(), "pos_x_pid_D[%f]", pos_x_pid_D);

  RCLCPP_INFO(this->get_logger(), "vel_x_pid_P[%f]", vel_x_pid_P);
  RCLCPP_INFO(this->get_logger(), "vel_x_pid_I[%f]", vel_x_pid_I);
  RCLCPP_INFO(this->get_logger(), "vel_x_pid_D[%f]", vel_x_pid_D);

  RCLCPP_INFO(this->get_logger(), "pos_y_pid_P[%f]", pos_y_pid_P);
  RCLCPP_INFO(this->get_logger(), "pos_y_pid_I[%f]", pos_y_pid_I);
  RCLCPP_INFO(this->get_logger(), "pos_y_pid_D[%f]", pos_y_pid_D);

  RCLCPP_INFO(this->get_logger(), "vel_y_pid_P[%f]", vel_y_pid_P);
  RCLCPP_INFO(this->get_logger(), "vel_y_pid_I[%f]", vel_y_pid_I);
  RCLCPP_INFO(this->get_logger(), "vel_y_pid_D[%f]", vel_y_pid_D);

  RCLCPP_INFO(this->get_logger(), "pos_z_pid_P[%f]", pos_z_pid_P);
  RCLCPP_INFO(this->get_logger(), "pos_z_pid_I[%f]", pos_z_pid_I);
  RCLCPP_INFO(this->get_logger(), "pos_z_pid_D[%f]", pos_z_pid_D);

  RCLCPP_INFO(this->get_logger(), "vel_z_pid_P[%f]", vel_z_pid_P);
  RCLCPP_INFO(this->get_logger(), "vel_z_pid_I[%f]", vel_z_pid_I);
  RCLCPP_INFO(this->get_logger(), "vel_z_pid_D[%f]", vel_z_pid_D);
#endif


}

/********************************************************************************
 * @brief  :位置控制器重置
 * @param  z_status:z轴状态
 * @return :NONE
 *********************************************************************************/
void Controller::position_controller_reset(float z_status){
  pid_x.reset();
  pid_y.reset();
  pid_z.reset();

  pid_vx.reset();
  pid_vy.reset();
  pid_vz.reset();


  //初始化XY轴目标值
  x_target_base = status.pos.x;
  y_target_base = status.pos.y;

  x_target_delta = 0.0;
  y_target_delta = 0.0;

  pos_target.z = z_status;

  // RCLCPP_INFO(this->get_logger(), "pos_target.z[%f]", pos_target.z);
}


/********************************************************************************
 * @brief  :目标值匹配，根据控制模式确定
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::setpoint_mapping(void){
  ModeMap[twist_cmd.ctrl_mode]->setpoint_mapping();

  //for debug
  target_angle_publisher->publish(angle_target);
  target_pos_publisher->publish(pos_target);

 
  if(twist_cmd.ctrl_mode != PILOT_MODE_MISSION){  //不是任务模式，置false,路径跟踪用
    got_follow_target = false;
    got_task_target   = false;
  }
}

/********************************************************************************
 * @brief  :艏向指令处理
 * @param  yaw_in:操纵量
 * @return :NONE
 *********************************************************************************/
void Controller::process_yaw_setpoint(void){
  if(status.reset_target_yaw_flag){  //重置
    status.angle_add = 0;
    status.reset_target_yaw_flag = 0;   //清除标志量，等待下次触发
  }

  status.angle_add += config.yaw_gain * twist_cmd.yaw;
  status.angle_add = LIMIT(status.angle_add, -config.yaw_limit, config.yaw_limit);   //角度目标值限幅

  if(status.yaw_base == DEFUALT_YAW_BASE){  //处理切换到自动模式时，目标角度突变问题
    status.yaw_base = status.angle.z;
    angle_target.z = status.angle_add + status.yaw_base;  //加上基准值
  }else{
    angle_target.z = status.angle_add + status.yaw_base;  //加上基准值
  }
  
  //规范到±180度范围
  angle_target.z = NORMALIZE_YAW(angle_target.z);

  angle_target.z = LIMIT(angle_target.z, -180, 180);   //角度目标值限幅

}

/********************************************************************************
 * @brief  :垂向指令处理
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
void Controller::process_z_setpoint(void){
  if(twist_cmd.ctrl_mode == PILOT_MODE_STABILIZE1 ||
     twist_cmd.ctrl_mode == PILOT_MODE_AUTOHOLD1 ||
     twist_cmd.ctrl_mode == PILOT_MODE_AUTODEPTH){  //Z轴是定深
    pos_target.z -= config.z_gain * twist_cmd.z;
    pos_target.z = LIMIT(pos_target.z, config.depth_min, config.depth_max);
  }else if(twist_cmd.ctrl_mode == PILOT_MODE_STABILIZE2 ||
           twist_cmd.ctrl_mode == PILOT_MODE_AUTOHOLD2 ||
           twist_cmd.ctrl_mode == PILOT_MODE_AUTODHIGHT){  //Z轴是定高
    pos_target.z += config.z_gain * twist_cmd.z;
    pos_target.z = LIMIT(pos_target.z, config.height_min, config.height_max);
  }
}

/********************************************************************************
 * @brief  :横向和纵向指令处理
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
void Controller::process_xy_setpoint(void){
  geometry_msgs::msg::Point p_in, p_out;
  p_in.x = config.xy_gain * twist_cmd.x;
  p_in.y = config.xy_gain * twist_cmd.y;

  pointBaseLinkToOdom(p_in, p_out);  

  x_target_delta -=  p_out.x;
  y_target_delta -=  p_out.y;

  pos_target.x = x_target_base + x_target_delta;
  pos_target.y = y_target_base + y_target_delta;


  pos_target.x = LIMIT(pos_target.x, config.x_min, config.x_max);
  pos_target.y = LIMIT(pos_target.y, config.y_min, config.y_max);
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
  //如果不是手动模式，也要计算一次手动模式，手动模式要每次都计算。
  if(twist_cmd.ctrl_mode != PILOT_MODE_MANUAL){
    ModeMap[PILOT_MODE_MANUAL]->update();
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
  
#if PUB_THRUSTER
  Thru_Cmd_Mix();  //动力分配
  thru_cmd_publisher->publish(thru_cmd);  //直接发布经过动力分配后，推进器的指令
#else
  control_output_publisher->publish(output); 
#endif

}

/********************************************************************************
 * @brief  :遥控器数据透传，这里只对垂向和航向做了S曲线处理，如果需要可以增加其他方向
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
void Controller::manual_controller_update(geometry_msgs::msg::Point& rate_output, geometry_msgs::msg::Point& vel_output){
  float vel_z = 0.0, gyro_z = 0.0;
  vel_output.x = twist_cmd.x;
  vel_output.y = twist_cmd.y;
  
  vel_z = 16 * LIMIT(twist_cmd.z, -1, 1); //S型函数在±8处对应的是0和1，
  if(vel_z >0){
    vel_output.z = 1.0/(1+exp(-COEF_a * (vel_z-8) + COEF_b));  //S曲线
  }else if(vel_z <0){
    vel_output.z = -1.0/(1+exp(-COEF_a * (-vel_z-8) + COEF_b));  //S曲线
  }else{
    vel_output.z = 0.0;
  }

  rate_output.x = 0.0;
  rate_output.y = 0.0;

  gyro_z = 16*LIMIT(twist_cmd.yaw, -1, 1);
  if(gyro_z > 0){
    rate_output.z = 1.0/(1+exp(-COEF_a * (gyro_z-8) + COEF_b));  //S曲线
  }else if(gyro_z <0){
    rate_output.z = -1.0/(1+exp(-COEF_a * (-gyro_z-8) + COEF_b));  //S曲线
  }else{
    rate_output.z = 0.0;
  }
}

/********************************************************************************
 * @brief  :姿态控制器
 * @param  rate_output:计算后的输出
 * @return :NONE
 *********************************************************************************/
void Controller::attitude_controller_update(geometry_msgs::msg::Point& rate_output){
  geometry_msgs::msg::Point attitude_desired;
  geometry_msgs::msg::Point rate_desired;

  attitude_desired.x = angle_target.x;
  attitude_desired.y = angle_target.y;  
  attitude_desired.z = angle_target.z;

  attitude_angle_pid(rate_desired, attitude_desired, status.angle);
  
  attitude_rate_pid(rate_output, rate_desired, status.rate);
}

/********************************************************************************
 * @brief  :角度PID，只做了艏向
 * @param  rate_desired:PID输出，作为角速度PID的输入
 * @param  attitude_desired:PID目标值
 * @param  attitude_actual:实际状态值
 * @return :NONE
 *********************************************************************************/
void Controller::attitude_angle_pid(geometry_msgs::msg::Point& rate_desired, const geometry_msgs::msg::Point attitude_desired, 
  const geometry_msgs::msg::Point attitude_actual){

  float roll_error  = attitude_desired.x - attitude_actual.x;
  float pitch_error = attitude_desired.y - attitude_actual.y;
  float yaw_error   = attitude_desired.z - attitude_actual.z;

  if(config.use_rollpitch_ctrl){ //使用俯仰滚转角控制
    rate_desired.x = -pid_angle_roll.pid_update(roll_error);
    rate_desired.y = -pid_angle_pitch.pid_update(pitch_error);
  }else{//不使用俯仰滚转角控制
    rate_desired.x = 0.0;
    rate_desired.y = 0.0;
  }

  //规范到±180度范围
  yaw_error = NORMALIZE_YAW(yaw_error);
  rate_desired.z = pid_angle_yaw.pid_update(yaw_error);

  // RCLCPP_INFO(this->get_logger(), "pitch_error[%f],rate_desired.y[%f]",pitch_error, rate_desired.y);
  // RCLCPP_INFO(this->get_logger(), "attitude_desired.z[%f], attitude_actual.z[%f]",attitude_desired.z, attitude_actual.z);
  // RCLCPP_INFO(this->get_logger(), "yaw_error[%f], rate_desired.z[%f]",yaw_error, rate_desired.z);
}

/********************************************************************************
 * @brief  :角度PID，只做了艏向
 * @param  rate_output:PID输出，
 * @param  rate_desired:PID目标值
 * @param  gyro_actual:实际状态值
 * @return :PID计算结果
 *********************************************************************************/
void Controller::attitude_rate_pid(geometry_msgs::msg::Point& rate_output, const geometry_msgs::msg::Point rate_desired,
  const geometry_msgs::msg::Point gyro_actual){

  float roll_rate_error   = rate_desired.x - gyro_actual.x;
  float pitch_rate_error  = rate_desired.y - gyro_actual.y;
  float yaw_rate_error    = rate_desired.z - gyro_actual.z;

  if(config.use_rollpitch_ctrl){
    rate_output.x = pid_rate_roll.pid_update(roll_rate_error);
    rate_output.y = pid_rate_pitch.pid_update(pitch_rate_error);
  }else{
    rate_output.x = 0.0;
    rate_output.y = 0.0;
  }

  rate_output.z = pid_rate_yaw.pid_update(yaw_rate_error);
}

/********************************************************************************
 * @brief  :位置控制器
 * @param  vel_output:速度期望值
 * @param  z_status:z轴位置，不同模式（定高和定深，或者不同传感器），来源不同
 * @return NONE
 *********************************************************************************/
void Controller::position_controller_update(geometry_msgs::msg::Point& vel_output, float z_status){
  geometry_msgs::msg::Point pos_desired;
  geometry_msgs::msg::Point vel_desired;

  pos_desired.x = pos_target.x;
  pos_desired.y = pos_target.y;
  pos_desired.z = pos_target.z;

  status.pos.z = z_status;


  position_pos_pid(vel_desired, pos_desired, status.pos);

  position_velocity_pid(vel_output, vel_desired, status.vel);
  
  vel_output.z = config.thrust_base + vel_output.z;
}

/********************************************************************************
 * @brief  :位置控制PID
 * @param  vel_desired:PID输出
 * @param  pos_desired:PID目标值
 * @param  pos_actual:实际状态值
 * @return NONE
 *********************************************************************************/
void Controller::position_pos_pid(geometry_msgs::msg::Point& vel_desired, const geometry_msgs::msg::Point pos_desired,
  const geometry_msgs::msg::Point pos_actual){
  
  geometry_msgs::msg::Point pint,pout;
  pint.x = pos_desired.x - pos_actual.x;
  pint.y = pos_desired.y - pos_actual.y;
  pointOdomToBaseLink(pint, pout);

  //限制位置误差值
  pout.x = LIMIT(pout.x, -2, 2);
  pout.y = LIMIT(pout.y, -2, 2);
  pout.z = LIMIT(pout.z, -2, 2);

  float x_error = pout.x;
  float y_error = pout.y;
  float z_error = pos_desired.z - pos_actual.z;

  //死区处理，也是控制精度处理
  x_error = DEADZONE(x_error, -0.5*config.ctrl_accuracy, 0.5*config.ctrl_accuracy);
  y_error = DEADZONE(y_error, -0.5*config.ctrl_accuracy, 0.5*config.ctrl_accuracy);
  z_error = DEADZONE(z_error, -config.ctrl_accuracy, config.ctrl_accuracy);

  vel_desired.x = pid_x.pid_update(x_error);
  vel_desired.y = pid_y.pid_update(y_error);
  vel_desired.z = pid_z.pid_update(z_error);
  // RCLCPP_INFO(this->get_logger(), "vel_desired.x[%f]",vel_desired.x);
  // RCLCPP_INFO(this->get_logger(), "x_error[%f], ",x_error);
}

/********************************************************************************
 * @brief  :速度控制PID
 * @param  vel_output:PID输出
 * @param  vel_desired:PID目标值
 * @param  vel_actual:实际状态值
 * @return NONE
 *********************************************************************************/
void Controller::position_velocity_pid(geometry_msgs::msg::Point& vel_output, const geometry_msgs::msg::Point vel_desired,
  const geometry_msgs::msg::Point vel_actual){

  float x_vel_error = vel_desired.x - vel_actual.x;
  float y_vel_error = vel_desired.y - vel_actual.y;
  float z_vel_error = vel_desired.z - vel_actual.z;

  // X and Y
  vel_output.x = -pid_vx.pid_update(x_vel_error);
  vel_output.y = -pid_vy.pid_update(y_vel_error);
  // Z
  vel_output.z = -pid_vz.pid_update(z_vel_error);

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
  if(!init_flag.isInitFinish){
    init_flag.isPosCtrl = isPosCtrlMode(msg.ctrl_mode);
    return;
  }

  twist_cmd.x = msg.x;
  twist_cmd.y = msg.y;
  twist_cmd.z = msg.z;
  twist_cmd.yaw = msg.yaw;
  twist_cmd.lock_status = msg.lock_status;
  twist_cmd.ctrl_mode   = msg.ctrl_mode;

  output.lock_status  =  msg.lock_status;
  output.ctrl_mode    =  msg.ctrl_mode;
}

void Controller::ImuPos_callback(const geometry_msgs::msg::PoseStamped& msg){
  double roll, pitch, yaw;
  // 使用tf2进行转换
  tf2::Quaternion tf_quat;
  tf2::fromMsg(msg.pose.orientation, tf_quat);

  tf2::Matrix3x3 m(tf_quat);
  m.getRPY(roll, pitch, yaw);
 
  status.angle.x = roll * RAD2DEG;
  status.angle.y = pitch * RAD2DEG;
  status.angle.z = NORMALIZE_YAW(yaw * RAD2DEG);

  // RCLCPP_INFO(this->get_logger(),"yaw[%f]",status.angle.z);

  if(config.use_imu2navi){  //使用IMU的位置作为位置参考
    // status.angle.z = NORMALIZE_YAW(yaw * RAD2DEG);
    status.pos.x = msg.pose.position.x;
    status.pos.y = msg.pose.position.y;
    status.imu_alt = msg.pose.position.z;
  }

  if(status.get_status == 0){
    status.get_status = 1;
  }
}

void Controller::ImuData_callback(const sensor_msgs::msg::Imu& msg){
  status.rate.x = msg.angular_velocity.x;
  status.rate.y = msg.angular_velocity.y;
  status.rate.z = msg.angular_velocity.z;
}

void Controller::Dvl_callback(const geometry_msgs::msg::TwistWithCovarianceStamped& msg){
  if(twist_cmd.ctrl_mode == PILOT_MODE_AUTOHOLD1 ||  //位置保持模式速度使用dvl速度，非位置保持模式使用imu速度
     twist_cmd.ctrl_mode == PILOT_MODE_AUTOHOLD2 ||
     twist_cmd.ctrl_mode == PILOT_MODE_MISSION ){

    // status.vel.x = msg.twist.twist.linear.x;
    // status.vel.y = msg.twist.twist.linear.y;
    // status.vel.z = msg.twist.twist.linear.z;
  }
}

void Controller::Depth_callback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg){
  status.depth = msg.pose.pose.position.z; //取第一个深度计数据
}

void Controller::Height_callback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg){
  status.sonar_height = msg.pose.pose.position.z; //单位m;
}

void Controller::PathTrackStatus_callback(const std_msgs::msg::Bool& msg){
  static bool last_status = status.track_status;   //初始认为跟踪已经结束

  if(status.track_status != msg.data){
    have_new_track_status = true;
    RCLCPP_INFO(this->get_logger(),"have_new_track_status true*********");
  }
 
  status.track_status = msg.data; 

  //清除计数，否则计数超时认为跟踪节点断连。
  //会进入位置保持模式，保持在当前位置。需要切换其他模式才可以操纵.
  //或者重新启动路径跟踪节点
  std::dynamic_pointer_cast<PilotMission>(ModeMap[PILOT_MODE_MISSION])->reset_count();
}

void Controller::imu_twist_callback(const geometry_msgs::msg::Twist& msg){
  status.vel.z = msg.linear.z;

  if(twist_cmd.ctrl_mode == PILOT_MODE_AUTOHOLD1 ||  //位置保持模式速度使用dvl速度，非位置保持模式使用imu速度
     twist_cmd.ctrl_mode == PILOT_MODE_AUTOHOLD2 ||
     twist_cmd.ctrl_mode == PILOT_MODE_MISSION ){

    return;
  }

  status.vel.x = msg.linear.x;
  status.vel.y = msg.linear.y;
}

void Controller::trackCmd_callback(const msg_FollowCmd& msg){
  vel_target.x = msg.twist.twist.linear.x;
  vel_target.y = 0.0; //y轴不控
  vel_target.z = msg.twist.twist.linear.z;

  rate_target.x = 0.0;  //roll在本地控
  rate_target.y = 0.0;  //pitch在本地控
  rate_target.z = msg.twist.twist.angular.z;

  if(status.track_status){
    follow_target_pos = msg.target; //跟踪的最终目标
    follow_target_ang = msg.angle_deg;
    follow_direct = msg.dir;  //跟踪的方向，0：前进，1：后退
    got_follow_target = true;
    got_task_target   = true;
  }
  
}

void Controller::taskPoseCmd_callback(const sealien_ctrlpilot_msgmanagement::msg::TaskPosCmd& msg){
  if(twist_cmd.ctrl_mode != PILOT_MODE_MISSION){  //不是在任务模式下，忽略任务的位置指令
    return;
  }

  pos_target.x = msg.x;
  pos_target.y = msg.y;
  pos_target.z = msg.z;

  angle_target.x = msg.roll;
  angle_target.y = msg.pitch;
  angle_target.z = msg.yaw;

  got_task_target = true;   //标志获得任务

  RCLCPP_INFO(this->get_logger(), "get task");

}


//定位模块发出的位置信息
void Controller::odom_callback(const nav_msgs::msg::Odometry& msg){
  if(config.use_imu2navi){ //使用IMU的位置数据，就直接返回。
    return;
  }

  if(init_flag.posCount>0){
    init_flag.posCount--;
  }


  double roll, pitch, yaw;
  // 使用tf2进行转换
  tf2::Quaternion tf_quat;
  tf2::fromMsg(msg.pose.pose.orientation, tf_quat);

  tf2::Matrix3x3 m(tf_quat);
  m.getRPY(roll, pitch, yaw);
 
  // status.angle.z = yaw*RAD2DEG;  //角度转换成°

  // geometry_msgs::msg::Point pout;
  // pointTransform(msg.pose.pose.position,  pout);

  // status.pos.x = pout.x;
  // status.pos.y = pout.y;

  status.pos.x = msg.pose.pose.position.x;
  status.pos.y = msg.pose.pose.position.y;

  status.vel.x = msg.twist.twist.linear.x;
  status.vel.y = msg.twist.twist.linear.y;
  // status.vel.z = msg.twist.twist.linear.z;
}

/********************************************************************************
 * @brief  :动力分配
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
void Controller::Thru_Cmd_Mix(void){
  float thruArray[8] = {0};

  if(output.lock_status == 0){  //上锁后油门归中位。解锁才计算
    for(int i=0; i<8; i++){
      thruArray[i] = actuator_mixer[i][0]*output.x + actuator_mixer[i][1]*output.y + actuator_mixer[i][2]*output.z
                   + actuator_mixer[i][3]*output.roll + actuator_mixer[i][4]*output.pitch+ actuator_mixer[i][5]*output.yaw;
      thruArray[i] = LIMIT(thruArray[i], -1, 1);  //限幅
    }
  }

  thru_cmd.thru1 = 500*thruArray[0] + 1500;  //1500是中位
  thru_cmd.thru2 = 500*thruArray[1] + 1500;  //1500是中位
  thru_cmd.thru3 = 500*thruArray[2] + 1500;  //1500是中位
  thru_cmd.thru4 = 500*thruArray[3] + 1500;  //1500是中位
  thru_cmd.thru5 = 500*thruArray[4] + 1500;  //1500是中位
  thru_cmd.thru6 = 500*thruArray[5] + 1500;  //1500是中位
  thru_cmd.thru7 = 500*thruArray[6] + 1500;  //1500是中位
  thru_cmd.thru8 = 500*thruArray[7] + 1500;  //1500是中位

  thru_cmd.thru9  = 1500;
  thru_cmd.thru10 = 1500;    
  thru_cmd.thru11 = 1500;  
  thru_cmd.thru12 = 1500;  
}

/********************************************************************************
 * @brief  :将位置误差从odom转换到base_link
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
void Controller::pointOdomToBaseLink(geometry_msgs::msg::Point p_in,  geometry_msgs::msg::Point& p_out){
  float yaw = status.angle.z/RAD2DEG;
  p_out.x = p_in.x * cos(yaw) + p_in.y * sin(yaw);
  p_out.y = -p_in.x * sin(yaw) + p_in.y * cos(yaw);
}

/********************************************************************************
 * @brief  :将位置误差从base_link转换到odom
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
void Controller::pointBaseLinkToOdom(geometry_msgs::msg::Point p_in,  geometry_msgs::msg::Point& p_out){
  float yaw = status.angle.z/RAD2DEG;
  p_out.x = p_in.x * cos(yaw) - p_in.y * sin(yaw);
  p_out.y = p_in.x * sin(yaw) + p_in.y * cos(yaw);
}

/********************************************************************************
 * @brief  :刹车
 * @param  pos_error：位置误差
 * @param  cur_vel：当前速度
 * @return :刹车控制量
 *********************************************************************************/
double Controller::brake(double pos_error, double cur_vel){
  double acc_cmd;
  if(config.pos_use_brake == false){
    return 0.0;
  }
  if(fabs(pos_error) < config.brake_pos_threshold && fabs(cur_vel)>config.brake_vel_threshold){
    acc_cmd = config.brake_kcoef *cur_vel;
  }else{
    acc_cmd = 0;
  }
  acc_cmd = acc_cmd > 1.0? 1.0:(acc_cmd<-1.0? -1.0:acc_cmd);   //限幅
  return acc_cmd;
}

//判断任务是否执行完成，在任务模式时使用
bool Controller::isTaskFinish(void){
  double cur_dist = sqrt(pow(status.pos.x-pos_target.x, 2) + 
                    pow(status.pos.y-pos_target.y, 2) +
                    pow(status.pos.y-pos_target.y, 3));

  bool vel_zero = (status.vel.x < 0.05) && (status.vel.y < 0.05) && (status.vel.z < 0.05);
  bool rate_zero = (status.rate.x < 0.05) && (status.rate.y < 0.05) && (status.rate.z < 0.05);
  bool delta_angle_zero = (fabs(angle_target.x - status.angle.x)<0.5) &&
                          (fabs(angle_target.y - status.angle.y)<0.5) &&
                          (fabs(angle_target.z - status.angle.z)<0.5) ;

  if(cur_dist<0.2 && vel_zero && rate_zero && delta_angle_zero){
    return true;
  }

  return false;
}

void Controller::TaskFinishPub(void){
  std_msgs::msg::Bool Tfinish_status;
  Tfinish_status.data = true;
  task_finish_publisher->publish(Tfinish_status);
}

bool Controller::isPosCtrlMode(int ctrlmod){ //判断是否是未知控制
  if(ctrlmod == PILOT_MODE_AUTOHOLD1 || ctrlmod == PILOT_MODE_AUTOHOLD2 || ctrlmod == PILOT_MODE_MISSION){
    return true;
  }else{
    return false;
  }
}

} //end namespace ControllerNS

