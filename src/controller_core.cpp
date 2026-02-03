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

// #define PRINT_PARAMS     1    //0： 开始时不打印参数，1:开始时打印参数
#define PUB_THRUSTER     0    //0： 发布twist_cmd，1:发布thruster_cmd


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

  imu_subscriber = this->create_subscription<sealien_ctrlpilot_msgmanagement::msg::ImuNavStatus>(
    "/sealien_mavros/imu", 10, std::bind(&Controller::Imu_callback, this, _1));       //订阅状态数据

  depth_subscriber = this->create_subscription<sealien_ctrlpilot_msgmanagement::msg::DepthStatus>(
    "/sealien_mavros/depthFinder", 10, std::bind(&Controller::Depth_callback, this, _1));       //订阅深度数据

  height_subscriber = this->create_subscription<sealien_ctrlpilot_msgmanagement::msg::SonarAltimeterStatus>(
    "/sealien_mavros/heightStatus", 10, std::bind(&Controller::Height_callback, this, _1));       //订阅高度数据


  target_angle_publisher  = this->create_publisher<geometry_msgs::msg::Point>("~/target_angle", 10);
  target_pos_publisher    = this->create_publisher<geometry_msgs::msg::Point>("~/target_pos", 10);
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
  // RCLCPP_INFO(this->get_logger(), "yaw_base[%f]", status.yaw_base);
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
  if(status.get_status){  //只有获取状态量后才开始计算
    setpoint_mapping();  //遥控器数据处理
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
  this->declare_parameter<double>("z_gain", DEFUALT_Z_GAIN);
  this->declare_parameter<double>("yaw_limit", DEFUALT_YAW_LIMIT);
  this->declare_parameter<double>("depth_min", DEFUALT_DEPTH_MIN);
  this->declare_parameter<double>("depth_max", DEFUALT_DEPTH_MAN);
  this->declare_parameter<int>("alt_source", DEFUALT_ALT_SOURCE);
  this->declare_parameter<double>("height_min", DEFUALT_ALT_MIN);
  this->declare_parameter<double>("height_max", DEFUALT_ALT_MAX);
  this->declare_parameter<double>("ctrl_accuracy", DEFUALT_ACCURACY);

  config.dt = this->get_parameter("dt").as_double();
  config.yaw_gain = this->get_parameter("yaw_gain").as_double(); 
  config.z_gain = this->get_parameter("z_gain").as_double(); 
  config.yaw_limit = this->get_parameter("yaw_limit").as_double(); 
  config.depth_min = this->get_parameter("depth_min").as_double(); 
  config.depth_max = this->get_parameter("depth_max").as_double(); 
  config.alt_source = this->get_parameter("alt_source").as_int();
  config.height_min = this->get_parameter("height_min").as_double();
  config.height_max = this->get_parameter("height_max").as_double();
  config.ctrl_accuracy = this->get_parameter("ctrl_accuracy").as_double();

  twist_cmd.ctrl_mode   = DEFUALT_PIOLT_MODE;  //初始控制模式
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

  float angle_yaw_pid_P  = this->get_parameter("angle_yaw_pid_P").as_double(); 
  float angle_yaw_pid_I = this->get_parameter("angle_yaw_pid_I").as_double(); 
  float angle_yaw_pid_D   = this->get_parameter("angle_yaw_pid_D").as_double(); 

  float rate_yaw_pid_P  = this->get_parameter("rate_yaw_pid_P").as_double(); 
  float rate_yaw_pid_I = this->get_parameter("rate_yaw_pid_I").as_double(); 
  float rate_yaw_pid_D   = this->get_parameter("rate_yaw_pid_D").as_double(); 


  pid_angle_roll.init(0 , 0, 0, angle_roll_integration_limit, angle_roll_output_limit, config.dt);
  pid_angle_pitch.init(0 , 0, 0, angle_pitch_integration_limit, angle_pitch_output_limit, config.dt);
  pid_angle_yaw.init(angle_yaw_pid_P , angle_yaw_pid_I, angle_yaw_pid_D, angle_yaw_integration_limit, angle_yaw_output_limit, config.dt);

  pid_rate_roll.init(0 , 0, 0, rate_roll_integration_limit, rate_roll_output_limit, config.dt);
  pid_rate_pitch.init(0 , 0, 0, rate_pitch_integration_limit, rate_pitch_output_limit, config.dt);
  pid_rate_yaw.init(rate_yaw_pid_P , rate_yaw_pid_I, rate_yaw_pid_D, rate_yaw_integration_limit, rate_yaw_output_limit, config.dt);
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
  // angle_target.z  = status.angle.z;  //重置姿态控制器后，即重新进入姿态控制，目标角度设定为当前值。
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

  this->declare_parameter<double>("pos_z_pid_P", 1.2f);
  this->declare_parameter<double>("pos_z_pid_I", 0.1f);
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


  pid_x.init(0 , 0, 0, pos_x_integration_limit, pos_x_output_limit, config.dt);
  pid_y.init(0 , 0, 0, pos_y_integration_limit, pos_y_output_limit, config.dt);
  pid_z.init(pos_z_pid_P , pos_z_pid_I, pos_z_pid_D, pos_z_integration_limit, pos_z_output_limit, config.dt);

  pid_vx.init(0 , 0, 0, vel_x_integration_limit, vel_x_output_limit, config.dt);
  pid_vy.init(0 , 0, 0, vel_y_integration_limit, vel_y_output_limit, config.dt);
  pid_vz.init(vel_z_pid_P , vel_z_pid_I, vel_z_pid_D, vel_z_integration_limit, vel_z_output_limit, config.dt);
}

/********************************************************************************
 * @brief  :位置控制器重置
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::position_controller_reset(void){
  pid_x.reset();
  pid_y.reset();
  pid_z.reset();

  pid_vx.reset();
  pid_vy.reset();
  pid_vz.reset();

  //初始化目标值
  if(twist_cmd.ctrl_mode == PIOLT_MODE_STABILIZE1 || twist_cmd.ctrl_mode == PIOLT_MODE_AUTODEPTH){
    pos_target.z = status.depth;
  }else if(twist_cmd.ctrl_mode == PIOLT_MODE_STABILIZE2 || twist_cmd.ctrl_mode == PIOLT_MODE_AUTODHIGHT){
    if(config.alt_source == HEIGHT_FROM_SONAR){  //高度数据来源于测距声呐
      pos_target.z = status.sonar_height;
    }else if(config.alt_source == HEIGHT_FROM_DVL){//高度数据来源于DVL
      pos_target.z = status.dvl_alt;
    }else{//高度数据来源于IMU
      pos_target.z = status.imu_alt;
    }
  }

  // RCLCPP_INFO(this->get_logger(), "pos_target.z[%f]", pos_target.z);
}


/********************************************************************************
 * @brief  :目标值匹配，根据控制模式确定
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::setpoint_mapping(void){
  if(twist_cmd.ctrl_mode == PIOLT_MODE_NONE || twist_cmd.ctrl_mode == PIOLT_MODE_MANUAL){ //NONE和手动模式
    angle_target.x = 0;
    angle_target.y = 0;
    angle_target.z = 0;

    pos_target.x = 0;
    pos_target.x = 0;
    pos_target.x = 0;
  }else if(twist_cmd.ctrl_mode == PIOLT_MODE_STABILIZE1 || twist_cmd.ctrl_mode == PIOLT_MODE_STABILIZE2){ //两种稳定模式
    angle_target.x = 0;
    angle_target.y = 0;
    process_yaw_setpoint();

    pos_target.x = 0;
    pos_target.y = 0;
    process_z_setpoint();

  }else if(twist_cmd.ctrl_mode == PIOLT_MODE_AUTODEPTH || twist_cmd.ctrl_mode == PIOLT_MODE_AUTODHIGHT){  //定深和定高模式
    angle_target.x = 0;
    angle_target.y = 0;
    angle_target.z = 0;

    pos_target.x = 0;
    pos_target.x = 0;
    process_z_setpoint();
  }else if(twist_cmd.ctrl_mode == PIOLT_MODE_AUTODIRCETION){ //定向模式
    angle_target.x = 0;
    angle_target.y = 0;
    process_yaw_setpoint();

    pos_target.x = 0;
    pos_target.x = 0;
    pos_target.x = 0;
  }else{

  }

  target_angle_publisher->publish(angle_target);
  target_pos_publisher->publish(pos_target);

}

/********************************************************************************
 * @brief  :艏向指令处理
 * @param  yaw_in:操纵量
 * @return :NONE
 *********************************************************************************/
void Controller::process_yaw_setpoint(void){
  static float angle_add = 0.0;
  if(status.reset_target_yaw_flag){  //重置
    angle_add = 0;
    status.reset_target_yaw_flag = 0;   //清除标志量，等待下次触发
  }

  angle_add -= config.yaw_gain * twist_cmd.yaw;
  angle_add = LIMIT(angle_add, -config.yaw_limit, config.yaw_limit);   //角度目标值限幅

  if(status.yaw_base == DEFUALT_YAW_BASE){  //处理切换到自动模式时，目标角度突变问题
    status.yaw_base = status.angle.z;
    angle_target.z = angle_add + status.yaw_base;  //加上基准值
  }else{
    angle_target.z = angle_add + status.yaw_base;  //加上基准值
  }
  

  //IUM角度范围是0-360，因此目标值要在这个范围内
  if(angle_target.z >= 360){
    angle_target.z -= 360; 
  }else if(angle_target.z < 0){
    angle_target.z += 360; 
  }

  angle_target.z = LIMIT(angle_target.z, 0, 360);   //角度目标值限幅

}

/********************************************************************************
 * @brief  :垂向指令处理
 * @param  z_in:操纵量
 * @return :NONE
 *********************************************************************************/
void Controller::process_z_setpoint(void){
  // if(status.get_status !=1){  //只有获取状态量后才开始计算
  //   return;
  // }
  if(twist_cmd.ctrl_mode == PIOLT_MODE_STABILIZE1){  //定深
    pos_target.z -= config.z_gain * twist_cmd.z;
    pos_target.z = LIMIT(pos_target.z, config.depth_min, config.depth_max);
  }else{  //定高
    pos_target.z += config.z_gain * twist_cmd.z;
    pos_target.z = LIMIT(pos_target.z, config.height_min, config.height_max);
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
  static uint32_t last_mode = DEFUALT_PIOLT_MODE;

  if (twist_cmd.ctrl_mode != last_mode){
    switch (twist_cmd.ctrl_mode){
      case PIOLT_MODE_NONE:{
        
        break;
      }
      case PIOLT_MODE_MANUAL:{
        status.yaw_base = DEFUALT_YAW_BASE;  //进入手动模式，航向角不进行限制，赋值一个很大的值
        break;
      }
      case PIOLT_MODE_STABILIZE1:  //定艏和定深
      case PIOLT_MODE_STABILIZE2:{   //定艏和定高
        attitude_controller_reset();
        position_controller_reset();

        break;
      }
      case PIOLT_MODE_AUTODEPTH:{  //只控深度
        position_controller_reset();

        break;
      }
      case PIOLT_MODE_AUTODHIGHT:{ //只控高度
        position_controller_reset();

        break;
      }
      case PIOLT_MODE_AUTODIRCETION:{//只控艏向
        attitude_controller_reset();

        break;
      }
      default:
        break;
    }

    last_mode = twist_cmd.ctrl_mode;
  }else{

  }
}

/********************************************************************************
 * @brief  :运行一步PID计算，需要周期执行
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
void Controller::controller_step(void){
  controller_mode_sw();  //每次计算都要先判断控制模式，如果需要就切换

  if (twist_cmd.lock_status){
    clear_output();
    return;
  }

  switch(twist_cmd.ctrl_mode){
    case PIOLT_MODE_NONE:{
      clear_output();
    }break;
    case PIOLT_MODE_MANUAL:{
      manual_controller();  //将遥控器指令直接作为输出，其实就是透传，只是做了范围映射
    }break;
    default:{
      // 由于模式可以共存，即手动时，可以开启部分自动控制，也可以全开，所以先分配遥控器输入，再通过相关控制器修改输出
      manual_controller();
      if(twist_cmd.ctrl_mode == PIOLT_MODE_STABILIZE1 || twist_cmd.ctrl_mode == PIOLT_MODE_STABILIZE2){//稳定模式
        attitude_controller_update();
        position_controller_update();
      }else if(twist_cmd.ctrl_mode == PIOLT_MODE_AUTODEPTH || twist_cmd.ctrl_mode ==  PIOLT_MODE_AUTODHIGHT){//定高、定深 
        position_controller_update();
      }else if(twist_cmd.ctrl_mode == PIOLT_MODE_AUTODIRCETION){ //定艏
        attitude_controller_update();
      }

    }break;
  }
}


/********************************************************************************
 * @brief  :选择控制量，并发布
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
void Controller::control_output(void){
  switch(twist_cmd.ctrl_mode){
    case PIOLT_MODE_NONE:{
      clear_output();
    }break;
    case PIOLT_MODE_MANUAL:{
      output.x = manual_output.x;
      output.y = manual_output.y;
      output.z = manual_output.z;
      output.yaw = manual_output.yaw;
    }break;
    default:{
      if(twist_cmd.ctrl_mode == PIOLT_MODE_STABILIZE1 || twist_cmd.ctrl_mode == PIOLT_MODE_STABILIZE2){//稳定模式
        output.x = manual_output.x;
        output.y = manual_output.y;
        output.z = controller_output.z;
        output.yaw = controller_output.yaw;
      }else if(twist_cmd.ctrl_mode == PIOLT_MODE_AUTODEPTH || twist_cmd.ctrl_mode ==  PIOLT_MODE_AUTODHIGHT){//定高、定深 
        output.x = manual_output.x;
        output.y = manual_output.y;
        output.z = controller_output.z;
        output.yaw = manual_output.yaw;
      }else if(twist_cmd.ctrl_mode == PIOLT_MODE_AUTODIRCETION){ //定艏
        output.x = manual_output.x;
        output.y = manual_output.y;
        output.z = manual_output.z;
        output.yaw = controller_output.yaw;
      }
      
    }break;
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
void Controller::manual_controller(void){
  // manual_output.x = twist_cmd.x;
  // manual_output.y = twist_cmd.y;
  // manual_output.z = twist_cmd.z;
  // manual_output.yaw = twist_cmd.yaw;

  float vel_z=0, gyro_z=0;
  manual_output.x = twist_cmd.x;
  manual_output.y = twist_cmd.y;
  
  vel_z = 16 * LIMIT(twist_cmd.z, -1, 1); //S型函数在±8处对应的是0和1，
  if(vel_z >0){
    manual_output.z = 1.0/(1+exp(-COEF_a * (vel_z-8) + COEF_b));  //S曲线
  }else if(vel_z <0){
    manual_output.z = -1.0/(1+exp(-COEF_a * (-vel_z-8) + COEF_b));  //S曲线
  }else{
    manual_output.z =0;
  }

  gyro_z = 16*LIMIT(twist_cmd.yaw, -1, 1);
  if(gyro_z >0){
    manual_output.yaw = 1.0/(1+exp(-COEF_a * (gyro_z-8) + COEF_b));  //S曲线
  }else if(gyro_z <0){
    manual_output.yaw = -1.0/(1+exp(-COEF_a * (-gyro_z-8) + COEF_b));  //S曲线
  }else{
    manual_output.yaw =0;
  }
}

/********************************************************************************
 * @brief  :姿态控制器，这里只做艏向控制
 * @param  NONE
 * @return :NONE
 *********************************************************************************/
void Controller::attitude_controller_update(void){
  geometry_msgs::msg::Point rate_output;
  geometry_msgs::msg::Point attitude_desired;
  geometry_msgs::msg::Point rate_desired;

  // attitude_desired.x = angle_target.x;
  // attitude_desired.y = angle_target.y;
  attitude_desired.z = angle_target.z;

  attitude_angle_pid(&rate_desired, attitude_desired, status.angle);
  // controller_output.yaw = rate_desired.z;
  
  attitude_rate_pid(&rate_output, rate_desired, status.rate);
  controller_output.yaw = rate_output.z;
}

/********************************************************************************
 * @brief  :角度PID，只做了艏向
 * @param  rate_desired:PID输出，作为角速度PID的输入
 * @param  attitude_desired:PID目标值
 * @param  attitude_actual:实际状态值
 * @return :NONE
 *********************************************************************************/
void Controller::attitude_angle_pid(geometry_msgs::msg::Point* rate_desired, const geometry_msgs::msg::Point attitude_desired, 
  const geometry_msgs::msg::Point attitude_actual){

  // float roll_error = attitude_desired.x - attitude_actual.x;
  // float pitch_error = attitude_desired.y - attitude_actual.y;
  float yaw_error = attitude_desired.z - attitude_actual.z;


  if (yaw_error > 180.0f){
    yaw_error -= 360.0f;
  }else if (yaw_error < -180.0){
    yaw_error += 360.0f;
  }

  rate_desired->z = -pid_update(&pid_angle_yaw, yaw_error);

}

/********************************************************************************
 * @brief  :角度PID，只做了艏向
 * @param  rate_output:PID输出，
 * @param  rate_desired:PID目标值
 * @param  gyro_actual:实际状态值
 * @return :PID计算结果
 *********************************************************************************/
void Controller::attitude_rate_pid(geometry_msgs::msg::Point* rate_output, const geometry_msgs::msg::Point rate_desired,
  const geometry_msgs::msg::Point gyro_actual){

  // float roll_rate_error = rate_desired.x - gyro_actual.x;
  // float pitch_rate_error = rate_desired.y - gyro_actual.y;
  float yaw_rate_error = rate_desired.z - gyro_actual.z;
  // rate_output->x = pid_update(&pid_rate_roll, roll_rate_error);
  // rate_output->y = pid_update(&pid_rate_pitch, pitch_rate_error);
  rate_output->z = rate_pid_update(&pid_rate_yaw, yaw_rate_error);
  // rate_output->z = pid_update(&pid_rate_yaw, yaw_rate_error);
  // rate_output->roll = normalize_float(rate_output->x, -10, 10, -1, 1);
  // rate_output->pitch = normalize_float(rate_output->y, -10, 10, -1, 1);
  rate_output->z = normalize_float(rate_output->z, -PID_RATE_YAW_OUTPUT_LIMIT, PID_RATE_YAW_OUTPUT_LIMIT, -1, 1);
}


/********************************************************************************
 * @brief  :此函数用作更新pid
 * @param  pid:PID结构体，
 * @param  error:状态误差值
 * @return :NONE
 *********************************************************************************/
float Controller::pid_update(Pid_Object *pid, const float error){
  float output; // 输出
  pid->error = error; // 误差

  pid->integ += pid->error * pid->dt; // 积分计算
  // 积分限幅
  if (pid->integ > pid->iLimit){ // 若大于，就积分 = 积分限幅
    pid->integ = pid->iLimit;
  }else if (pid->integ < -pid->iLimit){
    pid->integ = -pid->iLimit;
  }

  pid->deriv = (pid->error - pid->prevError) / pid->dt; // 微分计算公式
  pid->outP = pid->kp * pid->error; // kp的输出值 kp * error
  pid->outI = pid->ki * pid->integ; // ki的输出值 ki * integ
  pid->outD = pid->kd * pid->deriv; // kd的输出值 kd * deriv
  output = pid->outP + pid->outI + pid->outD; // 总输出

  // 输出限幅，此处如果设置outputLimit = 0，没有输出限幅，跳过此函数。
  if(pid->outputLimit != 0){
    if (output > pid->outputLimit){
      output = pid->outputLimit;
    }else if (output < -pid->outputLimit){
      output = -pid->outputLimit;
    }    
  }

  pid->prevError = pid->error; // 更新历史误差
  pid->out = output; // 更新输出
  return output; // 返回值
}

/********************************************************************************
 * @brief  :此函数用作更新pid,用于角速度PID更新，分段PID
 * @param  pid:PID结构体，
 * @param  error:状态误差值
 * @return :PID计算结果
 *********************************************************************************/
float Controller::rate_pid_update(Pid_Object *pid, const float error){
  float output; // 输出
  pid->error = error; // 误差
  pid->integ += pid->error * pid->dt; // 积分计算
  // 积分限幅
  if (pid->integ > pid->iLimit){ // 若大于，就积分 = 积分限幅
    pid->integ = pid->iLimit;
  }else if (pid->integ < -pid->iLimit){
    pid->integ = -pid->iLimit;
  }
  pid->deriv = (pid->error - pid->prevError) / pid->dt; // 微分计算公式
  if(fabs(pid->error) > 5){
    pid->outP = 3*pid->kp * pid->error; // kp的输出值 kp * error
  }else{
    pid->outP = pid->kp * pid->error; // kp的输出值 kp * error
  }
  
  pid->outI = pid->ki * pid->integ; // ki的输出值 ki * integ
  pid->outD = pid->kd * pid->deriv; // kd的输出值 kd * deriv
  output = pid->outP + pid->outI + pid->outD; // 总输出
  // 输出限幅，此处设置outputLimit = 0，没有输出限幅，跳过此函数。
  if (pid->outputLimit != 0){
    if (output > pid->outputLimit){
      output = pid->outputLimit;
    }else if (output < -pid->outputLimit){
      output = -pid->outputLimit;
    }   
  }
  pid->prevError = pid->error; // 更新历史误差
  pid->out = output; // 更新输出
  return output; // 返回值
}

/********************************************************************************
 * @brief  :范围重映射
 * @param  value:需要映射的原始值，
 * @param  original_min:原始值范围最小值
 * @param  original_max:原始值范围最大值
 * @param  new_min:重映射范围最小值
 * @param  new_max:重映射范围最大值
 * @return :重映射后的新值
 *********************************************************************************/
float Controller::normalize_float(float value, float original_min, float original_max, float new_min, float new_max){
  if (original_min == original_max){
    if (value == original_min){
      return new_min;
    }else{
      // The original value range is 0 and cannot be normalized
      return 0;
    }
  }
  float normalized = ((value - original_min) / (original_max - original_min)) * (new_max - new_min) + new_min;
  return normalized;
}

/********************************************************************************
 * @brief  :位置控制器
 * @param  NONE
 * @return NONE
 *********************************************************************************/
void Controller::position_controller_update(void){
  geometry_msgs::msg::Point pos_desired;
  geometry_msgs::msg::Point vel_desired;
  geometry_msgs::msg::Point vel_output;

  pos_desired.x = pos_target.x;
  pos_desired.y = pos_target.y;
  pos_desired.z = pos_target.z;


  status.pos.x = 0.0;
  status.pos.y = 0.0;

  //更新状态
  if(twist_cmd.ctrl_mode == PIOLT_MODE_STABILIZE1 || twist_cmd.ctrl_mode == PIOLT_MODE_AUTODEPTH){
    status.pos.z = status.depth;
  }else if(twist_cmd.ctrl_mode == PIOLT_MODE_STABILIZE2 || twist_cmd.ctrl_mode == PIOLT_MODE_AUTODHIGHT){
    if(config.alt_source == HEIGHT_FROM_SONAR){  //高度数据来源于测距声呐
      status.pos.z = status.sonar_height;
    }else if(config.alt_source == HEIGHT_FROM_DVL){//高度数据来源于DVL
      status.pos.z = status.dvl_alt;
    }else{//高度数据来源于IMU
      status.pos.z = status.imu_alt;
    }
       
  }

  position_pos_pid(&vel_desired, pos_desired, status.pos);
  position_velocity_pid(&vel_output, vel_desired, status.vel);
  
  // control_out->vel.x = vel_output.x;
  // control_out->vel.y = vel_output.y;
  // control_out->vel.z = (vel_output.z);
  controller_output.z = config.thrust_base + vel_output.z;
}

/********************************************************************************
 * @brief  :位置控制PID
 * @param  vel_desired:PID输出
 * @param  pos_desired:PID目标值
 * @param  pos_actual:实际状态值
 * @return NONE
 *********************************************************************************/
void Controller::position_pos_pid(geometry_msgs::msg::Point* vel_desired, const geometry_msgs::msg::Point pos_desired,
  const geometry_msgs::msg::Point pos_actual){

  // float x_error = pos_desired.x - pos_actual.x;
  // float y_error = pos_desired.y - pos_actual.y;
  float z_error = -pos_desired.z + pos_actual.z;

  if(fabs(z_error) < config.ctrl_accuracy){
    z_error = 0.0;
  }
  // No position input, no x,y position control enabled
  // vel_desired->x = pid_update(&pid_x, x_error);
  // vel_desired->y = pid_update(&pid_y, z_error);
  vel_desired->z = pid_update(&pid_z, z_error);
}

/********************************************************************************
 * @brief  :速度控制PID
 * @param  vel_output:PID输出
 * @param  vel_desired:PID目标值
 * @param  vel_actual:实际状态值
 * @return NONE
 *********************************************************************************/
void Controller::position_velocity_pid(geometry_msgs::msg::Point* vel_output, const geometry_msgs::msg::Point vel_desired,
  const geometry_msgs::msg::Point vel_actual){

  // float x_vel_error = vel_desired.x - vel_actual.x;
  // float y_vel_error = vel_desired.y - vel_actual.y;
  float z_vel_error = vel_desired.z - vel_actual.z;
  // X and Y
  // vel_output.x = pid_update(&pid_vx, x_vel_error);
  // vel_output.y = pid_update(&pid_vy, y_vel_error);
  // Z
  vel_output->z = pid_update(&pid_vz, z_vel_error);
}

/********************************************************************************
 * @brief  :清除PID输出
 * @param  NONE
 * @return NONE
 *********************************************************************************/
void Controller::clear_output(void){
  output.x = 0;
  output.y = 0;
  output.z = 0;
  output.yaw = 0;
}


void Controller::TwistCmd_callback(const sealien_ctrlpilot_msgmanagement::msg::TwistCmd& msg){
  twist_cmd.x = msg.x;
  twist_cmd.y = msg.y;
  twist_cmd.z = msg.z;
  twist_cmd.yaw = msg.yaw;
  twist_cmd.lock_status = msg.lock_status;
  twist_cmd.ctrl_mode = msg.ctrl_mode;

  output.lock_status =  msg.lock_status;
  output.ctrl_mode =  msg.ctrl_mode;
}

void Controller::Imu_callback(const sealien_ctrlpilot_msgmanagement::msg::ImuNavStatus& msg){
  status.angle.x = msg.roll_deg;
  status.angle.y = msg.pitch_deg;
  status.angle.z = msg.yaw_deg;

  status.rate.x = msg.angular_velocity_dps.x;
  status.rate.y = msg.angular_velocity_dps.y;
  status.rate.z = msg.angular_velocity_dps.z;

  status.vel.x = msg.dvl_velocity_mps.x;
  status.vel.y = msg.dvl_velocity_mps.y;
  status.vel.z = msg.dvl_velocity_mps.z;

  //TODO将经纬度转换成相对距离
  // status.pos.x = msg.alt;
  // status.pos.y = msg.alt;
  status.imu_alt = msg.altitude_m;

 
  status.dvl_alt = msg.dvl_height;
  status.dvl_alt = LIMIT(status.dvl_alt, config.height_min, config.height_max);
  

  if(status.get_status == 0){
    status.get_status = 1;
  }
}

void Controller::Depth_callback(const sealien_ctrlpilot_msgmanagement::msg::DepthStatus& msg){
  status.depth = msg.depth_m[0]; //取第一个深度计数据
}

void Controller::Height_callback(const sealien_ctrlpilot_msgmanagement::msg::SonarAltimeterStatus& msg){
  status.sonar_height = msg.near_dist_cm[0]; //取第一个声呐数据
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
                   + actuator_mixer[i][3]*0 + actuator_mixer[i][4]*0+ actuator_mixer[i][5]*output.yaw;
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

