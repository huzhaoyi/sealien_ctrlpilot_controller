#pragma once
#include "rclcpp/rclcpp.hpp"
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>
#include <thread>

#include "pid.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/twist_cmd.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/imu_nav_status.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/depth_status.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/sonar_altimeter_status.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/thruster_cmd.hpp"
#include "std_msgs/msg/float32.hpp"


#define PID_ANGLE_ROLL_INTEGRATION_LIMIT  (5.0f)
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT (5.0f)
#define PID_ANGLE_YAW_INTEGRATION_LIMIT   (1.0f)

#define PID_RATE_ROLL_INTEGRATION_LIMIT   (5.0f)
#define PID_RATE_PITCH_INTEGRATION_LIMIT  (5.0f)
#define PID_RATE_YAW_INTEGRATION_LIMIT    (5.0f)

#define PID_ANGLE_ROLL_OUTPUT_LIMIT       (10.0f)
#define PID_ANGLE_PITCH_OUTPUT_LIMIT      (10.0f)
#define PID_ANGLE_YAW_OUTPUT_LIMIT        (1.0f)

#define PID_RATE_ROLL_OUTPUT_LIMIT        (10.0f)
#define PID_RATE_PITCH_OUTPUT_LIMIT       (10.0f)
#define PID_RATE_YAW_OUTPUT_LIMIT         (1.0f)


#define PID_POS_X_INTEGRATION_LIMIT      (5.0f)
#define PID_POS_Y_INTEGRATION_LIMIT      (5.0f)
#define PID_POS_Z_INTEGRATION_LIMIT      (1.0f)

#define PID_VELOCITY_X_INTEGRATION_LIMIT (5.0f)
#define PID_VELOCITY_Y_INTEGRATION_LIMIT (5.0f)
#define PID_VELOCITY_Z_INTEGRATION_LIMIT (0.5f)

#define PID_POS_X_OUTPUT_LIMIT           (10.0f)
#define PID_POS_Y_OUTPUT_LIMIT           (10.0f)
#define PID_POS_Z_OUTPUT_LIMIT           (1.0f)

#define PID_VELOCITY_X_OUTPUT_LIMIT      (1.0f)
#define PID_VELOCITY_Y_OUTPUT_LIMIT      (1.0f)
#define PID_VELOCITY_Z_OUTPUT_LIMIT      (1.0f)

#define DEFUALT_THRUST_BASE (0.0f)
#define DEFUALT_PIOLT_MODE  (0)
#define DEFUALT_LOCK_STATUS (1)
#define DEFUALT_DT          (0.05f)
#define DEFUALT_YAW_GAIN    (1.0f)
#define DEFUALT_Z_GAIN      (0.005f)
#define DEFUALT_YAW_BASE    (30000.0f)
#define DEFUALT_YAW_LIMIT   (180.0f)
#define DEFUALT_DEPTH_MIN   (-300.0f)//向上为正
#define DEFUALT_DEPTH_MAN   (0.0f)  //向上为正
#define DEFUALT_ALT_SOURCE  (0)  //默认高度来源是测距声呐
#define DEFUALT_ALT_MIN  (0)  //默认高度最小值
#define DEFUALT_ALT_MAX  (20)  //默认高度最大值
#define DEFUALT_ACCURACY (0.02)  //默认控制精度

#define OUTPUT_TYPE   (1)  //输出类型，0:输出控制器的计算值（没有动力分配），1:输出推进器的值（加动力分配）

#define LIMIT(x, min, max)   (x<min? min:(x>max? max:x))
//exp函数系数
#define COEF_a   (0.25)     //a越大，平缓段越长；a越小，平缓段越短
#define COEF_b   (2)        //大斜率段平移


typedef enum{
  PIOLT_MODE_NONE           = 0,  // Default value
  PIOLT_MODE_MANUAL         = 1,  //手动模式
  PIOLT_MODE_STABILIZE1     = 2,   //稳定模式，定深与定向
  PIOLT_MODE_STABILIZE2     = 3,   //稳定模式，定高与定向
  PIOLT_MODE_AUTODEPTH      = 4,
  PIOLT_MODE_AUTODHIGHT     = 5,
  PIOLT_MODE_AUTODIRCETION  = 6,
  PIOLT_MODE_AUTOHOLD       = 7,
  PIOLT_MODE_MISSION_R      = 8,
  PIOLT_MODE_MISSION        = 9
} Ctrl_Mode;

typedef enum{
  HEIGHT_FROM_SONAR   = 0,  // 高度数据来源于测距声呐
  HEIGHT_FROM_DVL     = 1,  // 高度数据来源于DVL
  HEIGHT_FROM_IMU     = 2   // 高度数据来源于IMU
} Alt_Source;


typedef struct{
  float x;      //前向指令
  float y;      //侧向指令
  float z;      //垂向
  float yaw;    //航向指令
}Mtwist_t;

typedef struct{
  geometry_msgs::msg::Point angle;   //角度
  geometry_msgs::msg::Point rate;    //角速度
  geometry_msgs::msg::Point pos;     //位置
  geometry_msgs::msg::Point vel;     //速度
  float sonar_height;  //声呐高度
  float depth; //深度计深度
  float dvl_alt; //dvl高度
  float imu_alt; //imu高度
  uint8_t get_status;  //是否获得状态标质量，0:未获得， 1:获得
  float yaw_base; //基准航向，用于航向范围锁定,当切换到定向时，设为当前艏向角
  uint8_t reset_target_yaw_flag; //m目标航向角重置标志量
}Status_t;

typedef struct{
  float dt;  //循环时间间隔,单位s;
  float thrust_base;  //基础油门
  float yaw_gain; //航向增益
  float yaw_limit; //航向角限幅
  float z_gain; //垂向增益
  float depth_min; //最小深度
  float depth_max; //最大深度
  uint8_t alt_source; //高度数据来源，0:测距声呐，1:dvl, 2:imu, 
  float height_min; //最小高度，由高度计决定
  float height_max; //最大高度，由高度计决定
  float ctrl_accuracy;  //控制精度
}Cfg_t;

class Controller : public rclcpp::Node{
public:
  Controller(std::string node_name);
  ~Controller();

private:
  void controller_init();
  void attitude_controller_init(void);
  void position_controller_init(void);
  void attitude_controller_reset(void);
  void position_controller_reset(void);
  void setpoint_mapping(void);
  void process_yaw_setpoint(void);
  void process_z_setpoint(void);
  void manual_controller(void);
  void attitude_controller_update(void);
  void position_controller_update(void);
  void controller_mode_sw(void);
  void control_output(void);

  void attitude_angle_pid(geometry_msgs::msg::Point* rate_desired, const geometry_msgs::msg::Point attitude_desired,
     const geometry_msgs::msg::Point attitude_actual);
  void attitude_rate_pid(geometry_msgs::msg::Point* rate_output, const geometry_msgs::msg::Point rate_desired,
     const geometry_msgs::msg::Point gyro_actual);
  void position_pos_pid(geometry_msgs::msg::Point* vel_desired, const geometry_msgs::msg::Point pos_desired,
  const geometry_msgs::msg::Point pos_actual);
  void position_velocity_pid(geometry_msgs::msg::Point* vel_output, const geometry_msgs::msg::Point vel_desired,
  const geometry_msgs::msg::Point vel_actual);
  float pid_update(Pid_Object *pid, const float error);
  float rate_pid_update(Pid_Object *pid, const float error);
  float normalize_float(float value, float original_min, float original_max, float new_min, float new_max);
  void controller_step(void);
  void clear_output(void);
  void Run();
  void timer_20HZ_callback();
  void timer_10HZ_callback();
  void timer_1HZ_callback();

  void TwistCmd_callback(const sealien_ctrlpilot_msgmanagement::msg::TwistCmd& msg);
  void Imu_callback(const sealien_ctrlpilot_msgmanagement::msg::ImuNavStatus& msg);
  void Depth_callback(const sealien_ctrlpilot_msgmanagement::msg::DepthStatus& msg);
  void Height_callback(const sealien_ctrlpilot_msgmanagement::msg::SonarAltimeterStatus& msg);
  void Thru_Cmd_Mix(void);
  rclcpp::TimerBase::SharedPtr timer_cycle_20HZ;
  rclcpp::TimerBase::SharedPtr timer_cycle_10HZ;
  rclcpp::TimerBase::SharedPtr timer_cycle_1HZ;


  Pid_Object pid_angle_roll;
  Pid_Object pid_angle_pitch;
  Pid_Object pid_angle_yaw;

  Pid_Object pid_rate_roll;
  Pid_Object pid_rate_pitch;
  Pid_Object pid_rate_yaw;

  Pid_Object pid_x;
  Pid_Object pid_y;
  Pid_Object pid_z;

  Pid_Object pid_vx;
  Pid_Object pid_vy;
  Pid_Object pid_vz;

  Status_t status;   //当前状态
  Cfg_t config;  // 配置参数
  Mtwist_t manual_output;           //手动模式输出
  Mtwist_t controller_output;       //控制器输出输出
  sealien_ctrlpilot_msgmanagement::msg::TwistCmd output;     //最终输出
  geometry_msgs::msg::Point pos_target;
  geometry_msgs::msg::Point angle_target;
  sealien_ctrlpilot_msgmanagement::msg::ThrusterCmd thru_cmd;  //推进器指令
  sealien_ctrlpilot_msgmanagement::msg::TwistCmd twist_cmd; //直接从遥控器读到的操纵数据，四个操纵两,模式，锁状态
  rclcpp::Publisher<sealien_ctrlpilot_msgmanagement::msg::TwistCmd>::SharedPtr control_output_publisher; 
  rclcpp::Publisher<sealien_ctrlpilot_msgmanagement::msg::ThrusterCmd>::SharedPtr thru_cmd_publisher; 
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_angle_publisher; 
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pos_publisher; 
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr test_publisher; 

  rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::TwistCmd>::SharedPtr TwistCmd_subscriber;      //订阅遥控指令
  rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::ImuNavStatus>::SharedPtr imu_subscriber;                //订阅状态数据
  rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::DepthStatus>::SharedPtr  depth_subscriber;     //订阅深度数据
  rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::SonarAltimeterStatus>::SharedPtr height_subscriber;    //订阅高度数据
  // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_output_publisher; 
  
  // void ThruCmd_callback(const sealien_ctrlpilot_msgmanagement::msg::ThruCmd & msg);       
  // rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::ThruCmd>::SharedPtr ThruCmd_subscriber;           //订阅速度指令

};

