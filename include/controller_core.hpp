#pragma once
#include "rclcpp/rclcpp.hpp"
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>
#include <thread>

#include "pid.hpp"
#include "ctr_mode.hpp"
#include <map>
#include <memory>
#include <GeographicLib/LocalCartesian.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/twist_cmd.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/thruster_cmd.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/follow_cmd.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/gs_cmd.hpp"

#include "sealien_ctrlpilot_msgmanagement/msg/wire_displacement_status.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/pitch_motor_cmd.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/plunger_pump_cmd.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/switch_cmd.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/switch_status.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/task_pos_cmd.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "sealien_ctrlpilot_msgmanagement/action/percent_target.hpp"


namespace ControllerNS{

#define PID_ANGLE_PITCH_INTEGRATION_LIMIT (1.0f)
#define PID_ANGLE_YAW_INTEGRATION_LIMIT   (1.0f)

#define PID_RATE_PITCH_INTEGRATION_LIMIT  (1.0f)
#define PID_RATE_YAW_INTEGRATION_LIMIT    (1.0f)

#define PID_ANGLE_PITCH_OUTPUT_LIMIT      (1.0f)
#define PID_ANGLE_YAW_OUTPUT_LIMIT        (1.0f)

#define PID_RATE_PITCH_OUTPUT_LIMIT       (11.0f)
#define PID_RATE_YAW_OUTPUT_LIMIT         (1.0f)

#define PID_VELOCITY_X_INTEGRATION_LIMIT (5.0f)
#define PID_VELOCITY_X_OUTPUT_LIMIT      (1.0f)

#define DEFUALT_PILOT_MODE  (0)
#define DEFUALT_LOCK_STATUS (1)

#define MAX_GS_ANGLE (45.0)   //舵机范围±45°

#define LIMIT(x, min, max)   (x<min? min:(x>max? max:x))
#define DEADZONE(x, down, up) (x<down? x:(x>up? x:0))
#define NORMALIZE_YAW(x) (x>M_PI? (x- 2*M_PI):(x<-M_PI? (x+ 2*M_PI):x))
// #define DEG2RAD (180.0/M_PI)
#define RAD2DEG (180.0/M_PI)

typedef enum{
  PILOT_MODE_NONE           = 0,    // Default value
  PILOT_MODE_MANUAL         = 1,    //手动模式
  PILOT_MODE_MISSION        = 9,     //路径跟踪
} Ctrl_Mode;

typedef struct{
  float x;      //前向指令
  float y;      //侧向指令
  float z;      //垂向
  float roll;     
  float pitch;     
  float yaw;    //航向指令
}Mtwist_t;

typedef struct{
  float velx;         //前向速度   （m/s）
  float pitch_angle;  //垂向速度   （rad）
  float yaw_rate;     //航向角速度 （rad）
}Target_t;

typedef struct{
  geometry_msgs::msg::Point angle;   //角度rad
  geometry_msgs::msg::Point rate;    //角速度rad/s
  geometry_msgs::msg::Point pos;     //位置m
  geometry_msgs::msg::Point vel;     //速度m/s
  bool get_status;  //是否获得状态标志量，0:未获得， 1:获得

  float sensor_displace_oilbladder;  //油囊拉线传感器位移，单位%
  float sensor_displace_pitchmotor;  //俯仰电机拉线传感器位移，单位%
  bool valve1_status;   //阀1状态，false:关， true:开
  bool valve2_status;   //阀2状态，false:关， true:开

}Status_t;

class CtrModeBase;

class Controller : public rclcpp::Node{
public:
  using PercentTarget = sealien_ctrlpilot_msgmanagement::action::PercentTarget;
  using GoalHandlePercentTarget = rclcpp_action::ServerGoalHandle<PercentTarget>;
  using msg_FollowCmd = sealien_ctrlpilot_msgmanagement::msg::FollowCmd;

  Controller(std::string node_name);
  ~Controller();

  void clear_output(void);

  Pid_Object pid_angle_pitch;
  Pid_Object pid_rate_pitch;
  Pid_Object pid_rate_yaw;
  Pid_Object pid_vx;

  Status_t status;   //当前状态
  Mtwist_t output;     //控制器输出
  Target_t target_cmd;  //制动控制时的指令
  sealien_ctrlpilot_msgmanagement::msg::TwistCmd twist_cmd; //直接从遥控器读到的操纵数据，四个操纵量,模式，锁状态

  std::map<int, std::shared_ptr<CtrModeBase>> ModeMap;
  bool follow_direct;  //跟踪的方向，0：前进，1：后退
  float dt;   //循环时间间隔

  int gs1_dir;  //舵机正反
  int gs2_dir;
  int gs3_dir;
  int gs4_dir;
private:
  bool isModelegal(int ctrlmod);
  void controller_init();
  void Thru_Cmd_Mix(void);

  void controller_mode_sw(void);
  void control_output(void);

  void controller_step(void);
  void Run();
  void timer_20HZ_callback();
  void timer_1HZ_callback();

  void TwistCmd_callback(const sealien_ctrlpilot_msgmanagement::msg::TwistCmd& msg);
  void RovOdom_callback(const nav_msgs::msg::Odometry& msg);
  void LocateOdom_callback(const nav_msgs::msg::Odometry& msg);
  void trackCmd_callback(const msg_FollowCmd& msg);

  void displacement_callback(const sealien_ctrlpilot_msgmanagement::msg::WireDisplacementStatus& msg);
  void Switchs_callback(const sealien_ctrlpilot_msgmanagement::msg::SwitchStatus& msg);

  rclcpp_action::GoalResponse oilBladder_handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const PercentTarget::Goal> goal);
  rclcpp_action::CancelResponse oilBladder_handle_cancel(const std::shared_ptr<GoalHandlePercentTarget> goal_handle);
  void oilBladder_handle_accepted(const std::shared_ptr<GoalHandlePercentTarget> goal_handle);
  void oilBladder_execute(const std::shared_ptr<GoalHandlePercentTarget> goal_handle);

  rclcpp_action::GoalResponse pitchMotor_handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const PercentTarget::Goal> goal);
  rclcpp_action::CancelResponse pitchMotor_handle_cancel(const std::shared_ptr<GoalHandlePercentTarget> goal_handle);
  void pitchMotor_handle_accepted(const std::shared_ptr<GoalHandlePercentTarget> goal_handle);
  void pitchMotor_execute(const std::shared_ptr<GoalHandlePercentTarget> goal_handle);

  rclcpp::TimerBase::SharedPtr timer_cycle_20HZ;
  rclcpp::TimerBase::SharedPtr timer_cycle_1HZ;

  rclcpp::Publisher<sealien_ctrlpilot_msgmanagement::msg::TwistCmd>::SharedPtr control_output_publisher; 
  rclcpp::Publisher<sealien_ctrlpilot_msgmanagement::msg::ThrusterCmd>::SharedPtr thru_cmd_publisher; 
  rclcpp::Publisher<sealien_ctrlpilot_msgmanagement::msg::GsCmd>::SharedPtr gs_cmd_publisher; 

  rclcpp::Publisher<sealien_ctrlpilot_msgmanagement::msg::PitchMotorCmd>::SharedPtr pitch_cmd_publisher; 
  rclcpp::Publisher<sealien_ctrlpilot_msgmanagement::msg::PlungerPumpCmd>::SharedPtr pump_cmd_publisher; 
  rclcpp::Publisher<sealien_ctrlpilot_msgmanagement::msg::SwitchCmd>::SharedPtr switch_cmd_publisher; 
  rclcpp::Publisher<sealien_ctrlpilot_msgmanagement::msg::TaskPosCmd>::SharedPtr pid_output_publisher;  //for debug
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gs_output_publisher;  //for debug

  rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::TwistCmd>::SharedPtr TwistCmd_subscriber;      //订阅遥控指令
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr RovOdom_subscriber;                //订阅状态数据
  rclcpp::Subscription<msg_FollowCmd>::SharedPtr track_cmd_subscriber;   //订阅路径跟踪模块下发的速度

  rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::WireDisplacementStatus>::SharedPtr displacement_status_subscriber;   
  rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::SwitchStatus>::SharedPtr valve_status_subscriber;   

  rclcpp_action::Server<PercentTarget>::SharedPtr oilBladder_server_;  //油囊
  rclcpp_action::Server<PercentTarget>::SharedPtr pitchMotor_server_;  //俯仰舵机

};



} //end namespace ControllerNS
