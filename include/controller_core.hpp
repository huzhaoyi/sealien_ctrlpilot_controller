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
#include "sealien_ctrlpilot_msgmanagement/msg/twist_cmd.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/imu_nav_status.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/depth_status.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/sonar_altimeter_status.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/thruster_cmd.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/task_pos_cmd.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/follow_cmd.hpp"

#define msg_FollowCmd sealien_ctrlpilot_msgmanagement::msg::FollowCmd


namespace ControllerNS{

#define PID_ANGLE_ROLL_INTEGRATION_LIMIT  (1.0f)
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT (1.0f)
#define PID_ANGLE_YAW_INTEGRATION_LIMIT   (1.0f)

#define PID_RATE_ROLL_INTEGRATION_LIMIT   (1.0f)
#define PID_RATE_PITCH_INTEGRATION_LIMIT  (1.0f)
#define PID_RATE_YAW_INTEGRATION_LIMIT    (1.0f)

#define PID_ANGLE_ROLL_OUTPUT_LIMIT       (1.0f)
#define PID_ANGLE_PITCH_OUTPUT_LIMIT      (1.0f)
#define PID_ANGLE_YAW_OUTPUT_LIMIT        (1.0f)

#define PID_RATE_ROLL_OUTPUT_LIMIT        (1.0f)
#define PID_RATE_PITCH_OUTPUT_LIMIT       (11.0f)
#define PID_RATE_YAW_OUTPUT_LIMIT         (1.0f)


#define PID_POS_X_INTEGRATION_LIMIT      (1.0f)
#define PID_POS_Y_INTEGRATION_LIMIT      (1.0f)
#define PID_POS_Z_INTEGRATION_LIMIT      (1.0f)

#define PID_VELOCITY_X_INTEGRATION_LIMIT (5.0f)
#define PID_VELOCITY_Y_INTEGRATION_LIMIT (5.0f)
#define PID_VELOCITY_Z_INTEGRATION_LIMIT (0.5f)

#define PID_POS_X_OUTPUT_LIMIT           (1.0f)
#define PID_POS_Y_OUTPUT_LIMIT           (1.0f)
#define PID_POS_Z_OUTPUT_LIMIT           (1.0f)

#define PID_VELOCITY_X_OUTPUT_LIMIT      (1.0f)
#define PID_VELOCITY_Y_OUTPUT_LIMIT      (1.0f)
#define PID_VELOCITY_Z_OUTPUT_LIMIT      (1.0f)

#define DEFUALT_THRUST_BASE (0.0f)
#define DEFUALT_PILOT_MODE  (0)
#define DEFUALT_LOCK_STATUS (1)
#define DEFUALT_DT          (0.05f)
#define DEFUALT_YAW_GAIN    (1.0f)
#define DEFUALT_XY_GAIN     (0.005f)
#define DEFUALT_Z_GAIN      (0.005f)
#define DEFUALT_YAW_BASE    (30000.0f)
#define DEFUALT_YAW_LIMIT   (180.0f)
#define DEFUALT_DEPTH_MIN   (-100.0f)//向上为正
#define DEFUALT_DEPTH_MAN   (0.0f)  //向上为正
#define DEFUALT_X_MIN   (-100000.0f)
#define DEFUALT_X_MAN   (100000.0f)  
#define DEFUALT_Y_MIN   (-100000.0f)
#define DEFUALT_Y_MAN   (100000.0f)  
#define DEFUALT_ALT_SOURCE  (0)  //默认高度来源是测距声呐
#define DEFUALT_ALT_MIN  (0)  //默认高度最小值
#define DEFUALT_ALT_MAX  (20)  //默认高度最大值
#define DEFUALT_ACCURACY (0.02)  //默认控制精度
#define DEFUALT_USE_ROLLPITCH (false)  //默认不使用
#define DEFUALT_REF_ALT (0)  //
#define DEFUALT_TRACK_ALT_DEPTH (0)  //

#define LIMIT(x, min, max)   (x<min? min:(x>max? max:x))
#define DEADZONE(x, down, up) (x<down? x:(x>up? x:0))
#define NORMALIZE_YAW(x) (x>M_PI? (x- 2*M_PI):(x<-M_PI? (x+ 2*M_PI):x))
// #define DEG2RAD (180.0/M_PI)
#define RAD2DEG (180.0/M_PI)
//exp函数系数
#define COEF_a   (0.25)     //a越大，平缓段越长；a越小，平缓段越短
#define COEF_b   (2)        //大斜率段平移


typedef enum{
  PILOT_MODE_NONE           = 0,    // Default value
  PILOT_MODE_MANUAL         = 1,    //手动模式
  PILOT_MODE_STABILIZE1     = 2,    //稳定模式，定深与定向
  PILOT_MODE_STABILIZE2     = 3,    //稳定模式，定高与定向
  PILOT_MODE_AUTODEPTH      = 4,    //定深
  PILOT_MODE_AUTODHIGHT     = 5,    //定高
  PILOT_MODE_AUTODIRCETION  = 6,    //定向
  PILOT_MODE_AUTOHOLD1      = 7,    //x、y、z位置保持,z轴是定深
  PILOT_MODE_AUTOHOLD2      = 8,    //x、y、z位置保持,z轴是定高
  PILOT_MODE_MISSION1       = 9,     //路径跟踪，z轴是定深
  PILOT_MODE_MISSION2       = 10    //路径跟踪，z轴是定高
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
  float roll;     
  float pitch;     
  float yaw;    //航向指令
}Mtwist_t;

typedef struct{
  geometry_msgs::msg::Point angle;   //角度rad
  geometry_msgs::msg::Point rate;    //角速度rad/s
  geometry_msgs::msg::Point pos;     //位置m
  geometry_msgs::msg::Point vel;     //速度m/s
  double alt;  //高度m
  bool get_status;  //是否获得状态标质量，0:未获得， 1:获得
  float yaw_base; //基准航向，用于航向范围锁定,当切换到定向时，设为当前艏向角
  uint8_t reset_target_yaw_flag; //m目标航向角重置标志量
  bool track_status; //路径跟踪状态
  float angle_add;  //角度指令累加值
}Status_t;

typedef struct{
  float dt;  //循环时间间隔,单位s;
  float thrust_base;  //基础油门
  float yaw_gain; //航向增益
  float yaw_limit; //航向角限幅
  float z_gain; //垂向增益
  float xy_gain; //横向、纵向增益
  float depth_min; //最小深度
  float depth_max; //最大深度
  float x_min; //最小距离
  float x_max; //最大距离
  float y_min; //最小距离
  float y_max; //最大距离
  float height_min; //最小高度，由高度计决定
  float height_max; //最大高度，由高度计决定
  float ctrl_accuracy;  //控制精度
  bool use_rollpitch_ctrl;  //俯仰滚转角控制，0:不启用，1:启用
}Cfg_t; 


class CtrModeBase;

class Controller : public rclcpp::Node{
public:
  Controller(std::string node_name);
  ~Controller();

  void clear_output(void);
  void process_yaw_setpoint(void);
  void process_z_setpoint(void);
  void process_xy_setpoint(void);
  void manual_controller_update(geometry_msgs::msg::Point& rate_output, geometry_msgs::msg::Point& vel_output);
  void attitude_controller_update(geometry_msgs::msg::Point& rate_output);
  void position_controller_update(geometry_msgs::msg::Point& vel_output, float z_status);
  void attitude_controller_reset(void);
  void position_controller_reset(float z_status);
  bool isTaskFinish(void);   //判断任务是否完成
  void TaskFinishPub(void);   //判断任务是否完成
  bool isPosCtrlMode(int ctrlmod);
  bool isModelegal(int ctrlmod);

  void attitude_angle_pid(geometry_msgs::msg::Point& rate_desired, const geometry_msgs::msg::Point attitude_desired,
    const geometry_msgs::msg::Point attitude_actual);
  void attitude_rate_pid(geometry_msgs::msg::Point& rate_output, const geometry_msgs::msg::Point rate_desired,
    const geometry_msgs::msg::Point gyro_actual);
  void position_pos_pid(geometry_msgs::msg::Point& vel_desired, const geometry_msgs::msg::Point pos_desired,
  const geometry_msgs::msg::Point pos_actual);
  void position_velocity_pid(geometry_msgs::msg::Point& vel_output, const geometry_msgs::msg::Point vel_desired,
  const geometry_msgs::msg::Point vel_actual);

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
  float x_target_base;  //x轴位置基础值，切换到位置模式时置为当前值
  float x_target_delta;  //x轴位置遥控器增量值，累加
  float y_target_base;   //y轴位置基础值，切换到位置模式时置为当前值
  float y_target_delta; //y轴位置遥控器增量值，累加
  geometry_msgs::msg::Point angle_target;
  geometry_msgs::msg::Point rate_target;  //路径跟踪时的角速度指令，只使用航向角速度
  geometry_msgs::msg::Point vel_target;   //路径跟踪时的速度指令
  sealien_ctrlpilot_msgmanagement::msg::ThrusterCmd thru_cmd;  //推进器指令
  sealien_ctrlpilot_msgmanagement::msg::TwistCmd twist_cmd; //直接从遥控器读到的操纵数据，四个操纵两,模式，锁状态

  std::map<int, std::shared_ptr<CtrModeBase>> ModeMap;
  // bool track_status; //路径跟踪时的状态，0:跟踪任务结束，1:跟踪任务还未结束
  bool have_new_track_status; //有新的路径状态，用于检测路径状态更新

  geometry_msgs::msg::Point follow_target_pos; //跟踪的最终目标位置
  double follow_target_ang;//跟踪的最终目标角度
  bool follow_direct;  //跟踪的方向，0：前进，1：后退
  bool got_follow_target;  //是否获取跟踪路径，路径跟踪时使用
  bool got_task_target;   //是否获取任务，包括跟踪路径和跟踪目标点，在任务模式时使用。
private:
  void controller_init();
  void attitude_controller_init(void);
  void position_controller_init(void);
  void setpoint_mapping(void);
  void Thru_Cmd_Mix(void);
  void pointOdomToBaseLink(geometry_msgs::msg::Point p_in,  geometry_msgs::msg::Point& p_out);
  void pointBaseLinkToOdom(geometry_msgs::msg::Point p_in,  geometry_msgs::msg::Point& p_out);
 
  void controller_mode_sw(void);
  void control_output(void);

  void controller_step(void);
  void Run();
  void timer_20HZ_callback();
  void timer_10HZ_callback();
  void timer_1HZ_callback();

  void TwistCmd_callback(const sealien_ctrlpilot_msgmanagement::msg::TwistCmd& msg);
  void RovOdom_callback(const nav_msgs::msg::Odometry& msg);
  void Height_callback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);
  void LocateOdom_callback(const nav_msgs::msg::Odometry& msg);
  void trackCmd_callback(const msg_FollowCmd& msg);
  void PathTrackStatus_callback(const std_msgs::msg::Bool& msg);
  void taskPoseCmd_callback(const sealien_ctrlpilot_msgmanagement::msg::TaskPosCmd& msg);

  rclcpp::TimerBase::SharedPtr timer_cycle_20HZ;
  rclcpp::TimerBase::SharedPtr timer_cycle_10HZ;
  rclcpp::TimerBase::SharedPtr timer_cycle_1HZ;

  rclcpp::Publisher<sealien_ctrlpilot_msgmanagement::msg::TwistCmd>::SharedPtr control_output_publisher; 
  rclcpp::Publisher<sealien_ctrlpilot_msgmanagement::msg::ThrusterCmd>::SharedPtr thru_cmd_publisher; 
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_angle_publisher; 
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pos_publisher; 
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr test_publisher; 
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr task_finish_publisher; 

  rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::TwistCmd>::SharedPtr TwistCmd_subscriber;      //订阅遥控指令
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr RovOdom_subscriber;                //订阅状态数据
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr height_subscriber;    //订阅高度数据
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pathTrackStatus_subscriber;    //订阅路径跟踪状态
  rclcpp::Subscription<msg_FollowCmd>::SharedPtr track_cmd_subscriber;   //订阅路径跟踪模块下发的速度
  rclcpp::Subscription<sealien_ctrlpilot_msgmanagement::msg::TaskPosCmd>::SharedPtr taskPosCmd_subscriber;  //订阅任务模块的指令


};



} //end namespace ControllerNS
