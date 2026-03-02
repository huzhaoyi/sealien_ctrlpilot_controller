/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******header file****
* File name          : ctr_mode.hpp
* Author             : Yi Lu
* Brief              : 
********************************************************************************
* modify
* Version   Date                Author              Described
* V1.00     2026/2/11            Yi Lu               Created
*******************************************************************************/
#pragma once
#include "rclcpp/rclcpp.hpp"
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>
#include <controller_core.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace ControllerNS{

class Controller;

//基类
class CtrModeBase{
public:
  CtrModeBase(Controller* controller){ controller_ = controller;}
  ~CtrModeBase(){}

  Controller* controller_;
  geometry_msgs::msg::Point rate_output;  //角速度指令输出
  geometry_msgs::msg::Point vel_output;   //速度指令输出

  virtual void setpoint_mapping(void) = 0;
  virtual void update(void) = 0;
  virtual void reset(void) = 0;
  virtual void output(void) = 0;


};


//NONE
class PilotNone:public CtrModeBase{
public:
  PilotNone(Controller* controller):CtrModeBase(controller){}
  ~PilotNone(){}

  virtual void setpoint_mapping(void);
  virtual void update(void);
  virtual void reset(void);
  virtual void output(void);
};


//manual
class PilotManual:public CtrModeBase{
public:
  PilotManual(Controller* controller):CtrModeBase(controller){}
  ~PilotManual(){}

  virtual void setpoint_mapping(void);
  virtual void update(void);
  virtual void reset(void);
  virtual void output(void);
};

//PILOT_MODE_STABILIZE1
class PilotStablize1:public CtrModeBase{
public:
  PilotStablize1(Controller* controller):CtrModeBase(controller){}
  ~PilotStablize1(){}

  virtual void setpoint_mapping(void);
  virtual void update(void);
  virtual void reset(void);
  virtual void output(void);
};

//PILOT_MODE_STABILIZE2
class PilotStablize2:public CtrModeBase{
public:
  PilotStablize2(Controller* controller):CtrModeBase(controller){}
  ~PilotStablize2(){}

  virtual void setpoint_mapping(void);
  virtual void update(void);
  virtual void reset(void);
  virtual void output(void);
};

//PILOT_MODE_AUTODEPTH
class PilotAutodepth:public CtrModeBase{
public:
  PilotAutodepth(Controller* controller):CtrModeBase(controller){}
  ~PilotAutodepth(){}

  virtual void setpoint_mapping(void);
  virtual void update(void);
  virtual void reset(void);
  virtual void output(void);
};

//PILOT_MODE_AUTODHIGHT
class PilotAutoheight:public CtrModeBase{
public:
  PilotAutoheight(Controller* controller):CtrModeBase(controller){}
  ~PilotAutoheight(){}

  virtual void setpoint_mapping(void);
  virtual void update(void);
  virtual void reset(void);
  virtual void output(void);
};


//PILOT_MODE_AUTODIRCETION
class PilotAutoDirection:public CtrModeBase{
public:
  PilotAutoDirection(Controller* controller):CtrModeBase(controller){}
  ~PilotAutoDirection(){}

  virtual void setpoint_mapping(void);
  virtual void update(void);
  virtual void reset(void);
  virtual void output(void);
};

//PILOT_MODE_AUTOHOLD1
class PilotAutoHold1:public CtrModeBase{
public:
  PilotAutoHold1(Controller* controller):CtrModeBase(controller){}
  ~PilotAutoHold1(){}

  virtual void setpoint_mapping(void);
  virtual void update(void);
  virtual void reset(void);
  virtual void output(void);
};

//PILOT_MODE_AUTOHOLD2
class PilotAutoHold2:public CtrModeBase{
public:
  PilotAutoHold2(Controller* controller):CtrModeBase(controller){}
  ~PilotAutoHold2(){}

  virtual void setpoint_mapping(void);
  virtual void update(void);
  virtual void reset(void);
  virtual void output(void);
};


//PILOT_MODE_MISSION
class PilotMission:public CtrModeBase{
public:
  PilotMission(Controller* controller):CtrModeBase(controller){ lost_track_status_count = 0;}
  ~PilotMission(){}

  virtual void setpoint_mapping(void);
  virtual void update(void);
  virtual void reset(void);
  virtual void output(void);

  void reset_count(void){lost_track_status_count = 0;}
private:
  uint32_t lost_track_status_count;  //计时，判断是否丢失路径跟踪状态

};











} //end namespace ControllerNS



