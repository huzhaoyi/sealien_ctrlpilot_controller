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

  virtual void update(void) = 0;
  virtual void reset(void) = 0;
  virtual void output(void) = 0;
};


//NONE
class PilotNone:public CtrModeBase{
public:
  PilotNone(Controller* controller):CtrModeBase(controller){}
  ~PilotNone(){}

  virtual void update(void);
  virtual void reset(void);
  virtual void output(void);
};


//manual
class PilotManual:public CtrModeBase{
public:
  PilotManual(Controller* controller):CtrModeBase(controller){}
  ~PilotManual(){}

  virtual void update(void);
  virtual void reset(void);
  virtual void output(void);
};


//PILOT_MODE_MISSION
class PilotMission:public CtrModeBase{
public:
  PilotMission(Controller* controller):CtrModeBase(controller){}
  ~PilotMission(){}

  virtual void update(void);
  virtual void reset(void);
  virtual void output(void);
};


} //end namespace ControllerNS



