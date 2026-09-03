/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****
* File name          : ctr_mode.cpp
* Author             : Yi Lu
* Brief              : 
********************************************************************************
* modify
* Version   Date                Author              Described
* V1.00     2026/2/11            Yi Lu               Created
*******************************************************************************/

#include "ctr_mode.hpp"

namespace ControllerNS{

/****************NONE模式*******************/
void PilotNone::update(void){
  controller_->clear_output();
}

void PilotNone::reset(void){
  controller_->clear_output();
}

void PilotNone::output(void){
  controller_->output.x = 0.0;
  controller_->output.y = 0.0;
  controller_->output.z = 0.0;
  controller_->output.roll  = 0.0;
  controller_->output.pitch = 0.0;
  controller_->output.yaw   = 0.0;
}

/****************手动模式*******************/
 void PilotManual::update(void){
  vel_output.x = controller_->twist_cmd.x;
  vel_output.y = 0.0;
  vel_output.z = 0.0;
  
  rate_output.x = 0.0;
  rate_output.y = controller_->twist_cmd.pitch;
  rate_output.z = controller_->twist_cmd.yaw;
 }

 void PilotManual::reset(void){
  
 }

 void PilotManual::output(void){
  controller_->output.x = vel_output.x;
  controller_->output.y = vel_output.y;
  controller_->output.z = vel_output.z;
  controller_->output.roll  = rate_output.x;
  controller_->output.pitch = rate_output.y;
  controller_->output.yaw   = rate_output.z;
 }

/****************自动模式*******************/
void PilotMission::update(void){
  float x_vel_error;   //x轴方向的速度误差
  float pitch_error;   //俯仰角误差
  float pitch_rate_desired;   //俯仰角速度期望值
  float pitch_rate_error;     //俯仰角速度误差
  float yaw_rate_error;       //航向角速率误差

  // 前向速度闭环：velx 为速度目标 (m/s)，走 pid_vx
  x_vel_error = controller_->target_cmd.velx - controller_->status.vel.x;
  vel_output.x = controller_->pid_vx.pid_update(x_vel_error);

  //俯仰
  pitch_error = controller_->target_cmd.pitch_angle - controller_->status.angle.y;
  pitch_error = LIMIT(pitch_error, -M_PI/6, M_PI/6);
  pitch_rate_desired = controller_->pid_angle_pitch.pid_update(pitch_error);

  pitch_rate_error  = pitch_rate_desired - controller_->status.rate.y;
  rate_output.y = controller_->pid_rate_pitch.pid_update(pitch_rate_error);

  //航向
  yaw_rate_error = controller_->target_cmd.yaw_rate - controller_->status.rate.z;
  rate_output.z = controller_->pid_rate_yaw.pid_update(yaw_rate_error);
}

void PilotMission::reset(void){
  controller_->pid_angle_pitch.reset();
  controller_->pid_rate_pitch.reset();
  controller_->pid_rate_yaw.reset();
  controller_->pid_vx.reset();
}

void PilotMission::output(void){
  controller_->output.x = vel_output.x;
  controller_->output.y = vel_output.y;
  controller_->output.z = vel_output.z;

  controller_->output.roll  = rate_output.x;
  controller_->output.pitch = -rate_output.y;
  controller_->output.yaw   = rate_output.z;

}

} //end namespace ControllerNS