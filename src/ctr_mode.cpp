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
void PilotNone::setpoint_mapping(void){
 //do nothing
};

void PilotNone::update(void){
  controller_->clear_output();
};

void PilotNone::reset(void){
  controller_->clear_output();
};

void PilotNone::output(void){
  controller_->output.x = 0.0;
  controller_->output.y = 0.0;
  controller_->output.z = 0.0;
  controller_->output.roll  = 0.0;
  controller_->output.pitch = 0.0;
  controller_->output.yaw   = 0.0;
};


/****************手动模式*******************/
 void PilotManual::setpoint_mapping(void){
  //do nothing
 }

 void PilotManual::update(void){
  controller_->manual_controller_update(rate_output, vel_output);

  controller_->manual_output.x = vel_output.x;
  controller_->manual_output.y = vel_output.y;
  controller_->manual_output.z = vel_output.z;

  controller_->manual_output.roll   = rate_output.x;
  controller_->manual_output.pitch  = rate_output.y;
  controller_->manual_output.yaw    = rate_output.z;
 }

 void PilotManual::reset(void){
  //do nothing
 }

 void PilotManual::output(void){
  controller_->output.x = vel_output.x;
  controller_->output.y = vel_output.y;
  controller_->output.z = vel_output.z;
  controller_->output.roll  = rate_output.x;
  controller_->output.pitch = rate_output.y;
  controller_->output.yaw   = rate_output.z;
 }



/****************稳定模式1*******************/
void PilotStablize1::setpoint_mapping(void){
  controller_->process_yaw_setpoint();
  controller_->process_z_setpoint();
};

void PilotStablize1::update(void){
 controller_->attitude_controller_update(rate_output);
 controller_->position_controller_update(vel_output, controller_->status.depth);
};
void PilotStablize1::reset(void){
  controller_->attitude_controller_reset();
  controller_->position_controller_reset(controller_->status.depth);
};
void PilotStablize1::output(void){
  controller_->output.x = controller_->manual_output.x;
  controller_->output.y = controller_->manual_output.y;
  controller_->output.z = vel_output.z;

  controller_->output.roll = rate_output.x;
  controller_->output.pitch = rate_output.y;
  controller_->output.yaw = rate_output.z;
};

/****************稳定模式2*******************/
void PilotStablize2::setpoint_mapping(void){
  controller_->process_yaw_setpoint();
  controller_->process_z_setpoint();
};

void PilotStablize2::update(void){
  controller_->attitude_controller_update(rate_output);

  float height;
  if(controller_->config.alt_source == HEIGHT_FROM_SONAR){// 高度数据来源于测距声呐
    height = controller_->status.sonar_height;
  }else if(controller_->config.alt_source == HEIGHT_FROM_DVL){// 高度数据来源于DVL
    height = controller_->status.dvl_alt;
  }else if(controller_->config.alt_source == HEIGHT_FROM_IMU){// 高度数据来源于IMU
    height = controller_->status.imu_alt;
  }else{
    height = 0.0;
  }

  controller_->position_controller_update(vel_output, height);
};

void PilotStablize2::reset(void){
  controller_->attitude_controller_reset();

  float height;
  if(controller_->config.alt_source == HEIGHT_FROM_SONAR){// 高度数据来源于测距声呐
    height = controller_->status.sonar_height;
  }else if(controller_->config.alt_source == HEIGHT_FROM_DVL){// 高度数据来源于DVL
    height = controller_->status.dvl_alt;
  }else if(controller_->config.alt_source == HEIGHT_FROM_IMU){// 高度数据来源于IMU
    height = controller_->status.imu_alt;
  }else{
    height = 0.0;
  }

  controller_->position_controller_reset(height);
};

void PilotStablize2::output(void){
  controller_->output.x = controller_->manual_output.x;
  controller_->output.y = controller_->manual_output.y;
  controller_->output.z = vel_output.z;

  controller_->output.roll = rate_output.x;
  controller_->output.pitch = rate_output.y;
  controller_->output.yaw = rate_output.z;
};


/****************深度控制*******************/
void PilotAutodepth::setpoint_mapping(void){
  controller_->process_z_setpoint();
};

void PilotAutodepth::update(void){
  controller_->position_controller_update(vel_output, controller_->status.depth);
};

void PilotAutodepth::reset(void){
  controller_->position_controller_reset(controller_->status.depth);
};

void PilotAutodepth::output(void){
  controller_->output.x = controller_->manual_output.x;
  controller_->output.y = controller_->manual_output.y;
  controller_->output.z = controller_->manual_output.z;

  controller_->output.roll = controller_->manual_output.roll;
  controller_->output.pitch = controller_->manual_output.pitch;
  controller_->output.yaw = rate_output.z;
};


/****************高度控制*******************/
void PilotAutoheight::setpoint_mapping(void){
  controller_->process_z_setpoint();
};

void PilotAutoheight::update(void){
  controller_->position_controller_update(vel_output, controller_->status.depth);
};

void PilotAutoheight::reset(void){
  float height;
  if(controller_->config.alt_source == HEIGHT_FROM_SONAR){// 高度数据来源于测距声呐
    height = controller_->status.sonar_height;
  }else if(controller_->config.alt_source == HEIGHT_FROM_DVL){// 高度数据来源于DVL
    height = controller_->status.dvl_alt;
  }else if(controller_->config.alt_source == HEIGHT_FROM_IMU){// 高度数据来源于IMU
    height = controller_->status.imu_alt;
  }else{
    height = 0.0;
  }

  controller_->position_controller_reset(height);
};

void PilotAutoheight::output(void){
  controller_->output.x = controller_->manual_output.x;
  controller_->output.y = controller_->manual_output.y;
  controller_->output.z = controller_->manual_output.z;

  controller_->output.roll = controller_->manual_output.roll;
  controller_->output.pitch = controller_->manual_output.pitch;
  controller_->output.yaw = rate_output.z;
};

/****************艏向控制*******************/
void PilotAutoDirection::setpoint_mapping(void){
  controller_->process_yaw_setpoint();
};

void PilotAutoDirection::update(void){
  controller_->attitude_controller_update(rate_output);
};

void PilotAutoDirection::reset(void){
  controller_->attitude_controller_reset();
};

void PilotAutoDirection::output(void){
  controller_->output.x = controller_->manual_output.x;
  controller_->output.y = controller_->manual_output.y;
  controller_->output.z = controller_->manual_output.z;

  controller_->output.roll = rate_output.x;
  controller_->output.pitch = rate_output.y;
  controller_->output.yaw = rate_output.z;
};

/****************位置保持1*******************/
void PilotAutoHold1::setpoint_mapping(void){
  controller_->process_yaw_setpoint();
  controller_->process_xy_setpoint();
  controller_->process_z_setpoint();
};

void PilotAutoHold1::update(void){
  controller_->attitude_controller_update(rate_output);
  controller_->position_controller_update(vel_output, controller_->status.depth);
}

void PilotAutoHold1::reset(void){
  controller_->attitude_controller_reset();
  controller_->position_controller_reset(controller_->status.depth);
};

void PilotAutoHold1::output(void){
  double posx_error = controller_->pos_target.x - controller_->status.pos.x;
  double curx_vel = controller_->status.vel.x;
  double posy_error = controller_->pos_target.y - controller_->status.pos.y;
  double cury_vel = controller_->status.vel.y;

  controller_->output.x = vel_output.x + controller_->brake(posx_error, curx_vel);
  controller_->output.y = vel_output.y + controller_->brake(posy_error, cury_vel);
  controller_->output.z = vel_output.z;

  controller_->output.roll = rate_output.x;
  controller_->output.pitch = rate_output.y;
  controller_->output.yaw = rate_output.z;
};

/****************位置保持2*******************/
void PilotAutoHold2::setpoint_mapping(void){
  controller_->process_yaw_setpoint();
  controller_->process_xy_setpoint();
  controller_->process_z_setpoint();
};

void PilotAutoHold2::update(void){
  controller_->attitude_controller_update(rate_output);

  float height;
  if(controller_->config.alt_source == HEIGHT_FROM_SONAR){// 高度数据来源于测距声呐
    height = controller_->status.sonar_height;
  }else if(controller_->config.alt_source == HEIGHT_FROM_DVL){// 高度数据来源于DVL
    height = controller_->status.dvl_alt;
  }else if(controller_->config.alt_source == HEIGHT_FROM_IMU){// 高度数据来源于IMU
    height = controller_->status.imu_alt;
  }else{
    height = 0.0;
  }
  controller_->position_controller_update(vel_output, height);
}

void PilotAutoHold2::reset(void){
  controller_->attitude_controller_reset();

  float height;
  if(controller_->config.alt_source == HEIGHT_FROM_SONAR){// 高度数据来源于测距声呐
    height = controller_->status.sonar_height;
  }else if(controller_->config.alt_source == HEIGHT_FROM_DVL){// 高度数据来源于DVL
    height = controller_->status.dvl_alt;
  }else if(controller_->config.alt_source == HEIGHT_FROM_IMU){// 高度数据来源于IMU
    height = controller_->status.imu_alt;
  }else{
    height = 0.0;
  }
  controller_->position_controller_reset(height);
};

void PilotAutoHold2::output(void){
  double posx_error = controller_->pos_target.x - controller_->status.pos.x;
  double curx_vel = controller_->status.vel.x;
  double posy_error = controller_->pos_target.y - controller_->status.pos.y;
  double cury_vel = controller_->status.vel.y;

  controller_->output.x = vel_output.x + controller_->brake(posx_error, curx_vel);
  controller_->output.y = vel_output.y + controller_->brake(posy_error, cury_vel);;
  controller_->output.z = vel_output.z;

  controller_->output.roll = rate_output.x;
  controller_->output.pitch = rate_output.y;
  controller_->output.yaw = rate_output.z;
};

/****************路径跟踪*******************/
void PilotMission::setpoint_mapping(void){
  if(controller_->have_new_track_status){
    controller_->have_new_track_status = false;

    reset();//重置控制器
  }

  if(lost_track_status_count < 60){
    lost_track_status_count++;
  }

  if(lost_track_status_count >= 60 && lost_track_status_count!= 70){  //20HZ更新，大于3秒钟认为丢失路径跟踪节点
    lost_track_status_count = 70;  //防止再次进入
    controller_->status.track_status = false;   //强制状态为跟踪完成
    controller_->have_new_track_status = true;  //标志有新状态
  }
};

void PilotMission::update(void){
  if(controller_->status.track_status){//跟踪进行
    controller_->attitude_rate_pid(rate_output, controller_->rate_target, controller_->status.rate);

    controller_->position_velocity_pid(vel_output, controller_->vel_target, controller_->status.vel);
  
    vel_output.z = controller_->config.thrust_base + vel_output.z;
    // RCLCPP_INFO(controller_->get_logger(), "path track");
  }else{//位置保持
    controller_->attitude_controller_update(rate_output);

    float height;
    if(!controller_->config.track_alt_depth){
      height = controller_->status.depth;
    }else{
      if(controller_->config.alt_source == HEIGHT_FROM_SONAR){// 高度数据来源于测距声呐
        height = controller_->status.sonar_height;
      }else if(controller_->config.alt_source == HEIGHT_FROM_DVL){// 高度数据来源于DVL
        height = controller_->status.dvl_alt;
      }else if(controller_->config.alt_source == HEIGHT_FROM_IMU){// 高度数据来源于IMU
        height = controller_->status.imu_alt;
      }else{
        height = 0.0;
      }
    }

    controller_->position_controller_update(vel_output, height);

    // RCLCPP_INFO(controller_->get_logger(), "pos hold");
  }
}

void PilotMission::reset(void){
  controller_->attitude_controller_reset();
  if(!controller_->config.track_alt_depth){
    controller_->position_controller_reset(controller_->status.depth);
  }else{
    float height;
    if(controller_->config.alt_source == HEIGHT_FROM_SONAR){// 高度数据来源于测距声呐
      height = controller_->status.sonar_height;
    }else if(controller_->config.alt_source == HEIGHT_FROM_DVL){// 高度数据来源于DVL
      height = controller_->status.dvl_alt;
    }else if(controller_->config.alt_source == HEIGHT_FROM_IMU){// 高度数据来源于IMU
      height = controller_->status.imu_alt;
    }else{
      height = 0.0;
    }
    controller_->position_controller_reset(height);
  }

  if(controller_->status.track_status == false){  //从跟踪状态到非跟踪状态
    RCLCPP_INFO(controller_->get_logger(), "path controller_->got_follow_target[%d]",controller_->got_follow_target);
    if(controller_->got_follow_target == false){  //没有路径
      controller_->angle_target.z = controller_->status.yaw_base;
      controller_->pos_target.x = controller_->x_target_base;
      controller_->pos_target.y = controller_->y_target_base;
      RCLCPP_INFO(controller_->get_logger(), "path track  false");
    }else{//有路径
      controller_->pos_target.x = controller_->follow_target_pos.x;
      controller_->pos_target.y = controller_->follow_target_pos.y;
      
      if(controller_->follow_direct){
        controller_->angle_target.z = controller_->follow_target_ang + 180;
        controller_->angle_target.z = controller_->angle_target.z>180? (controller_->angle_target.z-360):controller_->angle_target.z;
      }else{
        controller_->angle_target.z = controller_->follow_target_ang;
      }
      
      RCLCPP_INFO(controller_->get_logger(), "path track  true");
      controller_->got_follow_target = false;
    }
  }
}

void PilotMission::output(void){
  controller_->output.x = vel_output.x;
  controller_->output.y = vel_output.y;
  controller_->output.z = vel_output.z;

  controller_->output.roll  = rate_output.x;
  controller_->output.pitch = rate_output.y;
  controller_->output.yaw   = rate_output.z;


  // controller_->output.x = 0.0;
  // controller_->output.y = 0.0;
  // controller_->output.z = 0.0;

  // controller_->output.roll  = 0.0;
  // controller_->output.pitch = 0.0;
  // controller_->output.yaw   = 0.0;
};




} //end namespace ControllerNS