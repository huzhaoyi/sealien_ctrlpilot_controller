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
 controller_->position_controller_update(vel_output, controller_->status.pos.z);
};
void PilotStablize1::reset(void){
  controller_->attitude_controller_reset();
  controller_->position_controller_reset(controller_->status.pos.z);
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
  controller_->position_controller_update(vel_output, controller_->status.alt);
};

void PilotStablize2::reset(void){
  controller_->attitude_controller_reset();
  controller_->position_controller_reset(controller_->status.alt);
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
  controller_->position_controller_update(vel_output, controller_->status.pos.z);
};

void PilotAutodepth::reset(void){
  controller_->position_controller_reset(controller_->status.pos.z);
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
  controller_->position_controller_update(vel_output, controller_->status.pos.z);
};

void PilotAutoheight::reset(void){
  controller_->position_controller_reset(controller_->status.alt);
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
  controller_->position_controller_update(vel_output, controller_->status.pos.z);
}

void PilotAutoHold1::reset(void){
  controller_->attitude_controller_reset();
  controller_->position_controller_reset(controller_->status.pos.z);
};

void PilotAutoHold1::output(void){
  controller_->output.x = vel_output.x;
  controller_->output.y = vel_output.y;
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
  controller_->position_controller_update(vel_output, controller_->status.alt);
}

void PilotAutoHold2::reset(void){
  controller_->attitude_controller_reset();
  controller_->position_controller_reset(controller_->status.alt);
};

void PilotAutoHold2::output(void){
  controller_->output.x = vel_output.x;
  controller_->output.y = vel_output.y;
  controller_->output.z = vel_output.z;

  controller_->output.roll = rate_output.x;
  controller_->output.pitch = rate_output.y;
  controller_->output.yaw = rate_output.z;
};

/****************路径跟踪1，定深*******************/
void PilotMission1::setpoint_mapping(void){
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

void PilotMission1::update(void){
  if(controller_->status.track_status){//跟踪进行
    controller_->attitude_rate_pid(rate_output, controller_->rate_target, controller_->status.rate);

    controller_->position_velocity_pid(vel_output, controller_->vel_target, controller_->status.vel);
  
    vel_output.z = controller_->config.thrust_base + vel_output.z;
    // RCLCPP_INFO(controller_->get_logger(), "path track");
  }else{//位置保持
    controller_->attitude_controller_update(rate_output);

    controller_->position_controller_update(vel_output, controller_->status.pos.z);


    if(controller_->got_task_target){  //任务在进行中，需要判断任务是否完成
      if(controller_->isTaskFinish()){
        controller_->TaskFinishPub();
        controller_->got_task_target = false;
      }
    }

    // RCLCPP_INFO(controller_->get_logger(), "pos hold");
  }
}

void PilotMission1::reset(void){
  controller_->attitude_controller_reset();

  controller_->position_controller_reset(controller_->status.pos.z);

  if(controller_->status.track_status == false){  //从跟踪状态到非跟踪状态
    RCLCPP_INFO(controller_->get_logger(), "M1 path controller_->got_follow_target[%d]",controller_->got_follow_target);
    if(controller_->got_follow_target == false){  //之前没有有效路径，那进入定位模式的目标点等于进入时的位置角度
      controller_->angle_target.x = 0.0;
      controller_->angle_target.y = 0.0;
      controller_->angle_target.z = controller_->status.yaw_base;
      controller_->pos_target.x = controller_->x_target_base;
      controller_->pos_target.y = controller_->y_target_base;
      RCLCPP_INFO(controller_->get_logger(), "path track  false");
    }else{//如果之前有有效路径，说明跟踪完成，则目标点是路径最后一点
      controller_->pos_target.x = controller_->follow_target_pos.x;
      controller_->pos_target.y = controller_->follow_target_pos.y;

      controller_->angle_target.x = 0.0;
      controller_->angle_target.y = 0.0;
      
      if(controller_->follow_direct){  //倒退时，目标角度与实际角度相差180°
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

void PilotMission1::output(void){
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

/****************路径跟踪2，定高*******************/
void PilotMission2::setpoint_mapping(void){
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

void PilotMission2::update(void){
  if(controller_->status.track_status){//跟踪进行
    controller_->attitude_rate_pid(rate_output, controller_->rate_target, controller_->status.rate);

    controller_->position_velocity_pid(vel_output, controller_->vel_target, controller_->status.vel);
  
    vel_output.z = controller_->config.thrust_base + vel_output.z;
    // RCLCPP_INFO(controller_->get_logger(), "path track");
  }else{//位置保持
    controller_->attitude_controller_update(rate_output);

    controller_->position_controller_update(vel_output, controller_->status.alt);


    if(controller_->got_task_target){  //任务在进行中，需要判断任务是否完成
      if(controller_->isTaskFinish()){
        controller_->TaskFinishPub();
        controller_->got_task_target = false;
      }
    }

    // RCLCPP_INFO(controller_->get_logger(), "pos hold");
  }
}

void PilotMission2::reset(void){
  controller_->attitude_controller_reset();

  controller_->position_controller_reset(controller_->status.alt);

  // RCLCPP_INFO(controller_->get_logger(), "alt[%f], target[%f]",controller_->status.alt, controller_->pos_target.z);

  if(controller_->status.track_status == false){  //从跟踪状态到非跟踪状态
    RCLCPP_INFO(controller_->get_logger(), "M2 path controller_->got_follow_target[%d]",controller_->got_follow_target);
    if(controller_->got_follow_target == false){  //之前没有有效路径，那进入定位模式的目标点等于进入时的位置角度
      controller_->angle_target.x = 0.0;
      controller_->angle_target.y = 0.0;
      controller_->angle_target.z = controller_->status.yaw_base;
      controller_->pos_target.x = controller_->x_target_base;
      controller_->pos_target.y = controller_->y_target_base;
      RCLCPP_INFO(controller_->get_logger(), "path track  false");
    }else{//如果之前有有效路径，说明跟踪完成，则目标点是路径最后一点
      controller_->pos_target.x = controller_->follow_target_pos.x;
      controller_->pos_target.y = controller_->follow_target_pos.y;

      controller_->angle_target.x = 0.0;
      controller_->angle_target.y = 0.0;
      
      if(controller_->follow_direct){  //倒退时，目标角度与实际角度相差180°
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

void PilotMission2::output(void){
  controller_->output.x = vel_output.x;
  controller_->output.y = vel_output.y;
  controller_->output.z = vel_output.z;

  controller_->output.roll  = rate_output.x;
  controller_->output.pitch = rate_output.y;
  controller_->output.yaw   = rate_output.z;

};

} //end namespace ControllerNS