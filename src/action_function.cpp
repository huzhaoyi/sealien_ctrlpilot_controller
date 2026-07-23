/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****
* File name          : action_function.cpp
* Author             : Yi Lu
* Brief              : 
********************************************************************************
* modify
* Version   Date                Author              Described
* V1.00     2026/7/23            Yi Lu               Created
*******************************************************************************/

#include "controller_core.hpp"

namespace ControllerNS{

using namespace std::placeholders;

/*+++++++++++++++++++++++++++oil bladder action++++++++++++++++++++++++++++++++++++*/


/********************************************************************************
 * @brief  :处理goal请求
 * @param  :
 * @return :NONE
 *********************************************************************************/
rclcpp_action::GoalResponse Controller::oilBladder_handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const PercentTarget::Goal> goal){
  RCLCPP_INFO(this->get_logger(), "oil_bladder 收到Goal请求,order type[%f]",goal->order);

  rclcpp_action::GoalResponse res;

  // 验证goal参数
  if(fabs(goal->order)<=100){
    res = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }else{
    res = rclcpp_action::GoalResponse::REJECT;
    RCLCPP_WARN(this->get_logger(), "拒绝goal: order必须在0-100内");
  } 
  
  return res;
}

/********************************************************************************
 * @brief  :处理取消请求
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
rclcpp_action::CancelResponse Controller::oilBladder_handle_cancel(const std::shared_ptr<GoalHandlePercentTarget> goal_handle){
  RCLCPP_INFO(this->get_logger(), "oil_bladder 收到取消请求");
  return rclcpp_action::CancelResponse::ACCEPT;
}

/********************************************************************************
 * @brief  :接受goal后执行（在新线程中）
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::oilBladder_handle_accepted(const std::shared_ptr<GoalHandlePercentTarget> goal_handle){
  // 使用单独的线程执行任务，避免阻塞
  std::thread{std::bind(&Controller::oilBladder_execute, this, _1), goal_handle}.detach();
}

/********************************************************************************
 * @brief  :执行具体的action逻辑
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::oilBladder_execute(const std::shared_ptr<GoalHandlePercentTarget> goal_handle){
  RCLCPP_INFO(this->get_logger(), "开始执行oil_bladder action");

  auto feedback = std::make_shared<PercentTarget::Feedback>();
  const auto goal = goal_handle->get_goal();
  auto order = goal->order;
  auto result = std::make_shared<PercentTarget::Result>();  //结果对象

  sealien_ctrlpilot_msgmanagement::msg::PlungerPumpCmd pumpPwm;
  sealien_ctrlpilot_msgmanagement::msg::SwitchCmd valve_cmd;

  if(order < (status.sensor_displace_oilbladder-2)){   //放油，关泵， 开阀
    pumpPwm.duty_pct_ch0 = 0;
    pumpPwm.duty_pct_ch1 = 0;
    pump_cmd_publisher->publish(pumpPwm);

    //这里两个阀同时控
    valve_cmd.index = 0;
    valve_cmd.value = 1;
    switch_cmd_publisher->publish(valve_cmd);

    valve_cmd.index = 1;
    valve_cmd.value = 1;
    switch_cmd_publisher->publish(valve_cmd);

  }else if(order > (status.sensor_displace_oilbladder+2)){ //补油，关阀，开泵
    //这里两个阀同时控
    valve_cmd.index = 0;
    valve_cmd.value = 0;
    switch_cmd_publisher->publish(valve_cmd);

    valve_cmd.index = 1;
    valve_cmd.value = 0;
    switch_cmd_publisher->publish(valve_cmd);

    pumpPwm.duty_pct_ch0 = 50;
    pumpPwm.duty_pct_ch1 = 50;
    pump_cmd_publisher->publish(pumpPwm);
  }else{ //偏差太小，不做处理
    result->result = status.sensor_displace_oilbladder; 
    goal_handle->succeed(result);
    return;
  }

  while(1){
    // 每步检查是否取消
    if (goal_handle->is_canceling()) {
      result->result = status.sensor_displace_oilbladder;   //赋值当前的位置百分比
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "oil_bladder action 被取消");
      break;
    }

    if(fabs(order - status.sensor_displace_oilbladder)<2){  //精度±2%
      result->result = status.sensor_displace_oilbladder; // 设置最终结果
      goal_handle->succeed(result);
      // goal_handle->abort(result);
      RCLCPP_INFO(this->get_logger(), "oil_bladder action 执行完成");
      break;
    }else{
      feedback->percent = status.sensor_displace_oilbladder;
      goal_handle->publish_feedback(feedback);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200)); //延时，控制循环频率50HZ
  }  
}

/*+++++++++++++++++++++++++++pitch motor action++++++++++++++++++++++++++++++++++++*/

/********************************************************************************
 * @brief  :处理goal请求
 * @param  :
 * @return :NONE
 *********************************************************************************/
rclcpp_action::GoalResponse Controller::pitchMotor_handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const PercentTarget::Goal> goal){
  RCLCPP_INFO(this->get_logger(), "pitch Motor 收到Goal请求,order type[%f]",goal->order);

  rclcpp_action::GoalResponse res;

  // 验证goal参数
  if(fabs(goal->order)<=100){
    res = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }else{
    res = rclcpp_action::GoalResponse::REJECT;
    RCLCPP_WARN(this->get_logger(), "拒绝goal: order必须在0-100内");
  } 
  
  return res;
}

/********************************************************************************
 * @brief  :处理取消请求
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
rclcpp_action::CancelResponse Controller::pitchMotor_handle_cancel(const std::shared_ptr<GoalHandlePercentTarget> goal_handle){
  RCLCPP_INFO(this->get_logger(), "pitch Motor 收到取消请求");
  return rclcpp_action::CancelResponse::ACCEPT;
}

/********************************************************************************
 * @brief  :接受goal后执行（在新线程中）
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::pitchMotor_handle_accepted(const std::shared_ptr<GoalHandlePercentTarget> goal_handle){
  // 使用单独的线程执行任务，避免阻塞
  std::thread{std::bind(&Controller::pitchMotor_execute, this, _1), goal_handle}.detach();
}

/********************************************************************************
 * @brief  :执行具体的action逻辑
 * @param  :NONE
 * @return :NONE
 *********************************************************************************/
void Controller::pitchMotor_execute(const std::shared_ptr<GoalHandlePercentTarget> goal_handle){
  RCLCPP_INFO(this->get_logger(), "开始执行 pitch Motor action");

  auto feedback = std::make_shared<PercentTarget::Feedback>();
  const auto goal = goal_handle->get_goal();
  auto order = goal->order;
  auto result = std::make_shared<PercentTarget::Result>();  //结果对象

  sealien_ctrlpilot_msgmanagement::msg::PitchMotorCmd pitch_cmd;

  if(order < (status.sensor_displace_pitchmotor-2)){   //正转
    pitch_cmd.run_cmd = 2;
    pitch_cmd.speed_rpm = 500;

    pitch_cmd_publisher->publish(pitch_cmd);

  }else if(order > (status.sensor_displace_pitchmotor+2)){ //反转
    pitch_cmd.run_cmd = 3;
    pitch_cmd.speed_rpm = 500;

    pitch_cmd_publisher->publish(pitch_cmd);
  }else{ //偏差太小，停止电机
    pitch_cmd.run_cmd = 0;
    pitch_cmd.speed_rpm = 0;

    pitch_cmd_publisher->publish(pitch_cmd);
    goal_handle->succeed(result);
    return;
  }

  while(1){
    // 每步检查是否取消
    if (goal_handle->is_canceling()) {
      result->result = status.sensor_displace_pitchmotor;   //赋值当前的位置百分比
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "pitch Motor action 被取消");
      break;
    }

    if(fabs(order - status.sensor_displace_pitchmotor)<2){  //精度±2%
      result->result = status.sensor_displace_pitchmotor; // 设置最终结果
      goal_handle->succeed(result);
      // goal_handle->abort(result);
      RCLCPP_INFO(this->get_logger(), "pitch Motor action 执行完成");
      break;
    }else{
      feedback->percent = status.sensor_displace_pitchmotor;
      goal_handle->publish_feedback(feedback);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200)); //延时，控制循环频率50HZ
  }  
}

} //end namespace ControllerNS

