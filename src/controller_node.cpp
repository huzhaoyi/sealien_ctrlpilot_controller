/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****
* File name          : attitude_node.cpp
* Author             : Yi Lu
* Brief              : 
********************************************************************************
* modify
* Version   Date                Author              Described
* V1.00     2025/11/27           Yi Lu               Created
*******************************************************************************/

#include "controller_core.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNS::Controller>("controller"));
  rclcpp::shutdown();
  return 0;
}


