/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******header file****
* File name          : pid.hpp
* Author             : Yi Lu
* Brief              : 
********************************************************************************
* modify
* Version   Date                Author              Described
* V1.00     2026/2/11            Yi Lu               Created
*******************************************************************************/

#pragma once

#define DEFAULT_PID_INTEGRATION_LIMIT 500.0 //
#define DEFAULT_PID_OUTPUT_LIMIT 0.0        //


namespace ControllerNS{

class Pid_Object{
public:
  float desired;     //< set point
  float error;       //< error
  float prevError;   //< previous error
  float integ;       //< integral
  float deriv;       //< derivative
  float kp;          //< proportional gain
  float ki;          //< integral gain
  float kd;          //< derivative gain
  float outP;        //< proportional output (debugging)
  float outI;        //< integral output (debugging)
  float outD;        //< derivative output (debugging)
  float iLimit;      //< integral limit
  float outputLimit; //< total PID output limit, absolute value. '0' means no limit.
  float dt;          //< delta-time dt
  float out;         //< out

  Pid_Object(void){
    desired = 0;     
    error = 0;       
    prevError = 0;   
    integ = 0;       
    deriv = 0;       
    kp = 0;          
    ki = 0;          
    kd = 0;          
    outP = 0;        
    outI = 0;        
    outD = 0;        
    iLimit = 0;      
    outputLimit = 0; 
    dt = 0;          
    out = 0;         
  }

  void init(float p, float i, float d, float ilim, float olimt , float dT){
    reset();
    desired = 0;                           // 目标值
    kp = p;                       // kp
    ki = i;                       // ki
    kd = d;                       // kd
    iLimit = ilim; // 积分限幅
    outputLimit = olimt; // 默认pid输出限幅，0为不限幅
    dt = dT;   
  }

  void reset(void){
    error = 0;
    prevError = 0;
    integ = 0;
    deriv = 0;                           
  }

  float pid_update(const float error_in){
    float output; // 输出
    error = error_in; // 误差
    integ += error * dt; // 积分计算
    // 积分限幅
    if (integ > iLimit){ // 若大于，就积分 = 积分限幅
      integ = iLimit;
    }else if (integ < -iLimit){
      integ = -iLimit;
    }
    deriv = (error - prevError) / dt; // 微分计算公式
    outP = kp * error; // kp的输出值 kp * error
    outI = ki * integ; // ki的输出值 ki * integ
    outD = kd * deriv; // kd的输出值 kd * deriv
    output = outP + outI + outD; // 总输出
    // 输出限幅，此处如果设置outputLimit = 0，没有输出限幅，跳过此函数。
    if(outputLimit != 0){
      if (output > outputLimit){
        output = outputLimit;
      }else if (output < -outputLimit){
        output = -outputLimit;
      }    
    }
    prevError = error; // 更新历史误差
    out = output; // 更新输出

    return output; // 返回值
  }
} ;



} //end namespace ControllerNS