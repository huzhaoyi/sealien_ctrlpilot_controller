#ifndef _PID_H_
#define _PID_H_

#define DEFAULT_PID_INTEGRATION_LIMIT 500.0 //
#define DEFAULT_PID_OUTPUT_LIMIT 0.0        //

// class Pid_Init_t{
// public:
//     float kp;
//     float ki;
//     float kd;
//     float iLimit;
//     float outputLimit;

//     pid_init_t(float p, float i, float d, float ilim, float olimt){
//         kp = p;
//         ki = i;
//         kd = d;
//         iLimit = ilim;
//         outputLimit = olimt;  
//     }
// };

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


} ;

#endif /* _PID_H_ */
