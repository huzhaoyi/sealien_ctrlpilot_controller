/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******header file****
* File name          : pid_debug_logger.hpp
* Brief              : Task-gated PID debug CSV logger
*******************************************************************************/

#pragma once

#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/task_stage.hpp"
#include "sealien_ctrlpilot_msgmanagement/msg/task_status.hpp"

namespace ControllerNS
{

/* TaskStatus.status codes (keep in sync with taskmanagement opcodes) */
enum
{
    PID_LOG_STATUS_READY = 0,
    PID_LOG_STATUS_PLAN = 1,
    PID_LOG_STATUS_ENTER = 2,
    PID_LOG_STATUS_ONGOING = 3,
    PID_LOG_STATUS_EXIT = 4,
    PID_LOG_STATUS_FINISH = 5,
    PID_LOG_STATUS_PAUSE = 6,
    PID_LOG_STATUS_FAIL = 7
};

/* SysTaskCmd script_code = SCRIPT_BASE + script_id */
enum
{
    PID_LOG_SCRIPT_BASE = 1000
};

struct pid_debug_sample_t
{
    double t_sec;
    uint32_t task_id;
    int script_id;
    int ctrl_mode;
    int lock_status;
    float depth_m;
    float velx_sp;
    float velx;
    float velx_err;
    float thrust_out;
    float pitch_sp;
    float pitch;
    float pitch_err;
    float pitch_rate_sp;
    float pitch_rate;
    float pitch_rate_out;
    float yaw_rate_sp;
    float yaw_rate;
    float yaw_rate_out;
    float out_x;
    float out_pitch;
    float out_yaw;
    float gs1;
    float gs2;
    float gs3;
    float gs4;
    /* live gains (may change mid-run via param) */
    float angle_pitch_kp;
    float angle_pitch_ki;
    float angle_pitch_kd;
    float rate_pitch_kp;
    float rate_pitch_ki;
    float rate_pitch_kd;
    float rate_yaw_kp;
    float rate_yaw_ki;
    float rate_yaw_kd;
    float vel_x_kp;
    float vel_x_ki;
    float vel_x_kd;
};

class PidDebugLogger
{
public:
    PidDebugLogger();
    ~PidDebugLogger();

    void configure(rclcpp::Logger logger, bool enable, const std::string &log_dir);
    void on_task_status(const sealien_ctrlpilot_msgmanagement::msg::TaskStatus &msg);
    void on_task_stage(const sealien_ctrlpilot_msgmanagement::msg::TaskStage &msg);
    void write_sample(const pid_debug_sample_t &sample);
    bool is_recording() const;
    uint32_t task_id() const
    {
        return task_id_;
    }
    int script_id() const
    {
        return script_id_;
    }

private:
    void start_recording(const sealien_ctrlpilot_msgmanagement::msg::TaskStatus &msg);
    void stop_recording();
    bool ensure_dir();
    std::string expand_log_dir(const std::string &raw) const;
    static const char *script_name_from_id(int script_id);
    static int script_id_from_params(const sealien_ctrlpilot_msgmanagement::msg::SysTaskCmd &task);

    rclcpp::Logger logger_;
    bool enable_;
    bool recording_;
    bool write_failed_;
    std::string log_dir_;
    std::string job_id_;
    std::string stage_name_;
    std::string stage_detail_;
    std::string script_name_;
    std::string params_summary_;
    uint32_t task_id_;
    int script_id_;
    FILE *fp_;
};

}  // namespace ControllerNS
