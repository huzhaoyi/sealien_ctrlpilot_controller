/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****
* File name          : pid_debug_logger.cpp
* Brief              : Task-gated PID debug CSV logger
*******************************************************************************/

#include "pid_debug_logger.hpp"

#include <cerrno>
#include <cstring>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

#include <sstream>

namespace ControllerNS
{

PidDebugLogger::PidDebugLogger()
    : logger_(rclcpp::get_logger("pid_debug_logger"))
    , enable_(false)
    , recording_(false)
    , write_failed_(false)
    , task_id_(0)
    , script_id_(-1)
    , fp_(nullptr)
{
}

PidDebugLogger::~PidDebugLogger()
{
    stop_recording();
}

void PidDebugLogger::configure(rclcpp::Logger logger, bool enable, const std::string &log_dir)
{
    logger_ = logger;
    enable_ = enable;
    log_dir_ = expand_log_dir(log_dir);
    RCLCPP_INFO(
        logger_,
        "pid_debug_log enable=%d dir=%s",
        enable_ ? 1 : 0,
        log_dir_.c_str());
}

std::string PidDebugLogger::expand_log_dir(const std::string &raw) const
{
    if (raw.empty())
    {
        const char *home = std::getenv("HOME");
        if (home == nullptr)
        {
            return std::string("/tmp/auv_pid_logs");
        }
        return std::string(home) + "/sealien_auv_ws/logs/pid";
    }
    if (raw[0] == '~')
    {
        const char *home = std::getenv("HOME");
        if (home == nullptr)
        {
            return std::string("/tmp/auv_pid_logs");
        }
        return std::string(home) + raw.substr(1);
    }
    return raw;
}

bool PidDebugLogger::ensure_dir()
{
    std::string path;
    path.reserve(log_dir_.size());
    for (size_t i = 0; i < log_dir_.size(); ++i)
    {
        path.push_back(log_dir_[i]);
        if (log_dir_[i] != '/' && i + 1 != log_dir_.size())
        {
            continue;
        }
        if (path.empty() || path == "/")
        {
            continue;
        }
        struct stat st;
        if (stat(path.c_str(), &st) == 0)
        {
            if (!S_ISDIR(st.st_mode))
            {
                RCLCPP_ERROR(logger_, "pid log path exists but is not dir: %s", path.c_str());
                return false;
            }
            continue;
        }
        if (mkdir(path.c_str(), 0755) != 0 && errno != EEXIST)
        {
            RCLCPP_ERROR(
                logger_,
                "mkdir pid log failed path=%s err=%s",
                path.c_str(),
                std::strerror(errno));
            return false;
        }
    }
    return true;
}

const char *PidDebugLogger::script_name_from_id(int script_id)
{
    switch (script_id)
    {
    case 1:
        return "DIVE_HOLD_SURFACE";
    case 2:
        return "EMERGENCY_SURFACE";
    case 3:
        return "DIVE_CRUISE_SURFACE";
    case 4:
        return "PID_TUNE";
    case 5:
        return "PID_TUNE_PITCH";
    case 6:
        return "PID_TUNE_VX";
    case 7:
        return "PID_TUNE_YAW";
    default:
        return "UNKNOWN";
    }
}

int PidDebugLogger::script_id_from_params(
    const sealien_ctrlpilot_msgmanagement::msg::SysTaskCmd &task)
{
    if (task.params.empty())
    {
        return -1;
    }
    const int code = static_cast<int>(task.params[0]);
    if (code >= PID_LOG_SCRIPT_BASE)
    {
        return code - PID_LOG_SCRIPT_BASE;
    }
    return -1;
}

void PidDebugLogger::start_recording(
    const sealien_ctrlpilot_msgmanagement::msg::TaskStatus &msg)
{
    if (!enable_ || recording_ || write_failed_)
    {
        return;
    }
    if (!ensure_dir())
    {
        write_failed_ = true;
        return;
    }

    task_id_ = msg.task.task_id;
    script_id_ = script_id_from_params(msg.task);
    script_name_ = script_name_from_id(script_id_);

    std::ostringstream params_oss;
    for (size_t i = 0; i < msg.task.params.size(); ++i)
    {
        if (i > 0)
        {
            params_oss << ';';
        }
        params_oss << msg.task.params[i];
    }
    params_summary_ = params_oss.str();

    char time_buf[32];
    const std::time_t now = std::time(nullptr);
    std::tm tm_now;
    localtime_r(&now, &tm_now);
    std::strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", &tm_now);

    char name_buf[320];
    std::snprintf(
        name_buf,
        sizeof(name_buf),
        "%s/pid_%s_job%u_s%d_%s.csv",
        log_dir_.c_str(),
        time_buf,
        task_id_,
        script_id_,
        script_name_.c_str());

    fp_ = std::fopen(name_buf, "w");
    if (fp_ == nullptr)
    {
        RCLCPP_ERROR(
            logger_,
            "open pid log failed path=%s err=%s",
            name_buf,
            std::strerror(errno));
        write_failed_ = true;
        return;
    }

    job_id_ = std::to_string(task_id_);

    /* Meta line: readable underwater run identity without opening ROS */
    std::fprintf(
        fp_,
        "# task_id=%u script_id=%d script=%s params=%s\n",
        task_id_,
        script_id_,
        script_name_.c_str(),
        params_summary_.empty() ? "-" : params_summary_.c_str());

    const int n = std::fprintf(
        fp_,
        "t_sec,task_id,script_id,script,job_id,stage,stage_detail,ctrl_mode,lock,depth_m,"
        "velx_sp,velx,velx_err,thrust_out,"
        "pitch_sp,pitch,pitch_err,pitch_rate_sp,pitch_rate,pitch_rate_out,"
        "yaw_rate_sp,yaw_rate,yaw_rate_out,"
        "out_x,out_pitch,out_yaw,gs1,gs2,gs3,gs4,"
        "angle_pitch_kp,angle_pitch_ki,angle_pitch_kd,"
        "rate_pitch_kp,rate_pitch_ki,rate_pitch_kd,"
        "rate_yaw_kp,rate_yaw_ki,rate_yaw_kd,"
        "vel_x_kp,vel_x_ki,vel_x_kd\n");
    if (n < 0)
    {
        RCLCPP_ERROR(logger_, "pid log header write failed");
        std::fclose(fp_);
        fp_ = nullptr;
        write_failed_ = true;
        return;
    }

    recording_ = true;
    RCLCPP_INFO(
        logger_,
        "pid debug CSV start: %s (task=%u script=%d %s)",
        name_buf,
        task_id_,
        script_id_,
        script_name_.c_str());
}

void PidDebugLogger::stop_recording()
{
    if (fp_ != nullptr)
    {
        std::fflush(fp_);
        std::fclose(fp_);
        fp_ = nullptr;
        RCLCPP_INFO(logger_, "pid debug CSV stop job_id=%s", job_id_.c_str());
    }
    recording_ = false;
}

void PidDebugLogger::on_task_status(const sealien_ctrlpilot_msgmanagement::msg::TaskStatus &msg)
{
    if (!enable_)
    {
        return;
    }

    const uint8_t st = msg.status;

    if (st == PID_LOG_STATUS_ENTER || st == PID_LOG_STATUS_ONGOING)
    {
        if (!recording_)
        {
            start_recording(msg);
        }
        return;
    }

    if (st == PID_LOG_STATUS_EXIT ||
        st == PID_LOG_STATUS_FINISH ||
        st == PID_LOG_STATUS_FAIL)
    {
        stop_recording();
        return;
    }
}

void PidDebugLogger::on_task_stage(const sealien_ctrlpilot_msgmanagement::msg::TaskStage &msg)
{
    if (!msg.job_id.empty())
    {
        job_id_ = msg.job_id;
    }
    if (!msg.stage_name.empty())
    {
        stage_name_ = msg.stage_name;
    }
    if (!msg.detail.empty())
    {
        stage_detail_ = msg.detail;
    }
}

void PidDebugLogger::write_sample(const pid_debug_sample_t &sample)
{
    if (!recording_ || fp_ == nullptr || write_failed_)
    {
        return;
    }

    /* Sanitize text fields for CSV (no commas/newlines) */
    auto sanitize = [](const std::string &in) -> std::string
    {
        std::string out = in;
        for (char &c : out)
        {
            if (c == ',' || c == '\n' || c == '\r')
            {
                c = '_';
            }
        }
        return out.empty() ? "-" : out;
    };

    const std::string stage = sanitize(stage_name_);
    const std::string detail = sanitize(stage_detail_);
    const std::string script = sanitize(script_name_);

    const int n = std::fprintf(
        fp_,
        "%.3f,%u,%d,%s,%s,%s,%s,%d,%d,%.4f,"
        "%.4f,%.4f,%.4f,%.4f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f\n",
        sample.t_sec,
        sample.task_id,
        sample.script_id,
        script.c_str(),
        job_id_.c_str(),
        stage.c_str(),
        detail.c_str(),
        sample.ctrl_mode,
        sample.lock_status,
        sample.depth_m,
        sample.velx_sp,
        sample.velx,
        sample.velx_err,
        sample.thrust_out,
        sample.pitch_sp,
        sample.pitch,
        sample.pitch_err,
        sample.pitch_rate_sp,
        sample.pitch_rate,
        sample.pitch_rate_out,
        sample.yaw_rate_sp,
        sample.yaw_rate,
        sample.yaw_rate_out,
        sample.out_x,
        sample.out_pitch,
        sample.out_yaw,
        sample.gs1,
        sample.gs2,
        sample.gs3,
        sample.gs4,
        sample.angle_pitch_kp,
        sample.angle_pitch_ki,
        sample.angle_pitch_kd,
        sample.rate_pitch_kp,
        sample.rate_pitch_ki,
        sample.rate_pitch_kd,
        sample.rate_yaw_kp,
        sample.rate_yaw_ki,
        sample.rate_yaw_kd,
        sample.vel_x_kp,
        sample.vel_x_ki,
        sample.vel_x_kd);

    if (n < 0)
    {
        RCLCPP_ERROR(logger_, "pid log write failed; stop recording");
        write_failed_ = true;
        stop_recording();
    }
}

bool PidDebugLogger::is_recording() const
{
    return recording_;
}

}  // namespace ControllerNS
