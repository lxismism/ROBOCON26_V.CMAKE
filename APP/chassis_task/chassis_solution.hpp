/**
 * @file chassis_solution.hpp
 * @author 大帅将军
 * @brief 底盘解算方案，X型全向轮运动学解算和PID控制
 * @version 0.1
 * @date 2026-04-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention : X型全向轮底盘解算
 * @note :
 * @versioninfo :
 */
#pragma once

#include "Motor.hpp"
#include "pid_controller.h"
#include "topic_pool.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

extern float_t robot_position_MF[6][5][4];

class OmniChassis {
public:
    enum WheelIndex : size_t {
        kLeftUp = 0,
        kRightUp = 1,
        kLeftDown = 2,
        kRightDown = 3,
        kWheelCount = 4,
    };

    struct Geometry {
        float wheel_diameter_m; // 轮径
        float track_width_m;    // 轮距
        float wheel_base_m;     // 轴距

        Geometry(float wheel_diameter = 0.16f, float track_width = 1.025f,
                 float wheel_base = 1.025f)
            : wheel_diameter_m(wheel_diameter), track_width_m(track_width),
              wheel_base_m(wheel_base) {}
    };

    struct SpeedPidParam {
        float kp;
        float ki;
        float kd;
        float max_out;
        float deadband;
        uint16_t improve;

        SpeedPidParam(float kp_in = 100.0f, float ki_in = 80.0f,
                      float kd_in = 0.0f, float max_out_in = 20000.0f,
                      float deadband_in = 0.3f, uint16_t improve_in = NONE)
            : kp(kp_in), ki(ki_in), kd(kd_in), max_out(max_out_in),
              deadband(deadband_in), improve(improve_in) {}
    };

    // 轮序固定为 [左上, 右上, 左下, 右下]
    OmniChassis(C620Motor &motor_lu, C620Motor &motor_ru,
                C620Motor &motor_ld, C620Motor &motor_rd,
                const std::array<SpeedPidParam, kWheelCount> &pid_params,
                const Geometry &geometry = Geometry())
        : motors_{&motor_lu, &motor_ru, &motor_ld, &motor_rd},
          geometry_(geometry) {
        setWheelDirectionSign({-1.0f, -1.0f, -1.0f, -1.0f});
        configureSpeedPid(pid_params);
    }

    OmniChassis(C620Motor &motor_lu, C620Motor &motor_ru,
                C620Motor &motor_ld, C620Motor &motor_rd,
                const Geometry &geometry = Geometry(),
                const SpeedPidParam &pid_param = SpeedPidParam())
        : motors_{&motor_lu, &motor_ru, &motor_ld, &motor_rd},
          geometry_(geometry) {
        setWheelDirectionSign({-1.0f, -1.0f, -1.0f, -1.0f});
        configureSpeedPid(pid_param);
    }

    void configureGeometry(const Geometry &geometry) { geometry_ = geometry; }

    void configureSpeedPid(const SpeedPidParam &param) {
        for (PID_t &pid : speed_pid_) {
            pid = {};
            pid.Kp = param.kp;
            pid.Ki = param.ki;
            pid.Kd = param.kd;
            pid.MaxOut = param.max_out;
            pid.DeadBand = param.deadband;
            pid.Improve = param.improve;
            PID_Init(&pid);
        }
    }

    void configureSpeedPid(
        const std::array<SpeedPidParam, kWheelCount> &params) {
        for (size_t i = 0; i < speed_pid_.size(); ++i) {
            applyPidParam(speed_pid_[i], params[i]);
        }
    }

    void configureAnglePid(
        const std::array<SpeedPidParam, kWheelCount> &params) {
        for (size_t i = 0; i < angle_pid_.size(); ++i) {
            applyPidParam(angle_pid_[i], params[i]);
        }
    }

    void configureSingleWheelSpeedPid(WheelIndex wheel,
                                      const SpeedPidParam &param) {
        applyPidParam(speed_pid_[wheel], param);
    }

    void setWheelDirectionSign(const std::array<float, 4> &direction_sign) {
        direction_sign_ = direction_sign;
    }

    // 运动学逆解：输入底盘速度命令，输出每个轮子的目标角速度(rad/s)
    std::array<float, 4> solveWheelRadPerSec(
        const pub_chassis_cmd &cmd) const {
        const float vx = cmd.linear_x_;
        const float vy = cmd.linear_y_;
        const float wz = cmd.omega_;

        const float rotation_term = geometry_.track_width_m * 0.5f * wz;

        constexpr float kInvSqrt2 = 0.70710678f; // 1/√2
        const float v_lu = (-vx - vy) * kInvSqrt2 + rotation_term;
        const float v_ru = (-vx + vy) * kInvSqrt2 + rotation_term;
        const float v_ld = ( vx - vy) * kInvSqrt2 + rotation_term;
        const float v_rd = ( vx + vy) * kInvSqrt2 + rotation_term;

        const float mps_to_rad_s = 2.0f / geometry_.wheel_diameter_m;
        return {v_lu * mps_to_rad_s, v_ru * mps_to_rad_s,
                v_ld * mps_to_rad_s, v_rd * mps_to_rad_s};
    }

    // 输入底盘速度命令，完成 解算→PID→setMotorCmd
    void run(const pub_chassis_cmd &cmd) {
        target_rad_s_ = solveWheelRadPerSec(cmd);
        for (size_t i = 0; i < motors_.size(); ++i) {
            target_rad_s_[i] *= direction_sign_[i];
            pid_output_[i] = PID_Calculate(
                &speed_pid_[i],
                motors_[i]->getCurrentSpeed(), // 输出轴角速度(rad/s)
                target_rad_s_[i]);
            motors_[i]->setMotorCmd(pid_output_[i]*1.10f);
        }
    }

    const std::array<float, 4> &targetRadPerSec() const {
        return target_rad_s_;
    }
    const std::array<float, 4> &pidOutput() const { return pid_output_; }
    const PID_t &pid(WheelIndex wheel) const { return speed_pid_[wheel]; }

private:
    static constexpr float kPi = 3.14159265358979323846f;

    static void applyPidParam(PID_t &pid, const SpeedPidParam &param) {
        pid = {};
        pid.Kp = param.kp;
        pid.Ki = param.ki;
        pid.Kd = param.kd;
        pid.MaxOut = param.max_out;
        pid.DeadBand = param.deadband;
        pid.Improve = param.improve;
        PID_Init(&pid);
    }

    std::array<C620Motor *, 4> motors_{};
    Geometry geometry_{};
    std::array<float, kWheelCount> direction_sign_{};
    std::array<PID_t, kWheelCount> speed_pid_{};
    std::array<PID_t, kWheelCount> angle_pid_{};
    std::array<float, kWheelCount> target_rad_s_{};
    std::array<float, kWheelCount> pid_output_{};
};
