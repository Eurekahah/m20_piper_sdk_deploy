
#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <vector>
#include <deque>
#include <map>
#include <cmath>
#include <memory>
#include <thread>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <cstdlib>

// ---------------------------------------------------------------------------
// 机身高度指令的默认值 / 允许上限
//   默认 0.513 m = 训练课程 s0 的锚点（DEF-009）；上限 0.55 m = 训练 `body_pose.height` 终值区间
//   （`WBCCommandsCfg.body_pose.height_range = (0.33, 0.55)`）。
//   环境变量（**手动手动标定用**，见 DEF-059 / TODO P1-9）：
//     M20_BODY_HEIGHT_DEFAULT=<m>  RL 站立的高度指令默认值（键盘 'l' 复位也用它）
//     M20_BODY_HEIGHT_MAX=<m>      键盘 H/J 的上限（>0.55 会超出训练区间，只用于扫范围）
//   用途：把"RL 站立位姿"顶到与"站立状态机结束位姿"一致（否则按 c 进 RL 时机器人会
//         自己挪一下 —— 入口瞬态）。0.513 是按**训练默认站姿**标定的，站立状态机走厂商
//         两段式（0.48 m）时两者不同，需要按实测调。
// ---------------------------------------------------------------------------
inline float DefaultBodyHeight() {
    static const float v = []() {
        const char *e = std::getenv("M20_BODY_HEIGHT_DEFAULT");
        return e ? static_cast<float>(std::atof(e)) : 0.513f;
    }();
    return v;
}

inline float MaxBodyHeight() {
    static const float v = []() {
        const char *e = std::getenv("M20_BODY_HEIGHT_MAX");
        return e ? static_cast<float>(std::atof(e)) : 0.55f;
    }();
    return v;
}
#include <unistd.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <stdio.h>
#include <csignal>
#include <pthread.h>


namespace types {
using Vec3f = Eigen::Vector3f;
using Vec3d = Eigen::Vector3d;
using Vec4f = Eigen::Vector4f;
using Vec4d = Eigen::Vector4d;
using VecXf = Eigen::VectorXf;
using VecXd = Eigen::VectorXd;

using Mat3f = Eigen::Matrix3f;
using Mat3d = Eigen::Matrix3d;
using MatXf = Eigen::MatrixXf;
using MatXd = Eigen::MatrixXd;

const float gravity = 9.815;

struct RobotBasicState {
    Vec3f base_rpy;
    Vec4f base_quat;
    Mat3f base_rot_mat;
    Vec3f base_omega;
    Vec3f base_acc;
    MatXf flt_base_acc_mat;
    VecXf joint_pos;
    VecXf joint_vel;
    VecXf joint_tau;

    RobotBasicState(int dof = 24) {
        base_rpy.setZero();
        base_quat.setZero();
        base_rot_mat.setIdentity();
        base_omega.setZero();
        base_acc.setZero();
        flt_base_acc_mat = Eigen::MatrixXf::Zero(20, 3);
        joint_pos = VecXf::Zero(dof);
        joint_vel = VecXf::Zero(dof);
        joint_tau = VecXf::Zero(dof);
    }
};

struct RobotAction {
    VecXf goal_joint_pos;
    VecXf goal_joint_vel;
    VecXf kp;
    VecXf kd;
    VecXf tau_ff;

    MatXf ConvertToMat() {
        MatXf res(goal_joint_pos.rows(), 5);
        res.col(0) = kp;
        res.col(1) = goal_joint_pos;
        res.col(2) = kd;
        res.col(3) = goal_joint_vel;
        res.col(4) = tau_ff;
        return res;
    }
};


struct UserCommand {
    double time_stamp;
        uint8_t safe_control_mode; //0 normal, 1 stand, 2 sit, 3 joint damping
    uint8_t target_mode;
    float forward_vel_scale = 0;
    float side_vel_scale = 0;
    float turnning_vel_scale = 0;
    float reserved_scale;

    // ---- body pose targets (absolute, accumulated by the keyboard) ----
    // NOTE: this struct is memset() to zero by the command interfaces, so the
    // keyboard/gamepad must re-initialise these after the memset.
    float body_height = DefaultBodyHeight();   // base height target (m)
    float body_pitch  = 0.0f;     // base pitch target (rad)
    float body_roll   = 0.0f;     // base roll target (rad)

    // ---- arm teleop (raw increments; arm_controller accumulates them) ----
    float ee_inc[6] = {0, 0, 0, 0, 0, 0};   // dx, dy, dz, droll, dpitch, dyaw
    float gripper_cmd = -1.0f;              // 0 open, 1 close, -1 no change
    uint8_t ee_reset = 0;                   // 1 = reset EE target to current pose

    // ---- filled by RLControlState from arm_controller feedback (obs) ----
    // `ee_goal` 是 **root（机体）坐标系**下的 EE 目标位姿 ← 策略观测要求的口径。
    // 默认 = Piper 默认关节姿态（arm2 0.5 / arm3 -0.5）下 `gripper_base` 在 root 系的位姿
    // （实测 (0.3492, 0, 0.4326)，见 scripts/check_mjcf_contract.py）。
    // ⚠️ 别写成臂基座坐标系的值 (0.1092, 0, 0.3439) —— 差 24 cm（DEF-019）。
    float ee_goal_pos[3]  = {0.3492f, 0.0f, 0.4327f};  // x, y, z (root frame)
    float ee_goal_quat[4] = {0.7373f, 0.0f, 0.6756f, 0.0f};  // w, x, y, z
};

enum FromType {
    inside = 0,
    user,
};
struct NetInfo {
    std::string ip;
    int port;
    FromType fromType;
};

struct DataInfo : public NetInfo {
    const char* buffer;
    ssize_t len;
};
struct CmdInfo : public NetInfo {
    int type;
    int command;
};
};  // namespace types
