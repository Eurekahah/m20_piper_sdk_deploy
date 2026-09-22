/**
 * @file piper_arm_interface.hpp
 * @brief ROS2 interface for the AgileX Piper arm (6 joints + 2 gripper)
 * @author M20+Piper deploy
 * @date 2026-08-20
 *
 * The arm is controlled independently of the M20 legs:
 *   - subscribes /ARM_JOINTS_DATA (drdds/msg/JointsData, 8 entries)
 *   - publishes   /ARM_JOINTS_CMD (drdds/msg/JointsDataCmd, 8 entries)
 *
 * All arm/gripper values are raw rad (identity calibration). The real arm
 * (pyAgxArm over CAN) is driven by the separate arm_controller node; this
 * interface only carries the sim-side arm feedback into the policy obs.
 *
 * 臂还没装/没上电时（`/ARM_JOINTS_DATA` 一直没有发布者）：
 *   - 本接口把 8 个关节**初始化为训练默认姿态**（arm2 0.5 / arm3 -0.5），不是 0 ——
 *     这样第一帧观测与 `ee_goal` 的默认值自洽（否则策略会被告知"臂在非默认姿态"，
 *     见 DEF-023）；
 *   - `HasData()` 为 false，`RLControlState::OnEnter` 会打印一条明确的警告，
 *     提醒"无臂部署请起 arm_stub_node.py，否则策略看到的臂状态是假的"。
 */

#pragma once

#include "robot_interface.h"

#include <chrono>

#include <atomic>
#include "dds_types.h"

#include "drdds/msg/joints_data.hpp"
#include "drdds/msg/joints_data_cmd.hpp"

using namespace types;

class PiperArmInterface : public RobotInterface {
private:
    VecXf joint_pos_, joint_vel_, joint_tau_;
    bool received_ = false;   // 有没有真的收到过 /ARM_JOINTS_DATA

    rclcpp::Publisher<drdds::msg::JointsDataCmd>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<drdds::msg::JointsData>::SharedPtr arm_data_sub_;
    // 也监听 /ARM_JOINTS_CMD：如果最近有别人在发（例如 arm_controller 正在做 IK），
    // 说明臂有"主人"，rl_deploy 就不要在进 RL 时抢着发一次"回默认位姿"。
    rclcpp::Subscription<drdds::msg::JointsDataCmd>::SharedPtr arm_cmd_sniff_sub_;
    std::atomic<double> last_arm_cmd_time_{-1e9};
    std::chrono::steady_clock::time_point t0_{std::chrono::steady_clock::now()};

public:
    static constexpr int kArmDof = 8;  // arm_joint1..6 + gripper_joint1..2

    PiperArmInterface(const std::string &robot_name, rclcpp::Node::SharedPtr node)
        : RobotInterface(robot_name, kArmDof, node) {
        // 训练默认姿态（arm_joint1..6 + gripper1/2）：与 Isaac init_state 一致
        joint_pos_ = VecXf::Zero(kArmDof);
        joint_pos_ << 0.0f, 0.5f, -0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
        joint_vel_ = VecXf::Zero(kArmDof);
        joint_tau_ = VecXf::Zero(kArmDof);

        arm_cmd_pub_ = node_->create_publisher<drdds::msg::JointsDataCmd>("/ARM_JOINTS_CMD", 10);
        arm_data_sub_ = node_->create_subscription<drdds::msg::JointsData>(
            "/ARM_JOINTS_DATA", 10,
            std::bind(&PiperArmInterface::Handler, this, std::placeholders::_1));
        arm_cmd_sniff_sub_ = node_->create_subscription<drdds::msg::JointsDataCmd>(
            "/ARM_JOINTS_CMD", 50,
            [this](const drdds::msg::JointsDataCmd::SharedPtr) {
                last_arm_cmd_time_.store(
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_)
                        .count(), std::memory_order_relaxed);
            });
    }

    ~PiperArmInterface() override = default;

    void Start() override {
        // Hold the arm at the default pose (matches the Isaac Lab default
        // joints: arm2 = 0.5, arm3 = -0.5, gripper closed).
        //
        // 但如果最近 0.5 s 内有人在发 /ARM_JOINTS_CMD（典型的：arm_controller 正在
        // 用 IK 控制臂），说明臂已经有主人：此时再发一次"回默认位姿"会把臂拽一下
        // （一个控制周期后又被 IK 抢回来），所以直接让位。
        const double now_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0_).count();
        const double last = last_arm_cmd_time_.load(std::memory_order_relaxed);
        if (now_s - last < 0.5) {
            std::cout << "[PiperArmInterface] /ARM_JOINTS_CMD 上已有其它发布者"
                         "（如 arm_controller），跳过进 RL 时的默认位姿保持命令"
                      << std::endl;
            return;
        }
        ResetToDefault();
    }

    void Stop() override {}

    /**
     * Set arm joint commands (raw rad).
     * Input matrix layout: [8 x 5] -> [kp, pos, kd, vel, torque_ff]
     */
    void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) override {
        auto msg = drdds::msg::JointsDataCmd();
        for (int i = 0; i < kArmDof; ++i) {
            msg.data.joints_data[i].position = input(i, 1);
            msg.data.joints_data[i].velocity = input(i, 3);
            msg.data.joints_data[i].torque = input(i, 4);
            msg.data.joints_data[i].kp = input(i, 0);
            msg.data.joints_data[i].kd = input(i, 2);
            msg.data.joints_data[i].control_word = dds::kIndexMotorControl;
        }
        arm_cmd_pub_->publish(msg);
    }

    void ResetToDefault() {
        VecXf kp(kArmDof), kd(kArmDof), pos(kArmDof);
        // arm: 300/20, gripper: 4000/200 (from the Isaac Lab DelayedPDActuatorCfg)
        kp << 300, 300, 300, 300, 300, 300, 4000, 4000;
        kd << 20, 20, 20, 20, 20, 20, 200, 200;
        pos << 0.0, 0.5, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0;

        MatXf cmd = MatXf::Zero(kArmDof, 5);
        cmd.col(0) = kp;
        cmd.col(1) = pos;
        cmd.col(2) = kd;
        SetJointCommand(cmd);
    }

    /// 有没有收到过真实的臂反馈（无臂部署时为 false）
    bool HasData() const { return received_; }

    void Handler(const drdds::msg::JointsData::SharedPtr msg) {
        received_ = true;
        for (int i = 0; i < kArmDof; ++i) {
            joint_pos_(i) = msg->data.joints_data[i].position;
            joint_vel_(i) = msg->data.joints_data[i].velocity;
            joint_tau_(i) = msg->data.joints_data[i].torque;
        }
    }

    double GetInterfaceTimeStamp() override { return 0.; }
    VecXf GetJointPosition() override { return joint_pos_; }
    VecXf GetJointVelocity() override { return joint_vel_; }
    VecXf GetJointTorque() override { return joint_tau_; }
    Vec3f GetImuRpy() override { return Vec3f::Zero(); }
    Vec3f GetImuAcc() override { return Vec3f::Zero(); }
    Vec3f GetImuOmega() override { return Vec3f::Zero(); }
    VecXf GetContactForce() override { return VecXf::Zero(4); }
};
