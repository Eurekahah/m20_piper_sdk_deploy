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
 */

#pragma once

#include "robot_interface.h"
#include "dds_types.h"

#include "drdds/msg/joints_data.hpp"
#include "drdds/msg/joints_data_cmd.hpp"

using namespace types;

class PiperArmInterface : public RobotInterface {
private:
    VecXf joint_pos_, joint_vel_, joint_tau_;

    rclcpp::Publisher<drdds::msg::JointsDataCmd>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<drdds::msg::JointsData>::SharedPtr arm_data_sub_;

public:
    static constexpr int kArmDof = 8;  // arm_joint1..6 + gripper_joint1..2

    PiperArmInterface(const std::string &robot_name, rclcpp::Node::SharedPtr node)
        : RobotInterface(robot_name, kArmDof, node) {
        joint_pos_ = VecXf::Zero(kArmDof);
        joint_vel_ = VecXf::Zero(kArmDof);
        joint_tau_ = VecXf::Zero(kArmDof);

        arm_cmd_pub_ = node_->create_publisher<drdds::msg::JointsDataCmd>("/ARM_JOINTS_CMD", 10);
        arm_data_sub_ = node_->create_subscription<drdds::msg::JointsData>(
            "/ARM_JOINTS_DATA", 10,
            std::bind(&PiperArmInterface::Handler, this, std::placeholders::_1));
    }

    ~PiperArmInterface() override = default;

    void Start() override {
        // Hold the arm at the default pose (matches the Isaac Lab default
        // joints: arm2 = 0.5, arm3 = -0.5, gripper closed).
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

    void Handler(const drdds::msg::JointsData::SharedPtr msg) {
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
