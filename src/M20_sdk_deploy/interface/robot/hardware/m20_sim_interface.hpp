/**
 * @file m20_sim_interface.hpp
 * @brief M20 leg interface for sim2sim with identity joint calibration
 * @author M20+Piper deploy
 * @date 2026-08-20
 *
 * For sim2sim the M20_Piper MJCF is generated from the same URDF that Isaac
 * Lab trains on, so the raw MJCF joint frame IS the policy frame. The real
 * M20Interface applies hardware dir/offset calibration and adjusts offsets by
 * +-360 deg for multi-turn joints during Start(), which corrupts the frame if
 * the robot drifts during startup. This interface uses identity calibration
 * and the base (wide-threshold) reset, keeping the whole chain exact.
 */

#pragma once

#include "dds_interface.hpp"

class M20SimInterface : public DdsInterface {
public:
    M20SimInterface(const std::string &robot_name) : DdsInterface(robot_name, 16) {
        for (int i = 0; i < dof_num_; ++i) {
            pos_offset_[i] = 0.f;
            joint_dir_[i] = 1.f;
            data_updated_[i] = false;
            joint_config_[i].dir = 1.f;
            joint_config_[i].offset = 0.f;
        }
    }

    ~M20SimInterface() override = default;

    /**
     * Unlike the real-robot interface, sim2sim does not need the multi-turn
     * offset calibration. We immediately hold the Isaac Lab default standing
     * pose so the robot does not collapse while waiting for the first data.
     */
    void Start() override {
        MatXf cmd = MatXf::Zero(dof_num_, 5);
        // wheels are position-held at 0 so the robot does not roll before RL
        VecXf kp = Vec4f(80, 80, 80, 10.).replicate(4, 1);
        VecXf kd = Vec4f(2, 2, 2, 0.6).replicate(4, 1);
        VecXf pos = Vec4f(0.0, -0.6, 1.0, 0.0).replicate(4, 1);
        // rear legs are mirrored (hl/hr hipy +0.6, knee -1.0)
        pos(9) = 0.6;  pos(10) = -1.0;
        pos(13) = 0.6; pos(14) = -1.0;
        cmd.col(0) = kp;
        cmd.col(1) = pos;
        cmd.col(2) = kd;
        SetJointCommand(cmd);

        // wait until the sim feedback arrives (max ~1 s)
        for (int i = 0; i < 200 && !IsDataUpdatedFinished(); ++i) {
            RefreshRobotData();
            rclcpp::spin_some(this->get_node());
            usleep(5 * 1000);
        }
    }
};
