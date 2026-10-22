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
        // 发**零增益**（= 厂商原版流程：收到第一条命令之前不使力，机器人就着初始位姿待着）。
        // ⚠️ 之前这里发的是"训练默认站姿 + kp 80/2"的保持命令，导致 sim2sim 里
        // rl_deploy 一起来机器人就自己站起来了（按 z 之前），与真机/厂商流程不符。
        // 现在保持"不使力"：MuJoCo 节点自己的默认保持（kp 80/2、目标 = 初始位姿
        // `M20_SIM_INIT_POSE`）已经保证机器人不会乱塌 —— 与厂商 sim（启动时
        // kp/kd/pos 全 0、机器人就着 `URDF_INIT` 趴着）一致。
        SetJointCommand(MatXf::Zero(dof_num_, 5));

        // wait until the sim feedback arrives (max ~1 s)
        for (int i = 0; i < 200 && !IsDataUpdatedFinished(); ++i) {
            RefreshRobotData();
            rclcpp::spin_some(this->get_node());
            usleep(5 * 1000);
        }
    }
};
