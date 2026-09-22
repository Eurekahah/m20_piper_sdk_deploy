#pragma once

#include "robot_interface.h"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include "dds_interface.hpp"

class M20Interface : public DdsInterface {
protected:
    void ResetPositionOffset() {
        memset(data_updated_, 0, dof_num_ * sizeof(bool));
        this->SetJointCommand(MatXf::Zero(dof_num_, 5));

        VecXf last_joint_pos = this->GetJointPosition();
        VecXf current_joint_pos = this->GetJointPosition();
        int cnt = 0;
        // 这段自检的本意：确认"真的收到了关节反馈"，并据此决定多圈编码器的 ±360°
        // 偏移（只用 hipy / knee 两列）。原实现要求**16 个关节（含 4 个轮子）
        // 全部发生变化**，且**无限等待** —— 真机上：
        //   * 不在 SDK 模式 / 收不到 /JOINTS_DATA 时，位置恒为 0，永远不满足；
        //   * 机器人站着不动、轮子不转时，也永远不满足；
        // 表现就是每隔 10 s 打印一次 `joint data update is not finished`，
        // 程序卡在启动里、机器人还被发了零增益（会软倒）。
        // 因此这里改成：**有限等待 + 明确诊断 + 按需降级**，绝不无限卡住。
        //   M20_JOINT_UPDATE_TIMEOUT_S=5   等待上限（秒）；0 = 完全跳过自检
        double timeout_s = 5.0;
        if (const char *e = std::getenv("M20_JOINT_UPDATE_TIMEOUT_S")) {
            timeout_s = std::atof(e);
        }
        const auto t0 = std::chrono::steady_clock::now();
        auto elapsed_s = [&t0]() {
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        };
        double next_report = 1.0;
        bool finished = IsDataUpdatedFinished();
        while (!finished && elapsed_s() < timeout_s) {
            ++cnt;
            usleep(1000);

            rclcpp::spin_some(this->get_node());
            current_joint_pos = this->GetJointPosition();
            for (int i = 0; i < dof_num_; ++i) {
                if (!data_updated_[i] && current_joint_pos(i) != last_joint_pos(i) &&
                    !std::isnan(current_joint_pos(i))) {
                    data_updated_[i] = true;
                }
            }
            last_joint_pos = current_joint_pos;
            finished = IsDataUpdatedFinished();
            if (!finished && elapsed_s() >= next_report) {
                next_report += 1.0;
                int n_up = 0;
                std::ostringstream missing;
                for (int i = 0; i < dof_num_; ++i) {
                    if (data_updated_[i]) {
                        ++n_up;
                    } else if (missing.tellp() < 200) {
                        missing << i << " ";
                    }
                }
                std::cout << "[M20Interface] 等待关节反馈变化 " << n_up << "/" << dof_num_
                          << "（未变化: " << missing.str() << "），已等 " << elapsed_s()
                          << "s / " << timeout_s << "s" << std::endl;
            }
        }

        int n_updated = 0;
        std::ostringstream missing;
        for (int i = 0; i < dof_num_; ++i) {
            if (data_updated_[i]) {
                ++n_updated;
            } else {
                missing << i << " ";
            }
        }
        // 决策：偏移校正只依赖 i%4==1(hipy)、2(knee)，这两列没更新就不能校正。
        const bool offset_cols_updated =
                [&]() {
                    for (int i = 1; i < dof_num_; i += 4) {
                        if (!data_updated_[i]) return false;
                    }
                    for (int i = 2; i < dof_num_; i += 4) {
                        if (!data_updated_[i]) return false;
                    }
                    return true;
                }();

        if (run_cnt_ == 0) {
            std::cout << "[M20Interface] ⚠️ 完全没有收到 /JOINTS_DATA（收到 0 条）。\n"
                         "    请确认：① 机器人已进入 SDK 模式（手柄 + 授权码）；\n"
                         "              ② ROS_DOMAIN_ID 与机器人服务一致；\n"
                         "              ③ `ros2 topic hz /JOINTS_DATA` 有数据。\n"
                         "    这次不会等下去：跳过 ±360° 偏移自检（坐标系可能未标定）。"
                      << std::endl;
        } else if (!finished) {
            std::cout << "[M20Interface] ⚠️ 自检超时：" << timeout_s << "s 内只有 "
                      << n_updated << "/" << dof_num_ << " 个关节发生变化（未变化: "
                      << missing.str() << "）。\n"
                         "    常见原因：机器人（或轮子）本来就静止不动 —— 这个自检要求\n"
                         "    **每个关节都动过一次**，站着不动时轮子永远不满足。\n"
                         "    处理：可以继续（见下），或把机器人置于放松/被轻微移动的状态再启动。"
                      << std::endl;
        }
        if (!offset_cols_updated) {
            std::cout << "[M20Interface] ⚠️ hipy/knee 没有全部发生变化 ⇒ **跳过**多圈 ±360° "
                         "偏移自检（保持构造函数里的默认 offset 表）。\n"
                         "    如果机器人当前 hipy/knee 停在多圈区间外，坐标系可能不准；\n"
                         "    请先手动把腿摆到接近默认姿态再启动，或设置 "
                         "M20_JOINT_UPDATE_TIMEOUT_S=10 重试。"
                      << std::endl;
        }
        // 打印一次实测角度（诊断用；真机上最需要看到的就是这 16 个数）
        {
            std::ostringstream out;
            out << "[M20Interface] joint pos (rad) = ";
            for (int i = 0; i < dof_num_; ++i) {
                out << std::fixed << std::setprecision(3) << current_joint_pos(i) << " ";
            }
            std::cout << out.str() << std::endl;
        }

        if (!offset_cols_updated) {
            return;   // 不做偏移校正，保持默认标定
        }
        for (int i = 1; i < dof_num_; i += 4) {
            if (current_joint_pos(i) < -140. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] + 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            } else if (current_joint_pos(i) > 140. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] - 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            }
        }
        for (int i = 2; i < dof_num_; i += 4) {
            if (current_joint_pos(i) < -164. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] + 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            } else if (current_joint_pos(i) > 164. / 180. * M_PI) {
                pos_offset_[i] = pos_offset_[i] - 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            }
        }
        for (int i = 0; i < dof_num_; ++i) {
            joint_config_[i].dir = joint_dir_[i];
            joint_config_[i].offset = Deg2Rad(pos_offset_[i]);
        }
    }

public:
    M20Interface(const std::string &robot_name) : DdsInterface(robot_name, 16) {
        battery_data_.resize(2 * BATTERY_DATA_SIZE);

        float init_pos_offset[16] = {-25, -131, 160, 0.,
                                     25, -131, 160, 0,
                                     -25, 131, -160, 0,
                                     25, 131, -160, 0};
        float joint_dir[16] = {1, 1, -1, 1,
                               1, -1, 1, -1,
                               -1, 1, -1, 1.,
                               -1, -1, 1, -1};
        for (int i = 0; i < dof_num_; ++i) {
            pos_offset_[i] = init_pos_offset[i];
            joint_dir_[i] = joint_dir[i];
            data_updated_[i] = false;
            joint_config_[i].dir = joint_dir_[i];
            joint_config_[i].offset = Deg2Rad(pos_offset_[i]);
        }
    }

    ~M20Interface() {
    }

    virtual void Start() {
        time_stamp_ = GetTimestampMs();
        ResetPositionOffset();
    }

    virtual void Stop() {
    }

    /**
     * Sends joint commands to the robot.
     * Input matrix layout: [dof × 5] → [kp, pos, kd, vel, torque_ff]
     * 
     * ⚠ Torque values (column 4) are protected by low-level hardware torque limits
     *    → commands exceeding limits are saturated at the firmware/hardware level
     * 
     * Joints with index % 4 == 3 have kp forced to 0 (velocity/torque mode)
     */
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) {
        auto msg = drdds::msg::JointsDataCmd();
        for (int i = 0; i < dof_num_; ++i) {
            msg.data.joints_data[i].position =
                    (input(i, 1) - joint_config_[i].offset) * joint_config_[i].dir;
            msg.data.joints_data[i].velocity = input(i, 3) * joint_dir_[i];
            msg.data.joints_data[i].torque = input(i, 4) * joint_dir_[i];
            msg.data.joints_data[i].kp = input(i, 0);
            if (i % 4 == 3) {
                msg.data.joints_data[i].kp = 0;
            }
            msg.data.joints_data[i].kd = input(i, 2);
            msg.data.joints_data[i].control_word = kIndexMotorControl;
        }
        joint_cmd_pub_->publish(msg);

    }

};
