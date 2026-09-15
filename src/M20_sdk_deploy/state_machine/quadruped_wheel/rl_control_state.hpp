/**
 * @file rl_control_state.hpp
 * @brief rl policy runnning state for quadruped-wheel robot
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once
#include "state_base.h"
#include "policy_runner_base.hpp"
#include "m20_policy_runner.hpp"
#include "m20_piper_policy_runner.hpp"
#include "robot_interface.h"
#include "hardware/piper_arm_interface.hpp"
#include "user_command_interface.h"
#include "json.hpp"
#include "basic_function.hpp"

#include <std_msgs/msg/float32_multi_array.hpp>

#include <array>
#include <cstdlib>
#include <mutex>

namespace qw {
    class RLControlState : public StateBase {
    private:
        RobotBasicState rbs_[2];
        std::atomic<int> rbs_write_index_{0};
        int getrbsReadIndex() const { return 1 - rbs_write_index_.load(std::memory_order_acquire); }

        int state_run_cnt_;

        std::shared_ptr<PolicyRunnerBase> policy_ptr_;
        std::shared_ptr<M20PolicyRunner> m20_policy_;
        std::shared_ptr<M20PiperPolicyRunner> piper_policy_;
        std::shared_ptr<PiperArmInterface> arm_ri_ptr_;

        std::thread run_policy_thread_;
        bool start_flag_ = true;

        float policy_cost_time_ = 1;

        // arm teleop is owned by the standalone arm_teleop node; rl_deploy only
        // receives back the absolute EE goal (used in the policy obs)
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr arm_state_sub_;
        std::mutex ee_mutex_;
        std::array<float, 7> ee_goal_{0.1092f, 0.0f, 0.3439f,   // pos
                                      0.7373f, 0.0f, 0.6756f, 0.0f};  // quat wxyz

        // VR leg/body teleop (published by vr_teleop_node at 50 Hz)
        std::mutex vr_mutex_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vr_teleop_sub_;
        std::atomic<bool> vr_active_{false};
        bool vr_calibrate_ = false;
        bool vr_calib_prev_ = false;
        bool vr_anchor_valid_ = false;
        bool vr_reset_prev_ = false;
        float vr_vel_[3] = {0.f, 0.f, 0.f};
        float vr_body_off_[3] = {0.f, 0.f, 0.f};
        float vr_body_anchor_[3] = {0.513f, 0.f, 0.f};  // height/pitch/roll

        Eigen::MatrixXf acc_rot = Eigen::MatrixXf::Zero(20, 3);
        int acc_rot_count = 0;

        // ---- wheeled-leg joint velocity monitor -------------------------------
        // The 4 wheel joints are the only ones driven in velocity mode
        // (kp = 0, kd = 0.6, target = action_scale * action = 5 * action), so a
        // frame/sign/scale/limit mistake shows up there first: their feedback is
        // what the policy sees as joint_vel[12:16] (policy order) / [3,7,11,15]
        // (MJCF order). Print measured vs commanded wheel velocity, plus the
        // wheel position (unbounded multi-turn value, only zeroed inside the
        // policy joint_pos obs), at ~1 Hz.
        //   M20_JVEL_DEBUG=0        -> silence
        //   M20_JVEL_PERIOD=<ticks> -> print period in policy ticks (default 50)
        bool jvel_debug_ = true;
        int jvel_period_ = 50;   // 50 policy ticks = 1 s at the 50 Hz policy rate
        int jvel_cnt_ = 0;

        void PrintLegWheelVelocity(const RobotAction &ra, const RobotBasicState &rbs) {
            static const char *leg_name[4] = {"fl", "fr", "hl", "hr"};
            std::ostringstream out;
            out << std::fixed << std::setprecision(3);
            out << "[JVEL] meas rad/s (hipx hipy knee wheel):";
            for (int leg = 0; leg < 4; ++leg) {
                out << " " << leg_name[leg] << "[";
                for (int j = 0; j < 4; ++j) {
                    out << " " << rbs.joint_vel(leg * 4 + j);
                }
                out << " ]";
            }
            out << " | wheel q rad:";
            for (int leg = 0; leg < 4; ++leg) {
                out << " " << leg_name[leg] << " " << rbs.joint_pos(leg * 4 + 3);
            }
            out << "\n";

            Vec3f rpy_deg = rbs.base_rpy * (180.0f / static_cast<float>(M_PI));
            out << "[JVEL] rpy deg " << rpy_deg.transpose()
                << " | wheel target rad/s:";
            for (int leg = 0; leg < 4; ++leg) {
                int r = leg * 4 + 3;
                out << " " << leg_name[leg] << " " << ra.goal_joint_vel(r)
                    << " (raw a " << ra.goal_joint_vel(r) / 5.0f << ")";
            }
            out << " | wheel kp " << ra.kp(3) << " kd " << ra.kd(3)
                << " | leg kp " << ra.kp(0) << " kd " << ra.kd(0);
            std::cout << out.str() << std::endl;
        }

        void UpdateRobotObservation() {
            int write_idx = rbs_write_index_.load(std::memory_order_relaxed);
            RobotBasicState& buffer = rbs_[write_idx];

            buffer.base_rpy = ri_ptr_->GetImuRpy();
            buffer.base_rot_mat = RpyToRm(buffer.base_rpy);
            buffer.base_omega = ri_ptr_->GetImuOmega();
            buffer.base_acc = ri_ptr_->GetImuAcc();

            // legs (16) from the M20 interface, arm/gripper (8) from the Piper interface
            buffer.joint_pos.head(16) = ri_ptr_->GetJointPosition();
            buffer.joint_vel.head(16) = ri_ptr_->GetJointVelocity();
            buffer.joint_tau.head(16) = ri_ptr_->GetJointTorque();
            buffer.joint_pos.tail(8) = arm_ri_ptr_->GetJointPosition();
            buffer.joint_vel.tail(8) = arm_ri_ptr_->GetJointVelocity();
            buffer.joint_tau.tail(8) = arm_ri_ptr_->GetJointTorque();

            // 储存
            buffer.flt_base_acc_mat.row(acc_rot_count) = buffer.base_acc.transpose();
            acc_rot_count += 1;
            acc_rot_count = acc_rot_count % 20;

            rbs_write_index_.store(1 - write_idx,  std::memory_order_release);
        }

        void ApplyVrCommand(UserCommand* uc) {
            if (!vr_active_.load()) return;

            std::lock_guard<std::mutex> lock(vr_mutex_);
            if (vr_calibrate_ || !vr_anchor_valid_) {
                // re-anchor body pose to the current commanded target so the
                // VR controller offset does not cause a jump on B press
                vr_body_anchor_[0] = uc->body_height;
                vr_body_anchor_[1] = uc->body_pitch;
                vr_body_anchor_[2] = uc->body_roll;
                vr_anchor_valid_ = true;
                vr_calibrate_ = false;
                std::cout << "[VR] body anchor = "
                          << vr_body_anchor_[0] << " / "
                          << vr_body_anchor_[1] << " / "
                          << vr_body_anchor_[2]
                          << ", applying vr_vel = "
                          << vr_vel_[0] << " / " << vr_vel_[1] << " / "
                          << vr_vel_[2] << std::endl;
            }

            uc->forward_vel_scale = vr_vel_[0];
            uc->side_vel_scale = vr_vel_[1];
            uc->turnning_vel_scale = vr_vel_[2];

            uc->body_height = std::clamp(vr_body_anchor_[0] + vr_body_off_[0],
                                         0.33f, 0.60f);
            uc->body_pitch = std::clamp(vr_body_anchor_[1] + vr_body_off_[1],
                                        -0.35f, 0.35f);
            uc->body_roll = std::clamp(vr_body_anchor_[2] + vr_body_off_[2],
                                       -0.25f, 0.25f);
        }

        void PolicyRunner() {
            int run_cnt_record = -1;
            while (start_flag_) {
                if (state_run_cnt_ % policy_ptr_->decimation_ == 0 && state_run_cnt_ != run_cnt_record) {
                    timespec start_timestamp, end_timestamp;
                    clock_gettime(CLOCK_MONOTONIC, &start_timestamp);

                    UserCommand* uc = uc_ptr_->GetUserCommand();

                    // refresh the absolute EE goal from arm_controller feedback (obs)
                    {
                        std::lock_guard<std::mutex> lock(ee_mutex_);
                        uc->ee_goal_pos[0] = ee_goal_[0];
                        uc->ee_goal_pos[1] = ee_goal_[1];
                        uc->ee_goal_pos[2] = ee_goal_[2];
                        uc->ee_goal_quat[0] = ee_goal_[3];
                        uc->ee_goal_quat[1] = ee_goal_[4];
                        uc->ee_goal_quat[2] = ee_goal_[5];
                        uc->ee_goal_quat[3] = ee_goal_[6];
                    }

                    // VR owns velocity/body fields while active
                    ApplyVrCommand(uc);

                    const RobotBasicState &rbs = rbs_[getrbsReadIndex()];
                    auto ra = policy_ptr_->getRobotAction(rbs, *uc);

                    if (jvel_debug_ && (++jvel_cnt_ % jvel_period_ == 0)) {
                        PrintLegWheelVelocity(ra, rbs);
                    }

                    MatXf res = ra.ConvertToMat();

                    ri_ptr_->SetJointCommand(res);
                    run_cnt_record = state_run_cnt_;
                    clock_gettime(CLOCK_MONOTONIC, &end_timestamp);
                    policy_cost_time_ = (end_timestamp.tv_sec - start_timestamp.tv_sec) * 1e3
                                        + (end_timestamp.tv_nsec - start_timestamp.tv_nsec) / 1e6;

                }
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }

    public:
        RLControlState(const RobotName &robot_name, const std::string &state_name,
                       std::shared_ptr<ControllerData> data_ptr) : StateBase(robot_name, state_name, data_ptr) {
            if (robot_name_ == RobotName::M20) {
                namespace fs = std::filesystem;
                fs::path base = fs::path(__FILE__).parent_path();
                auto model_path = base / ".." / ".." / "policy" / "history_adaptation_full.onnx";
                if (!fs::exists(model_path)) {
                    std::cerr << "[RLControlState] policy not found: " << model_path
                              << "\nGenerate it with scripts/export_history_policy_onnx.py "
                                 "and place it at policy/history_adaptation_full.onnx" << std::endl;
                    exit(0);
                }
                auto model_path_abs = fs::canonical(model_path);
                piper_policy_ = std::make_shared<M20PiperPolicyRunner>("m20_piper_policy", model_path_abs.string());

                auto node = ri_ptr_->get_node();
                arm_ri_ptr_ = std::make_shared<PiperArmInterface>("M20PiperArm", node);
                arm_state_sub_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
                    "/ARM_TELEOP_STATE", 10,
                    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                        if (msg->data.size() >= 7) {
                            std::lock_guard<std::mutex> lock(ee_mutex_);
                            for (int i = 0; i < 7; ++i) ee_goal_[i] = msg->data[i];
                        }
                    });

                vr_teleop_sub_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
                    "/VR_TELEOP", 10,
                    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                        // layout in vr_teleop_protocol.py (16 floats)
                        if (msg->data.size() < 16) return;
                        bool active = msg->data[13] > 0.5f;
                        {
                            std::lock_guard<std::mutex> lock(vr_mutex_);
                            vr_active_ = active;
                            // calibrate/reset are edge-triggered pulses so a
                            // repeated 1 in the stream cannot re-anchor every tick
                            bool calib = msg->data[14] > 0.5f;
                            if (calib && !vr_calib_prev_) vr_calibrate_ = true;
                            vr_calib_prev_ = calib;
                            bool reset = msg->data[15] > 0.5f;
                            if (reset && !vr_reset_prev_) vr_anchor_valid_ = false;
                            vr_reset_prev_ = reset;
                            for (int i = 0; i < 3; ++i) {
                                vr_vel_[i] = msg->data[i];
                                vr_body_off_[i] = msg->data[9 + i];
                            }
                        }
                        // while VR is active the keyboard must not overwrite
                        // velocity/body fields
                        uc_ptr_->SetRemoteActive(active);
                        if (active) {
                            ApplyVrCommand(uc_ptr_->GetUserCommand());
                        }
                    });
            }

            policy_ptr_ = piper_policy_;
            if (!policy_ptr_) {
                std::cerr << "error policy" << std::endl;
                exit(0);
            }
            if (const char *e = std::getenv("M20_JVEL_DEBUG")) {
                jvel_debug_ = (std::string(e) != "0");
            }
            if (const char *p = std::getenv("M20_JVEL_PERIOD")) {
                int period = std::atoi(p);
                if (period > 0) jvel_period_ = period;
            }
            std::cout << "[JVEL] wheeled-leg joint velocity print "
                      << (jvel_debug_ ? "enabled" : "disabled")
                      << ", period " << jvel_period_ << " policy ticks"
                      << " (M20_JVEL_DEBUG=0 disables)" << std::endl;
            policy_ptr_->DisplayPolicyInfo();
        }

        ~RLControlState() {}

        virtual void OnEnter() {
            state_run_cnt_ = -1;
            start_flag_ = true;
            arm_ri_ptr_->Start();
            run_policy_thread_ = std::thread(std::bind(&RLControlState::PolicyRunner, this));
            policy_ptr_->OnEnter();
            StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControlMode);
        };

        virtual void OnExit() {
            start_flag_ = false;
            run_policy_thread_.join();
            arm_ri_ptr_->Stop();
            state_run_cnt_ = -1;
        }

        virtual void Run() {
            UpdateRobotObservation();
            state_run_cnt_++;
        }

        virtual bool LoseControlJudge() {
            if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::JointDamping)) return true;
            return PostureUnsafeCheck();
        }

        bool PostureUnsafeCheck() {
            // Vec3f rpy = ri_ptr_->GetImuRpy();
            // if(rpy(0) > 30./180*M_PI || rpy(1) > 45./180*M_PI){
            //     std::cout << "posture value: " << 180./M_PI*rpy.transpose() << std::endl;
            //     return true;
            // }
            return false;
        }

        virtual StateName GetNextStateName() {
            if (uc_ptr_->GetUserCommand()->safe_control_mode != 0) 
                return StateName::kJointDamping;
            if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::LieDown))
                return StateName::kLieDown;
            
            return StateName::kRLControl;
        }
    };
};
