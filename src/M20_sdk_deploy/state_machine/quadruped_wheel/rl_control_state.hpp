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
#include <set>

namespace qw {
    class RLControlState : public StateBase {
    private:
        RobotBasicState rbs_[2];
        std::atomic<int> rbs_write_index_{0};
        // 有没有拿到过至少一帧**真实**观测。策略线程在 OnEnter 里就起来了，
        // 如果不在门禁后面等一帧，第一拍会拿默认构造的 rbs_（全零关节角、
        // 单位旋转矩阵）去算动作，而且这一"垃圾帧"会被写进 history 的最旧端，
        // 影响后面 10 个策略周期（DEF-017）。
        std::atomic<bool> rbs_ready_{false};
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
        // Policy obs 里的 `ee_goal` = **root（机体）坐标系**下的 EE 目标位姿
        // （训练 `HeightInvariantEECommand.command_local`）。默认值取"Piper 默认
        // 关节姿态下 `gripper_base` 在 root 系的位姿"，由
        // `scripts/check_mjcf_contract.py` 实测：pos=(0.3492, 0, 0.4326)（与训练值差
        // 5.7e-5 m）、相对旋转角差 0.000°。
        // ⚠️ 曾经写的是臂基座坐标系下的同一个位姿 (0.1092, 0, 0.3439)——
        // 两者差 24 cm，会让策略拿到一个落在机体内部的 EE 目标（DEF-019）。
        std::array<float, 7> ee_goal_{0.3492f, 0.0f, 0.4327f,   // pos (root frame)
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

        // ---- 安全接管（P0-7 / DEF-016）----
        float tilt_takeover_ = 0.8f;      // rad；训练终止阈值
        float leg_fold_takeover_ = 1.2f;  // rad；|q - q_default| 的兜底阈值
        std::set<std::string> warned_;

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
            rbs_ready_.store(true, std::memory_order_release);
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

            // 训练终值区间 (0.33, 0.55)，见 keyboard_interface.hpp 的说明（DEF-009）
            uc->body_height = std::clamp(vr_body_anchor_[0] + vr_body_off_[0],
                                         0.33f, 0.55f);
            uc->body_pitch = std::clamp(vr_body_anchor_[1] + vr_body_off_[1],
                                        -0.35f, 0.35f);
            uc->body_roll = std::clamp(vr_body_anchor_[2] + vr_body_off_[2],
                                       -0.25f, 0.25f);
        }

        void PolicyRunner() {
            int run_cnt_record = -1;
            while (start_flag_) {
                if (!rbs_ready_.load(std::memory_order_acquire)) {
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                    continue;
                }
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
                // 策略目录（含 policy.onnx + policy_layout.json）；环境变量可覆盖
                fs::path policy_dir = base / ".." / ".." / "policy" / "m20_piper_history_20260920";
                if (const char *e = std::getenv("M20_POLICY_DIR")) policy_dir = e;
                if (!fs::exists(policy_dir / "policy.onnx")) {
                    std::cerr << "[RLControlState] policy not found in " << policy_dir
                              << "\n把训练侧 <run>/exported_deploy/ 整个目录放到 "
                                 "src/M20_sdk_deploy/policy/<run 名>/（需要 policy.onnx + "
                                 "policy_layout.json），或用 M20_POLICY_DIR 指定目录。"
                              << std::endl;
                    exit(1);
                }
                const std::string policy_dir_abs = fs::canonical(policy_dir).string();
                piper_policy_ = std::make_shared<M20PiperPolicyRunner>("m20_piper_policy",
                                                                      policy_dir_abs);

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
            if (const char *e = std::getenv("M20_TILT_TAKEOVER")) {
                tilt_takeover_ = std::atof(e);
            }
            if (const char *e = std::getenv("M20_LEG_FOLD_TAKEOVER")) {
                leg_fold_takeover_ = std::atof(e);
            }
            std::cout << "[TAKEOVER] thresholds: tilt > " << tilt_takeover_
                      << " rad (" << tilt_takeover_ * 180.0 / M_PI << " deg), |q-q_default| > "
                      << leg_fold_takeover_ << " rad"
                      << " (M20_TILT_TAKEOVER / M20_LEG_FOLD_TAKEOVER 可覆盖，<=0 关闭)"
                      << std::endl;
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
            // 先把当前真实状态采一帧，再让策略线程跑（否则第一拍用的是全零观测）
            rbs_ready_.store(false, std::memory_order_release);
            UpdateRobotObservation();
            arm_ri_ptr_->Start();
            // 先让 runner 复位（清零 last_action / history / run_cnt_），
            // 再起策略线程 —— 否则第一拍可能读到上一段或未初始化的状态（DEF-018）
            policy_ptr_->OnEnter();
            run_policy_thread_ = std::thread(std::bind(&RLControlState::PolicyRunner, this));
            StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControlMode);
        };

        virtual void OnExit() {
            // 幂等：切换状态时调用一次，进程退出（QwStateMachine::Stop）时可能再调用一次。
            if (!run_policy_thread_.joinable()) {
                return;
            }
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
            // 接管阈值取训练的终止阈值（docs/sim2sim_layout_contract_zh.md）：
            //   倾角 = acos(-g_z) = acos(cos(roll)cos(pitch)) > 0.8 rad (45.8°)
            // 触发后由 StateMachineBase 切到 kJointDamping（在 JointDampingState 里
            // 零增益 + 阻尼，和训练里"终止后不再执行策略"的语义一致）。
            //   M20_TILT_TAKEOVER     倾角阈值 [rad]（默认 0.8；设 <=0 关闭）
            //   M20_LEG_FOLD_TAKEOVER 关节"折叠"阈值 [rad]（默认 1.2；设 <=0 关闭）
            //     腿被压在身下时 hipy/knee 会远离默认角，用 |q - q_default| 兜底
            const Vec3f rpy = ri_ptr_->GetImuRpy();
            if (tilt_takeover_ > 0.f) {
                const float cos_tilt = std::cos(rpy(0)) * std::cos(rpy(1));
                const float tilt = std::acos(std::max(-1.f, std::min(1.f, cos_tilt)));
                if (tilt > tilt_takeover_) {
                    WarnOnce("tilt", "tilt", tilt,
                             "rad > " + std::to_string(tilt_takeover_));
                    return true;
                }
            }
            if (leg_fold_takeover_ > 0.f) {
                const VecXf &q = ri_ptr_->GetJointPosition();
                // MJCF 序：每腿 (hipx, hipy, knee, wheel)
                static const float def_hipy[4] = {-0.6f, -0.6f, 0.6f, 0.6f};
                static const float def_knee[4] = {1.0f, 1.0f, -1.0f, -1.0f};
                for (int leg = 0; leg < 4; ++leg) {
                    const float d_hipy = std::fabs(q(leg * 4 + 1) - def_hipy[leg]);
                    const float d_knee = std::fabs(q(leg * 4 + 2) - def_knee[leg]);
                    if (d_hipy > leg_fold_takeover_ || d_knee > leg_fold_takeover_) {
                        WarnOnce("leg_fold", "leg fold", std::max(d_hipy, d_knee),
                                 "rad > " + std::to_string(leg_fold_takeover_));
                        return true;
                    }
                }
            }
            return false;
        }

        void WarnOnce(const char *key, const std::string &what, float value,
                      const std::string &how) {
            if (warned_.count(key)) return;
            warned_.insert(key);
            std::cout << "[TAKEOVER!] " << what << " = " << value << " (" << how
                      << ") -> switch to joint damping" << std::endl;
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
