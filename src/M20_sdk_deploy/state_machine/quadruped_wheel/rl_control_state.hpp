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

        Eigen::MatrixXf acc_rot = Eigen::MatrixXf::Zero(20, 3);
        int acc_rot_count = 0;

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

        void PolicyRunner() {
            int run_cnt_record = -1;
            while (start_flag_) {
                if (state_run_cnt_ % policy_ptr_->decimation_ == 0 && state_run_cnt_ != run_cnt_record) {
                    timespec start_timestamp, end_timestamp;
                    clock_gettime(CLOCK_MONOTONIC, &start_timestamp);

                    // refresh the absolute EE goal from arm_controller feedback (obs)
                    {
                        std::lock_guard<std::mutex> lock(ee_mutex_);
                        UserCommand* uc = uc_ptr_->GetUserCommand();
                        uc->ee_goal_pos[0] = ee_goal_[0];
                        uc->ee_goal_pos[1] = ee_goal_[1];
                        uc->ee_goal_pos[2] = ee_goal_[2];
                        uc->ee_goal_quat[0] = ee_goal_[3];
                        uc->ee_goal_quat[1] = ee_goal_[4];
                        uc->ee_goal_quat[2] = ee_goal_[5];
                        uc->ee_goal_quat[3] = ee_goal_[6];
                    }

                    auto ra = policy_ptr_->getRobotAction(rbs_[getrbsReadIndex()], *(uc_ptr_->GetUserCommand()));

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
            }

            policy_ptr_ = piper_policy_;
            if (!policy_ptr_) {
                std::cerr << "error policy" << std::endl;
                exit(0);
            }
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
