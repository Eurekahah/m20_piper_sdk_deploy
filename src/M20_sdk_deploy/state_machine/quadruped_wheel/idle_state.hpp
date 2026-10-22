/**
 * @file idle_state.hpp
 * @brief robot need to confirm sensor input while in idle state
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once


#include "state_base.h"

namespace qw {
    class IdleState : public StateBase {
    private:
        int joint_normal_flag_ = -1;
        int imu_normal_flag_ = -1;
        bool first_enter_flag_ = true;
        VecXf joint_pos_, joint_vel_, joint_tau_;
        VecXf hold_joint_pos_;
        // M20_IDLE_ZERO_GAIN ⇒ 厂商原版 idle：发零增益（完全不使力，机器人就地塌着，
        //   等按 z 由站立状态机拉起来）。**默认 1 = 厂商原版**（2026-10-14 按操作员确认改的：
        //   "idle 状态就应该是塌的"）。设 `M20_IDLE_ZERO_GAIN=0` 回到本项目行为
        //   （kp 80/2 主动保持进入 idle 时的姿态，sim2sim 里用它防塌）。见 DEF-060。
        bool zero_gain_ = true;
        Vec3f rpy_, acc_, omg_;
        double enter_state_time_ = 10000.;
        double last_print_time = 0;

        void GetProprioceptiveData() {
            joint_pos_ = ri_ptr_->GetJointPosition();
            joint_vel_ = ri_ptr_->GetJointVelocity();
            joint_tau_ = ri_ptr_->GetJointTorque();
            rpy_ = ri_ptr_->GetImuRpy();
            acc_ = ri_ptr_->GetImuAcc();
            omg_ = ri_ptr_->GetImuOmega();
        }

        int JointDataNormalCheck() {
            VecXf joint_pos_lower(16), joint_pos_upper(16);
            Vec4f fl_lower, fl_upper;
            fl_lower << cp_ptr_->fl_joint_lower_, -INFINITY;
            fl_upper << cp_ptr_->fl_joint_upper_, INFINITY;
            Vec4f fr_lower(-fl_upper(0), fl_lower(1), fl_lower(2), -INFINITY);
            Vec4f fr_upper(-fl_lower(0), fl_upper(1), fl_upper(2), INFINITY);
            joint_pos_lower << fl_lower, fr_lower, fl_lower, fr_lower;
            joint_pos_upper << fl_upper, fr_upper, fl_upper, fr_upper;
            for (int i = 0; i < 16; ++i) {
                if (std::isnan(joint_pos_(i))){
                    return 1;
                }
                if (joint_pos_(i) > joint_pos_upper(i) + 0.2){
                    return 2;
                }
                if (joint_pos_(i) < joint_pos_lower(i) - 0.2){
                    return 3;
                }
                if (std::isnan(joint_vel_(i))) {
                    return 4;
                } else {
                    if (i % 4 == 3) {
                        if (joint_vel_(i) > cp_ptr_->wheel_vel_limit_ + 0.1) return 5;
                    } else {
                        if (joint_vel_(i) > cp_ptr_->joint_vel_limit_(i % 4) + 0.1) return 6;
                    }
                }
            }
            return 0;
        }

        int ImuDataNormalCheck() {
            for (int i = 0; i < 3; ++i) {
                if (std::isnan(rpy_(i)) || fabs(rpy_(i)) > M_PI) {
                    return 1;
                }
                if (std::isnan(omg_(i)) || fabs(omg_(i)) > M_PI) {
                    return 2;
                }
            }
            if (acc_.norm() < 0.1 * gravity || acc_.norm() > 3.0 * gravity) {
                return 3;
            }
            return 0;
        }

        void DisplayProprioceptiveInfo() {
            std::cout << "Joint Data: \n";
            std::cout << "FL pos: ";
            PrintEigenVectorf(joint_pos_.segment(0, 4));
            std::cout << "FR pos: ";
            PrintEigenVectorf(joint_pos_.segment(4, 4));
            std::cout << "HL pos: ";
            PrintEigenVectorf(joint_pos_.segment(8, 4));
            std::cout << "HR pos: ";
            PrintEigenVectorf(joint_pos_.segment(12, 4));
            std::cout << "vel: ";
            PrintEigenVectorf(joint_vel_);
            std::cout << "tau: ";
            PrintEigenVectorf(joint_tau_);
            std::cout << "Imu Data: \n";
            std::cout << "rpy: " << rpy_.transpose() << std::endl;
            std::cout << "acc: " << acc_.transpose() << std::endl;
            std::cout << "omg: " << omg_.transpose() << std::endl;
        }

        void DisplayAxisValue() {
            auto cmd = uc_ptr_->GetUserCommand();
            std::cout << "User Command Input: \n";
            std::cout << "axis value:  " << cmd->forward_vel_scale << " "
                      << cmd->side_vel_scale << " "
                      << cmd->turnning_vel_scale << std::endl;
            std::cout << "target mode: " << int(cmd->target_mode) << std::endl;
        }

        void PrintEigenVectorf(const VecXf &vec, int width = 6) {
            for (int i = 0; i < vec.size(); ++i) {
                std::cout << std::setw(width)
                          << std::setprecision(3)
                          << std::right
                          << vec(i) << " ";  // 输出矩阵元素
            }
            std::cout << "\n";
        }


    public:
        IdleState(const RobotName &robot_name, const std::string &state_name,
                  std::shared_ptr<ControllerData> data_ptr) : StateBase(robot_name, state_name, data_ptr) {
            if (const char *e = std::getenv("M20_IDLE_ZERO_GAIN")) {
                zero_gain_ = (std::string(e) != "0");
            }
            if (zero_gain_) {
                std::cout << "[IDLE] 厂商原版 idle（M20_IDLE_ZERO_GAIN 默认 1）：发零增益、完全不使力；"
                             "机器人会就地塌下，按 z 由站立状态机拉起（厂商流程）。"
                             "要改用主动保持请设 M20_IDLE_ZERO_GAIN=0。" << std::endl;
            } else {
                std::cout << "[IDLE] 主动保持档（M20_IDLE_ZERO_GAIN=0）：腿 kp 80/kd 2、"
                             "轮 kp 0/kd 0.6 保持进入 idle 时的姿态" << std::endl;
            }
        }

        ~IdleState() {}

        virtual void OnEnter() {
            StateBase::msfb_.UpdateCurrentState(RobotMotionState::WaitingForStand);
            enter_state_time_ = ri_ptr_->GetInterfaceTimeStamp();
            // lock the initial pose so the robot does not collapse while idle
            GetProprioceptiveData();
            hold_joint_pos_ = joint_pos_;
            // hold the wheels at zero so the robot does not roll while idle
            for (int i = 3; i < 16; i += 4) hold_joint_pos_(i) = 0.f;
        };

        virtual void OnExit() {
            first_enter_flag_ = false;
        }

        virtual void Run() {
            
            GetProprioceptiveData();
            joint_normal_flag_ = JointDataNormalCheck();
            if (((ri_ptr_->GetInterfaceTimeStamp() - last_print_time) > 1)) {
                // DisplayProprioceptiveInfo();
                // DisplayAxisValue();
                last_print_time = ri_ptr_->GetInterfaceTimeStamp();
            }
            // hold the current joints (instead of zero gains) so the robot does
            // not collapse while waiting for the user command
            MatXf cmd = MatXf::Zero(16, 5);
            if (!zero_gain_) {
                // 腿 80/2、轮 kp=0 / kd=0.6 —— 与真机**完全一致**（DEF-060）：
                // 真机 `M20Interface::SetJointCommand` 对 i%4==3 强制 kp=0（轮子是
                // 速度/阻尼模式），所以"轮子位置保持 kp=10"只在仿真里成立、真机从来
                // 收不到。仿真也发 kp=0 之后，轮子的语义在两边都是
                // `tau = 0.6 * (0 - v)`（速度目标 0 + 阻尼）。
                VecXf hold_kp = Vec4f(80, 80, 80, 0.).replicate(4, 1);
                VecXf hold_kd = Vec4f(2, 2, 2, 0.6).replicate(4, 1);
                cmd.col(0) = hold_kp;
                cmd.col(1) = hold_joint_pos_;
                cmd.col(2) = hold_kd;
            }
            ri_ptr_->SetJointCommand(cmd);
        }

        virtual bool LoseControlJudge() {
            return false;
        }

        virtual StateName GetNextStateName() {
            if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::StandingUp)) {
                if (joint_normal_flag_ != 0) {
                    std::cout << "========== warning ========>> "
                              << "stand rejected: joint status = "
                              << joint_normal_flag_ << std::endl;
                    return StateName::kIdle;
                }
                if (uc_ptr_->GetUserCommand()->safe_control_mode != 0) {
                    std::cout << "========== warning ========>> "
                              << "stand rejected: safe_control_mode = "
                              << int(uc_ptr_->GetUserCommand()->safe_control_mode)
                              << std::endl;
                    return StateName::kIdle;
                }
                return StateName::kStandUp;
            }
            return StateName::kIdle;
        }
    };

};
