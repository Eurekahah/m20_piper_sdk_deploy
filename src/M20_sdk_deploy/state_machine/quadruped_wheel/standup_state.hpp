/**
 * @file standup_state.hpp
 * @brief from sit state to stand state
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once

#include "state_base.h"

namespace qw{
class StandUpState : public StateBase{
private:
    VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
    double time_stamp_record_, run_time_;
    VecXf goal_joint_pos_, kp_, kd_;
    MatXf joint_cmd_;
    double stand_duration_ = 2.;
        // M20_STANDUP_VENDOR=1 ⇒ 用厂商原版（046b72d）的两段式站立：
        //   第一段折到 pre_height_ 蹲姿、第二段按高度规划升到 stand_height_，轮子 kp=0。
        //   默认 0 = 本项目行为（一步到训练默认姿态并保持）。见 DEF-060 / TODO P1-9。
        bool vendor_mode_ = false;
        // M20_STANDUP_HIPX_DEG=<度>：给四个 hipx 目标加一个**镜像对称**的偏置
        //   （fl/hl += d，fr/hr -= d），用来在真机上做"站姿 → 侧摆静力矩"的扫描
        //   （DEF-059 §7：厂商姿态把侧摆从 29 N·m 降到 10.7，但膝盖仍然过热）。
        //   默认 0 = 不动。⚠️ 会让站位偏离训练默认姿态 ⇒ 别带着它进 RL。
        float hipx_offset_ = 0.f;
        // M20_STANDUP_HEIGHT_M=<m>：配合 vendor_mode_ 用，覆盖厂商的 stand_height_（默认 0.48）。
        //   用来在真机上扫"站高 → 各关节静力矩"，因为 A/B 显示站高对侧摆负载影响很大
        //   而仿真对姿态不敏感（DEF-059 §7）。默认 = cp_ptr_->stand_height_。
        float stand_height_m_ = -1.f;

    const float init_hipx_pos_ = Deg2Rad(0.);
    const float set_wheel_kd_ = 1.;

    void GetRobotJointValue(){
        current_joint_pos_ = ri_ptr_->GetJointPosition();
        current_joint_vel_ = ri_ptr_->GetJointVelocity();
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
    }

    void RecordJointData(){
        init_joint_pos_ = current_joint_pos_;
        init_joint_vel_ = current_joint_vel_;
        time_stamp_record_ = run_time_;
    }

    float GetHipYPosByHeight(float h){
        float l1 = cp_ptr_->thigh_len_;
        float l2 = cp_ptr_->shank_len_;
        float default_pos = (cp_ptr_->fl_joint_lower_(1)+cp_ptr_->fl_joint_upper_(1)) / 2.;
        if(fabs(h) >= l1 + l2) {
            std::cerr << "error height input" << std::endl;
            return 0;
        }
        float theta = -acos((l1*l1+h*h-l2*l2)/(2.*h*l1));
        theta = LimitNumber(theta, cp_ptr_->fl_joint_lower_(1), cp_ptr_->fl_joint_upper_(1));
        return theta;
    }

    float GetKneePosByHeight(float h){
        float l1 = cp_ptr_->thigh_len_;
        float l2 = cp_ptr_->shank_len_;
        float default_pos = (cp_ptr_->fl_joint_lower_(2)+cp_ptr_->fl_joint_upper_(2)) / 2.;
        if(fabs(h) >= l1 + l2) {
            std::cerr << "error height input" << std::endl;
            return 0;
        }
        float theta = M_PI-acos((l1*l1+l2*l2-h*h)/(2*l1*l2));
        theta = LimitNumber(theta, cp_ptr_->fl_joint_lower_(2), cp_ptr_->fl_joint_upper_(2));
        return theta;
    }

public:
    StandUpState(const RobotName& robot_name, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_name, state_name, data_ptr){
            if (const char *e = std::getenv("M20_STANDUP_VENDOR")) vendor_mode_ = (std::string(e) != "0");
            if (vendor_mode_) {
                // 厂商原版：goal = 按 pre_height_ 算出的蹲姿（hipx = init_hipx_pos_ = 0），
                // 之后由第二段高度规划把机身抬到 stand_height_。
                goal_joint_pos_ = Vec4f(init_hipx_pos_, GetHipYPosByHeight(cp_ptr_->pre_height_),
                                        GetKneePosByHeight(cp_ptr_->pre_height_), 0).replicate(4, 1);
                goal_joint_pos_(4) = -init_hipx_pos_; goal_joint_pos_(12) = -init_hipx_pos_;
                // 后髋膝取反
                goal_joint_pos_(9) = -goal_joint_pos_(9);
                goal_joint_pos_(10) = -goal_joint_pos_(10);
                goal_joint_pos_(13) = -goal_joint_pos_(13);
                goal_joint_pos_(14) = -goal_joint_pos_(14);
                std::cout << "[STANDUP] M20_STANDUP_VENDOR=1 ⇒ 厂商原版两段式站立"
                             "（pre_height " << cp_ptr_->pre_height_ << " m → stand_height "
                          << cp_ptr_->stand_height_ << " m，轮子 kp=0/kd=1）。"
                             "注意：结束站位与训练默认姿态不同，进 RL 前建议先把开关去掉。"
                          << std::endl;
            } else {
                // goal = the policy default standing pose (matches the Isaac Lab
                // init_state / USD default: hipy +/-0.6, knee -/+1.0, wheels 0),
                // so RL control starts from the exact state the policy was trained on.
                goal_joint_pos_ = Vec4f(0.0, -0.6, 1.0, 0.0).replicate(4, 1);
                goal_joint_pos_(4) = 0.0; goal_joint_pos_(12) = 0.0;
                goal_joint_pos_(9) = 0.6; goal_joint_pos_(10) = -1.0;
                goal_joint_pos_(13) = 0.6; goal_joint_pos_(14) = -1.0;
            }
            Vec4f one_leg_kp, one_leg_kd;
            if (vendor_mode_) {
                // 厂商原版：轮子 kp=0（第一段 kd=0，第二段 kd=set_wheel_kd_）
                one_leg_kp << cp_ptr_->swing_leg_kp_, 0;
                one_leg_kd << cp_ptr_->swing_leg_kd_, 0;
            } else {
                // wheels: position-hold at 0 (kp 10) so the robot does not roll
                one_leg_kp << cp_ptr_->swing_leg_kp_, 10;
                one_leg_kd << cp_ptr_->swing_leg_kd_, 1;
            }
            kp_ = one_leg_kp.replicate(4, 1);
            kd_ = one_leg_kd.replicate(4, 1);
            joint_cmd_ = MatXf::Zero(16, 5);
            joint_cmd_.col(0) = kp_;
            joint_cmd_.col(2) = kd_;
            stand_duration_ = cp_ptr_->stand_duration_;

            if (const char *ho = std::getenv("M20_STANDUP_HIPX_DEG")) {
                hipx_offset_ = Deg2Rad(std::atof(ho));
            }
            if (const char *hs = std::getenv("M20_STANDUP_HEIGHT_M")) {
                stand_height_m_ = std::atof(hs);
            }
            if (stand_height_m_ > 0.f) {
                if (!vendor_mode_) {
                    std::cout << "[STANDUP] ⚠️ M20_STANDUP_HEIGHT_M 只对 M20_STANDUP_VENDOR=1 生效"
                                 "（当前是训练姿态档，忽略）" << std::endl;
                } else {
                    std::cout << "[STANDUP] M20_STANDUP_HEIGHT_M=" << stand_height_m_
                              << " m（覆盖厂商 stand_height_ = " << cp_ptr_->stand_height_
                              << " m）" << std::endl;
                }
            }
            if (hipx_offset_ != 0.f) {
                goal_joint_pos_(0) += hipx_offset_;   goal_joint_pos_(8) += hipx_offset_;
                goal_joint_pos_(4) -= hipx_offset_;   goal_joint_pos_(12) -= hipx_offset_;
                std::cout << "[STANDUP] M20_STANDUP_HIPX_DEG=" << (hipx_offset_ * 180. / M_PI)
                          << " deg ⇒ hipx 目标 (fl/fr/hl/hr) = " << goal_joint_pos_(0) << " / "
                          << goal_joint_pos_(4) << " / " << goal_joint_pos_(8) << " / "
                          << goal_joint_pos_(12) << "（镜像对称；偏离训练姿态，别进 RL）"
                          << std::endl;
            }
        }
    ~StandUpState(){}

    virtual void OnEnter() {
        GetRobotJointValue();
        RecordJointData();
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::StandingUp);
    };
    virtual void OnExit() {
    }
    virtual void Run() {
        GetRobotJointValue();
        VecXf planning_joint_pos(current_joint_pos_.rows());
        VecXf planning_joint_vel(current_joint_pos_.rows());
        if(vendor_mode_){
            double dt = 0.001;
            if(run_time_ - time_stamp_record_ <= stand_duration_){
                // 第一段：折到 pre_height_ 蹲姿
                for(int i=0;i<current_joint_pos_.rows();++i){
                    planning_joint_pos(i) = GetCubicSplinePos(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0,
                                                    run_time_ - time_stamp_record_, stand_duration_);
                    planning_joint_vel(i) = GetCubicSplineVel(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0,
                                                    run_time_ - time_stamp_record_, stand_duration_);
                    if(i%4==3){
                        planning_joint_vel(i) = 0.0;
                    }
                }
                for(int i=3;i<16;i+=4) kd_(i) = 0.f;          // 厂商：这一段的轮子完全自由
            }else{
                // 第二段：按高度规划从 pre_height_ 抬到 stand_height_
                double new_time = run_time_ - time_stamp_record_ - stand_duration_;
                const float goal_h = (stand_height_m_ > 0.f) ? stand_height_m_
                                                             : cp_ptr_->stand_height_;
                float plan_height = GetCubicSplinePos(cp_ptr_->pre_height_, 0, goal_h, 0,
                                                    new_time, stand_duration_);
                float plan_height_next = GetCubicSplinePos(cp_ptr_->pre_height_, 0, goal_h, 0,
                                                    new_time+dt, stand_duration_);
                float hipy_pos = GetHipYPosByHeight(plan_height);
                float hipy_vel = (GetHipYPosByHeight(plan_height_next) - hipy_pos) / dt;
                float knee_pos = GetKneePosByHeight(plan_height);
                float knee_vel = (GetKneePosByHeight(plan_height_next) - knee_pos) / dt;
                planning_joint_pos = Vec4f(init_hipx_pos_, hipy_pos, knee_pos, 0).replicate(4, 1);
                planning_joint_pos(4) = -init_hipx_pos_; planning_joint_pos(12) = -init_hipx_pos_;
                planning_joint_vel = Vec4f(0, hipy_vel, knee_vel, 0).replicate(4, 1);
                // 后髋膝取反
                planning_joint_pos(9) = -planning_joint_pos(9);
                planning_joint_pos(10) = -planning_joint_pos(10);
                planning_joint_pos(13) = -planning_joint_pos(13);
                planning_joint_pos(14) = -planning_joint_pos(14);
                planning_joint_vel(9) = -planning_joint_vel(9);
                planning_joint_vel(10) = -planning_joint_vel(10);
                planning_joint_vel(13) = -planning_joint_vel(13);
                planning_joint_vel(14) = -planning_joint_vel(14);
                for(int i=3;i<16;i+=4) kd_(i) = set_wheel_kd_;
            }
            joint_cmd_.col(2) = kd_;
        }else if(run_time_ - time_stamp_record_ <= stand_duration_){
            for(int i=0;i<current_joint_pos_.rows();++i){
                planning_joint_pos(i) = GetCubicSplinePos(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, stand_duration_);
                planning_joint_vel(i) = GetCubicSplineVel(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, stand_duration_);
                if(i%4==3){
                    planning_joint_vel(i) = 0.0;
                }
            }
        }else{
            // hold the policy default standing pose
            planning_joint_pos = goal_joint_pos_;
            planning_joint_vel.setZero();
        }

        joint_cmd_.col(1) = planning_joint_pos;
        joint_cmd_.col(3) = planning_joint_vel;
        ri_ptr_->SetJointCommand(joint_cmd_);
    }
    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::JointDamping)) return true;
        return false;
    }
    virtual StateName GetNextStateName() {
        if(uc_ptr_->GetUserCommand()->safe_control_mode!=0) return StateName::kJointDamping;
        if(run_time_ - time_stamp_record_ <= 2.*stand_duration_){
            return StateName::kStandUp;
        }else{
            if(uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::RLControlMode)){
                return StateName::kRLControl;
            }else if(uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::LieDown)){
                return StateName::kLieDown;
            }
        }
        return StateName::kStandUp;
    }
};

};
