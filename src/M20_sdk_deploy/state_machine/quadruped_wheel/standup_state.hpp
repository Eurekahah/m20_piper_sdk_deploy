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
        // M20_STANDUP_VENDOR ⇒ 用厂商原版（046b72d）的两段式站立：
        //   第一段折到 pre_height_ 蹲姿、第二段按高度规划升到 stand_height_，轮子 kp=0。
        //   **默认 1 = 厂商原版**（2026-10-14 按操作员要求把默认切回原厂，便于在 SDK 模式下
        //   做"完全和原版一致"的站立测试）；设 `M20_STANDUP_VENDOR=0` 则回到本项目行为
        //   （一步到训练默认姿态并保持，= RL 的入口姿态）。见 DEF-060 / TODO P1-9。
        bool vendor_mode_ = true;
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
                // 把关键数字打出来，方便与厂商原版逐项核对（第二段的目标角用同一个 IK 反算）
                std::cout << "[STANDUP] 厂商原版档（M20_STANDUP_VENDOR 默认 1）\n"
                          << "          第一段 " << cp_ptr_->stand_duration_ << " s → 蹲姿 pre_height "
                          << cp_ptr_->pre_height_ << " m：hipx " << init_hipx_pos_
                          << " / hipy " << GetHipYPosByHeight(cp_ptr_->pre_height_)
                          << " / knee " << GetKneePosByHeight(cp_ptr_->pre_height_) << "\n"
                          << "          第二段 " << cp_ptr_->stand_duration_ << " s → 站高 stand_height "
                          << (stand_height_m_ > 0.f ? stand_height_m_ : cp_ptr_->stand_height_) << " m：hipx "
                          << init_hipx_pos_ << " / hipy "
                          << GetHipYPosByHeight(stand_height_m_ > 0.f ? stand_height_m_ : cp_ptr_->stand_height_)
                          << " / knee "
                          << GetKneePosByHeight(stand_height_m_ > 0.f ? stand_height_m_ : cp_ptr_->stand_height_)
                          << "\n          腿 kp/kd " << cp_ptr_->swing_leg_kp_(0) << "/"
                          << cp_ptr_->swing_leg_kd_(0)
                          << "，轮 kp/kd 0/0（第一段）→ 0/" << set_wheel_kd_ << "（第二段）\n"
                          << "          ⚠️ 结束站位**不是**训练默认姿态；若随后要按 c 进 RL，"
                             "请改设 M20_STANDUP_VENDOR=0（或用完即撤）。" << std::endl;
            } else {
                // goal = the policy default standing pose (matches the Isaac Lab
                // init_state / USD default: hipy +/-0.6, knee -/+1.0, wheels 0),
                // so RL control starts from the exact state the policy was trained on.
                goal_joint_pos_ = Vec4f(0.0, -0.6, 1.0, 0.0).replicate(4, 1);
                goal_joint_pos_(4) = 0.0; goal_joint_pos_(12) = 0.0;
                goal_joint_pos_(9) = 0.6; goal_joint_pos_(10) = -1.0;
                goal_joint_pos_(13) = 0.6; goal_joint_pos_(14) = -1.0;
                std::cout << "[STANDUP] 训练默认姿态档（M20_STANDUP_VENDOR=0）："
                             "hipx 0 / hipy ∓0.6 / knee ±1.0 → 永久保持（= RL 入口姿态）"
                          << std::endl;
            }
            Vec4f one_leg_kp, one_leg_kd;
            if (vendor_mode_) {
                // 厂商原版：轮子 kp=0（第一段 kd=0，第二段 kd=set_wheel_kd_）
                one_leg_kp << cp_ptr_->swing_leg_kp_, 0;
                one_leg_kd << cp_ptr_->swing_leg_kd_, 0;
            } else {
                // 轮子 kp=0 / kd=1 —— 与真机一致（DEF-060）：真机 `M20Interface`
                // 对 i%4==3 强制 kp=0，所以旧的"轮子位置保持 kp=10"只有仿真里存在，
                // 真机收到的本来就是 0/1。kd=1 与厂商第二段相同；轮速目标恒为 0
                // ⇒ 两边都是纯阻尼 `tau = 1 * (0 - v)`，不会再"仿真刹住、真机滚动"。
                one_leg_kp << cp_ptr_->swing_leg_kp_, 0;
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
