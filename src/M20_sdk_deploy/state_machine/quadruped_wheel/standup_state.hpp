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
            // goal = the policy default standing pose (matches the Isaac Lab
            // init_state / USD default: hipy +/-0.6, knee -/+1.0, wheels 0),
            // so RL control starts from the exact state the policy was trained on.
            goal_joint_pos_ = Vec4f(0.0, -0.6, 1.0, 0.0).replicate(4, 1);
            goal_joint_pos_(4) = 0.0; goal_joint_pos_(12) = 0.0;
            goal_joint_pos_(9) = 0.6; goal_joint_pos_(10) = -1.0;
            goal_joint_pos_(13) = 0.6; goal_joint_pos_(14) = -1.0;

            Vec4f one_leg_kp, one_leg_kd;
            // wheels: position-hold at 0 (kp 10) so the robot does not roll
            one_leg_kp << cp_ptr_->swing_leg_kp_, 10;
            one_leg_kd << cp_ptr_->swing_leg_kd_, 1;
            kp_ = one_leg_kp.replicate(4, 1);
            kd_ = one_leg_kd.replicate(4, 1);
            joint_cmd_ = MatXf::Zero(16, 5);
            joint_cmd_.col(0) = kp_;
            joint_cmd_.col(2) = kd_;
            stand_duration_ = cp_ptr_->stand_duration_;
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
        if(run_time_ - time_stamp_record_ <= stand_duration_){
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
