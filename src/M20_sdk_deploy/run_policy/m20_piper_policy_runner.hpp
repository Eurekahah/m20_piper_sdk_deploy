/**
 * @file m20_piper_policy_runner.hpp
 * @brief Policy runner for the M20 + Piper history-adaptation policy
 * @author M20+Piper deploy
 * @date 2026-08-20
 *
 * The policy was trained with rsl_rl's ActorCriticHistory (RMA-style): a
 * HistoryEncoder (TCN) compresses a 10-step proprioceptive window into a
 * 32-dim latent, and the actor consumes [policy_obs, latent].
 *
 * The deployment ONNX (policy/history_adaptation_full.onnx) already contains
 * encoder + actor, with:
 *
 *   inputs:  obs          [1, 86]   policy obs (single step)
 *            obs_history  [1, 770]  10 steps x 77 single-step obs
 *   outputs: actions      [1, 23]   12 leg pos + 4 wheel vel + 7 ee_ik (ignored)
 *
 * Policy obs (86) layout, matching env.yaml observations.policy:
 *   [0:3]    base angular velocity      (x 0.25)
 *   [3:6]    projected gravity
 *   [6:9]    base velocity command      (vx, vy, wz)
 *   [9:31]   joint pos rel. default     (12 legs + 4 wheels(0) + 6 arm)
 *   [31:53]  joint velocities           (x 0.05)
 *   [53:76]  last action                (23, processed like Isaac Lab)
 *   [76:83]  ee goal (pos 3 + quat wxyz 4)
 *   [83:86]  body pose command (height, pitch, roll)
 *
 * Single-step history obs (77), matching history_single_step_obs:
 *   [0:3]    base angular velocity      (raw)
 *   [3:6]    projected gravity
 *   [6:30]   joint pos rel. default     (all 24 joints, incl. wheels/gripper)
 *   [30:54]  joint velocities           (raw)
 *   [54:77]  last action                (23)
 *
 * Action (23): [0:12] leg pos targets (hipx x0.125, hipy/knee x0.25),
 * [12:16] wheel vel targets (x5.0), [16:23] ee_ik part (bypassed, ignored).
 */

#pragma once
#define PI 3.14159265358979323846

#include "policy_runner_base.hpp"

#include <onnxruntime_cxx_api.h>
#include <onnxruntime_c_api.h>

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

class M20PiperPolicyRunner : public PolicyRunnerBase {
private:
    // ---- model topology (from the history_adaptation training run) ----
    static constexpr int kLegDof = 12;         // 4 legs x (hipx, hipy, knee)
    static constexpr int kWheelDof = 4;
    static constexpr int kArmDof = 6;          // arm_joint1..6
    static constexpr int kGripperDof = 2;
    static constexpr int kIkActionDim = 7;     // CommandDrivenIKAction.action_dim
    static constexpr int kPolicyJointDof = kLegDof + kWheelDof + kArmDof;  // 22
    static constexpr int kHistoryJointDof = kPolicyJointDof + kGripperDof; // 24
    static constexpr int kRobotDof = kHistoryJointDof;                     // 24
    static constexpr int kActionDim = kLegDof + kWheelDof + kIkActionDim;  // 23
    static constexpr int kEeGoalDim = 7;
    static constexpr int kBodyPoseDim = 3;
    static constexpr int kObsDim = 3 + 3 + 3 + kPolicyJointDof + kPolicyJointDof +
                                   kActionDim + kEeGoalDim + kBodyPoseDim;  // 86
    static constexpr int kHistorySteps = 10;
    static constexpr int kHistoryStepDim = 3 + 3 + kHistoryJointDof + kHistoryJointDof +
                                           kActionDim;                      // 77
    static constexpr int kHistoryObsDim = kHistorySteps * kHistoryStepDim;  // 770
    static constexpr int kLatentDim = 32;

    const std::string policy_path_;

    VecXf kp_, kd_;
    VecXf default_robot_;       // 24, MJCF order (Isaac Lab default joints)
    VecXf default_policy_;      // 22, policy obs order
    VecXf action_default_pos_;  // 12, leg default positions in action order
    std::vector<float> action_scale_;    // 16 (legs + wheels only)
    std::vector<int> action2robot_idx_;  // 24, -1 for arm/gripper
    std::vector<int> robot2policy_idx_;  // 22
    std::vector<int> robot2history_idx_;  // 24 (MJCF order -> USD joint order)

    Vec3f gravity_direction_ = Vec3f(0., 0., -1.);
    float omega_scale_ = 0.25f;
    float dof_vel_scale_ = 0.05f;

    VecXf joint_pos_obs_, joint_vel_obs_, last_action_obs_, current_action_,
          current_observation_, history_obs_, history_step_, robot_goal_;
    bool history_initialized_ = false;

    RobotAction robot_action_;

    Ort::Env env_;
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info{nullptr};
    const char* input_names_[2] = {"obs", "obs_history"};
    const char* output_names_[1] = {"actions"};
    std::array<int64_t, 2> obs_shape_ = {1, kObsDim};
    std::array<int64_t, 2> history_shape_ = {1, kHistoryObsDim};

public:
    M20PiperPolicyRunner(const std::string &policy_name, const std::string &policy_path)
        : PolicyRunnerBase(policy_name), policy_path_(policy_path),
          env_(ORT_LOGGING_LEVEL_WARNING, "M20PiperPolicyRunner"),
          session_options_{},
          session_{nullptr},
          memory_info(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault)) {

        if (access(policy_path_.c_str(), F_OK) != 0) {
            std::cerr << "[M20PiperPolicyRunner] Model file not found: " << policy_path_
                      << "\nGenerate it with scripts/export_history_policy_onnx.py or place "
                         "your exported ONNX there." << std::endl;
            throw std::runtime_error("Model file missing");
        }

        session_options_.SetIntraOpNumThreads(4);
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);

        // verify the model input/output dims
        int num_inputs = static_cast<int>(session_.GetInputCount());
        int num_outputs = static_cast<int>(session_.GetOutputCount());
        if (num_inputs != 2 || num_outputs != 1) {
            std::cerr << "[M20PiperPolicyRunner] expected 2 inputs / 1 output, got "
                      << num_inputs << " / " << num_outputs << std::endl;
            throw std::runtime_error("Policy I/O mismatch");
        }
        auto in0 = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        auto in1 = session_.GetInputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape();
        auto out0 = session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        int model_obs = (in0.size() == 2 && in0[1] > 0) ? static_cast<int>(in0[1]) : kObsDim;
        int model_hist = (in1.size() == 2 && in1[1] > 0) ? static_cast<int>(in1[1]) : kHistoryObsDim;
        int model_action = (out0.size() == 2 && out0[1] > 0) ? static_cast<int>(out0[1]) : kActionDim;
        if (model_obs != kObsDim || model_hist != kHistoryObsDim || model_action != kActionDim) {
            std::cerr << "[M20PiperPolicyRunner] model dims obs=" << model_obs
                      << " history=" << model_hist << " actions=" << model_action
                      << " ; expected " << kObsDim << " / " << kHistoryObsDim
                      << " / " << kActionDim << std::endl;
            throw std::runtime_error("Policy dim mismatch");
        }

        InitTopology();
        InitDefaults();

        robot_action_.kp = kp_;
        robot_action_.kd = kd_;
        robot_action_.tau_ff = VecXf::Zero(kRobotDof);
        robot_action_.goal_joint_pos = default_robot_;
        robot_action_.goal_joint_vel = VecXf::Zero(kRobotDof);

        current_observation_.setZero(kObsDim);
        history_step_.setZero(kHistoryStepDim);
        history_obs_.setZero(kHistoryObsDim);
        last_action_obs_.setZero(kActionDim);
        current_action_.setZero(kActionDim);
        joint_pos_obs_.setZero(kPolicyJointDof);
        joint_vel_obs_.setZero(kPolicyJointDof);
        robot_goal_ = default_robot_;

        SetDecimation(4);  // sim 5ms x 4 = 20ms policy tick (50 Hz)
    }

    ~M20PiperPolicyRunner() override = default;

    void DisplayPolicyInfo() override {
        std::cout << "[M20PiperPolicyRunner] obs=" << kObsDim
                  << " history=" << kHistorySteps << "x" << kHistoryStepDim
                  << " latent=" << kLatentDim
                  << " action=" << kActionDim
                  << " robot_dof=" << kRobotDof << std::endl;
    }

    void OnEnter() override {
        run_cnt_ = 0;
        cmd_vel_input_.setZero();
        last_action_obs_.setZero(kActionDim);
        current_action_.setZero(kActionDim);
        robot_goal_ = default_robot_;
        history_initialized_ = false;
    }

private:
    // ---- joint name tables (MJCF order and the training obs order) ----
    static const std::vector<std::string>& RobotOrder() {
        static const std::vector<std::string> v = {
            "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
            "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
            "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
            "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint",
            "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6",
            "gripper_joint1", "gripper_joint2"};
        return v;
    }

    static const std::vector<std::string>& PolicyJointOrder() {
        static const std::vector<std::string> v = {
            "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint",
            "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint",
            "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint",
            "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint",
            "fl_wheel_joint", "fr_wheel_joint", "hl_wheel_joint", "hr_wheel_joint",
            "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6"};
        return v;
    }

    // The USD articulation exposes joints in arm-first order (verified with
    // usd-core on M20_Piper_own.usd): arm1..6, gripper1..2, then fl/fr/hl/hr.
    // The history obs uses ALL joints in this USD order.
    static const std::vector<std::string>& HistoryJointOrder() {
        static const std::vector<std::string> v = {
            "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6",
            "gripper_joint1", "gripper_joint2",
            "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
            "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
            "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
            "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint"};
        return v;
    }

    static const std::vector<std::string>& ActionJointOrder() {
        static const std::vector<std::string> v = {
            "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint",
            "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint",
            "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint",
            "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint",
            "fl_wheel_joint", "fr_wheel_joint", "hl_wheel_joint", "hr_wheel_joint"};
        return v;
    }

    std::vector<int> GeneratePermutation(const std::vector<std::string>& from,
                                         const std::vector<std::string>& to) {
        std::unordered_map<std::string, int> idx_map;
        for (size_t i = 0; i < from.size(); ++i) idx_map[from[i]] = static_cast<int>(i);
        std::vector<int> perm;
        perm.reserve(to.size());
        for (const auto& name : to) {
            auto it = idx_map.find(name);
            perm.push_back(it != idx_map.end() ? it->second : -1);
        }
        return perm;
    }

    void InitTopology() {
        robot2policy_idx_ = GeneratePermutation(RobotOrder(), PolicyJointOrder());
        robot2history_idx_ = GeneratePermutation(RobotOrder(), HistoryJointOrder());
        const auto& action_order = ActionJointOrder();
        action2robot_idx_.resize(kRobotDof, -1);
        for (int r = 0; r < kRobotDof; ++r) {
            for (size_t a = 0; a < action_order.size(); ++a) {
                if (RobotOrder()[r] == action_order[a]) {
                    action2robot_idx_[r] = static_cast<int>(a);
                    break;
                }
            }
        }
    }

    void InitDefaults() {
        // default joints from the Isaac Lab init_state (M20_Piper_own USD):
        //   hipx 0, fl/fr hipy -0.6, hl/hr hipy +0.6,
        //   fl/fr knee +1.0, hl/hr knee -1.0, wheels 0,
        //   arm [0, 0.5, -0.5, 0, 0, 0], gripper closed (0, 0)
        default_robot_.resize(kRobotDof);
        default_robot_ << 0.0, -0.6, 1.0, 0.0,
                          0.0, -0.6, 1.0, 0.0,
                          0.0,  0.6, -1.0, 0.0,
                          0.0,  0.6, -1.0, 0.0,
                          0.0, 0.5, -0.5, 0.0, 0.0, 0.0,
                          0.0, 0.0;

        default_policy_.resize(kPolicyJointDof);
        default_policy_ << 0.0, -0.6, 1.0,
                           0.0, -0.6, 1.0,
                           0.0,  0.6, -1.0,
                           0.0,  0.6, -1.0,
                           0.0, 0.0, 0.0, 0.0,
                           0.0, 0.5, -0.5, 0.0, 0.0, 0.0;

        action_default_pos_.resize(kLegDof);
        action_default_pos_ << 0.0, -0.6, 1.0,
                               0.0, -0.6, 1.0,
                               0.0,  0.6, -1.0,
                               0.0,  0.6, -1.0;

        action_scale_ = {0.125f, 0.25f, 0.25f,
                         0.125f, 0.25f, 0.25f,
                         0.125f, 0.25f, 0.25f,
                         0.125f, 0.25f, 0.25f,
                         5.0f, 5.0f, 5.0f, 5.0f};

        // legs: kp 80 / kd 2; wheels: kp 0 / kd 0.6 (from training config)
        kp_ = Vec4f(80, 80, 80, 0.).replicate(4, 1);
        kd_ = Vec4f(2, 2, 2, 0.6).replicate(4, 1);
        VecXf zero_arm = VecXf::Zero(kGripperDof + kArmDof);
        kp_.conservativeResize(kRobotDof);
        kd_.conservativeResize(kRobotDof);
        kp_.tail(kGripperDof + kArmDof) = zero_arm;
        kd_.tail(kGripperDof + kArmDof) = zero_arm;
    }

    VecXf OnnxInfer(const VecXf& obs, const VecXf& history) {
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, const_cast<float*>(obs.data()), obs.size(),
            obs_shape_.data(), obs_shape_.size());
        Ort::Value history_tensor = Ort::Value::CreateTensor<float>(
            memory_info, const_cast<float*>(history.data()), history.size(),
            history_shape_.data(), history_shape_.size());

        std::vector<Ort::Value> inputs;
        inputs.emplace_back(std::move(input_tensor));
        inputs.emplace_back(std::move(history_tensor));

        auto outputs = session_.Run(Ort::RunOptions{nullptr},
                                    input_names_, inputs.data(), 2,
                                    output_names_, 1);
        float* action_data = outputs[0].GetTensorMutableData<float>();
        Eigen::Map<Eigen::VectorXf> action_map(action_data, kActionDim);
        return VecXf(action_map);
    }

public:
    RobotAction getRobotAction(const RobotBasicState &ro, const UserCommand &uc) override {
        Vec3f base_omega = ro.base_omega * omega_scale_;
        Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity_direction_;
        Vec3f vel_cmd(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);

        // policy obs: joints in policy order, relative to default; wheels zeroed in pos
        for (int i = 0; i < kPolicyJointDof; ++i) {
            joint_pos_obs_(i) = ro.joint_pos(robot2policy_idx_[i]) - default_policy_(i);
            joint_vel_obs_(i) = ro.joint_vel(robot2policy_idx_[i]) * dof_vel_scale_;
        }
        joint_pos_obs_.segment(kLegDof, kWheelDof).setZero();

        VecXf ee_goal(kEeGoalDim);
        ee_goal << uc.ee_goal_pos[0], uc.ee_goal_pos[1], uc.ee_goal_pos[2],
                   uc.ee_goal_quat[0], uc.ee_goal_quat[1], uc.ee_goal_quat[2], uc.ee_goal_quat[3];
        VecXf body_pose(kBodyPoseDim);
        body_pose << uc.body_height, uc.body_pitch, uc.body_roll;

        current_observation_ << base_omega, projected_gravity, vel_cmd,
                                joint_pos_obs_, joint_vel_obs_, last_action_obs_,
                                ee_goal, body_pose;

        // single-step history obs (raw, all 24 joints, same last_action)
        history_step_.head(3) = ro.base_omega;
        history_step_.segment(3, 3) = projected_gravity;
        for (int i = 0; i < kHistoryJointDof; ++i) {
            int r = robot2history_idx_[i];  // USD order -> MJCF robot index
            history_step_(6 + i) = ro.joint_pos(r) - default_robot_(r);
            history_step_(6 + kHistoryJointDof + i) = ro.joint_vel(r);
        }
        history_step_.tail(kActionDim) = last_action_obs_;

        if (!history_initialized_) {
            // RMA-style history buffer: repeat the first single-step obs
            for (int s = 0; s < kHistorySteps; ++s) {
                history_obs_.segment(s * kHistoryStepDim, kHistoryStepDim) = history_step_;
            }
            history_initialized_ = true;
        } else {
            // shift window: drop the oldest step, append the current one
            history_obs_.head(kHistoryObsDim - kHistoryStepDim) =
                history_obs_.tail(kHistoryObsDim - kHistoryStepDim);
            history_obs_.tail(kHistoryStepDim) = history_step_;
        }

        current_action_ = OnnxInfer(current_observation_, history_obs_);

        // last_action obs uses the processed action, like Isaac Lab's
        // action_manager.action: legs default+scale*raw, wheels scale*raw,
        // ee_ik part stays raw (bypassed, not used for control).
        for (int i = 0; i < kLegDof; ++i) {
            last_action_obs_(i) = action_default_pos_(i) + action_scale_[i] * current_action_(i);
        }
        for (int i = kLegDof; i < kLegDof + kWheelDof; ++i) {
            last_action_obs_(i) = action_scale_[i] * current_action_(i);
        }
        for (int i = kLegDof + kWheelDof; i < kActionDim; ++i) {
            last_action_obs_(i) = current_action_(i);
        }

        // build the robot command (MJCF order, 24 rows; arm rows keep default + zero gain)
        robot_goal_ = default_robot_;
        robot_action_.goal_joint_vel.setZero();
        for (int r = 0; r < kRobotDof; ++r) {
            int ai = action2robot_idx_[r];
            if (ai >= 0) {
                if (r % 4 == 3) {
                    robot_action_.goal_joint_vel(r) = action_scale_[ai] * current_action_(ai);
                } else {
                    robot_goal_(r) += action_scale_[ai] * current_action_(ai);
                }
            }
        }
        robot_action_.goal_joint_pos = robot_goal_;

        ++run_cnt_;
        return robot_action_;
    }
};
