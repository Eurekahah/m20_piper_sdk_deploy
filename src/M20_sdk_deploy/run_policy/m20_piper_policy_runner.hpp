/**
 * @file m20_piper_policy_runner.hpp
 * @brief Layout-driven policy runner for the M20 + Piper history-adaptation policy
 * @author M20+Piper deploy
 * @date 2026-09-20
 *
 * 这个 runner **不再写死任何观测/动作维度**：接口从策略目录里的
 * `policy_layout.json`（由训练侧 `export_deploy_policy.py` 导出）读出来，
 * 并与 ONNX 的输入输出形状交叉断言。换策略 = 换一个目录。
 *
 * 策略：rsl_rl `ActorCriticHistory`（TCN history encoder + actor），
 * run `2026-09-20_00-50-31` / `model_19999.pt`，资产 `M20_Piper_own`。
 *
 * 目录约定（`policy/<run 名>/`）：
 *   policy.onnx           输入 `policy_obs[batch,83]` + `history_flat[batch,700]`
 *                         输出 `action[batch,16]`
 *   policy_layout.json    kind=history / 各维度 / history 语义 / onnx 自检结论
 *
 * 观测（83 = 3+3+3+24+24+16+7+3），来源：训练 `observations.policy`
 *   [ 0: 3] base_ang_vel      x0.25
 *   [ 3: 6] projected_gravity
 *   [ 6: 9] velocity_commands (vx, vy, wz) 机体系
 *   [ 9:33] joint_pos  24 个关节 **原生序**，`q - q_default`，**4 个轮子列置零**
 *   [33:57] joint_vel  24 个关节原生序，x0.05
 *   [57:73] actions    上一步的 16 维动作（= 实际下发的值）
 *   [73:80] ee_goal    [pos(3), quat wxyz(4)]，root 系，clip ±3
 *   [80:83] body_pose_cmd [height, pitch, roll]
 *
 * history（10 x 70）每步：raw base_ang_vel(3) + projected_gravity(3)
 *   + joint_pos(24, 原生序, 含轮子不置零, `q - q_default`) + joint_vel(24)
 *   + last_action(16)，整窗最旧→最新，只 clip ±100（不乘 scale）。
 *
 * 动作（16）：[0:12] 12 腿位置（`q_default + gain*a`，hipx 0.125 其余 0.25）
 *   + [12:16] 4 轮速度（`ω_des = 5.0*a`，力矩 `0.6*(ω_des-ω)`）。
 *   **动作里没有机械臂** —— 臂由 `arm_controller` 的 IK 从 `ee_pose` 命令驱动。
 *
 * 原生关节序（探针 `probe_deploy_layout.py` 在 M20_Piper_own 上的实测输出，
 * 2026-09-20；见 docs/sim2sim_layout_contract_zh.md 第 3 节）：
 *   0-3 fl/fr/hl/hr_hipx, 4 arm_joint1, 5-8 fl/fr/hl/hr_hipy, 9 arm_joint2,
 *   10-13 fl/fr/hl/hr_knee, 14 arm_joint3, 15-18 fl/fr/hl/hr_wheel,
 *   19-21 arm_joint4/5/6, 22-23 gripper_joint1/2
 */

#pragma once
#define PI 3.14159265358979323846

#include "policy_runner_base.hpp"
#include "json.hpp"

#include <onnxruntime_cxx_api.h>
#include <onnxruntime_c_api.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using nlohmann::json;

class M20PiperPolicyRunner : public PolicyRunnerBase {
public:
    // ---- 固定的 M20+Piper 结构（与策略无关）----
    static constexpr int kLegDof = 12;         // 4 腿 x (hipx, hipy, knee)
    static constexpr int kWheelDof = 4;
    static constexpr int kArmDof = 6;          // arm_joint1..6
    static constexpr int kGripperDof = 2;
    static constexpr int kRobotDof = kLegDof + kWheelDof + kArmDof + kGripperDof;  // 24
    static constexpr int kJointObsDof = kRobotDof;   // 24（本 checkpoint 含夹爪）
    static constexpr int kEeGoalDim = 7;
    static constexpr int kBodyPoseDim = 3;

private:
    // ---- 从 policy_layout.json 读出来的接口维度 ----
    int obs_dim_ = 0;
    int history_steps_ = 0;
    int history_step_dim_ = 0;
    int history_obs_dim_ = 0;
    int action_dim_ = 0;
    int latent_dim_ = 0;
    std::string policy_dir_, onnx_path_, layout_path_;

    // ---- 关节表 ----
    std::vector<std::string> robot_order_;     // MJCF / 接口顺序（= 本仓库 robot_interface 的 24 维）
    std::vector<std::string> native_order_;    // articulation 原生序（观测列序）
    std::vector<std::string> action_order_;    // 动作槽位顺序（12 腿 + 4 轮）
    std::vector<int> robot2native_idx_;        // robot(MJCF) -> native
    std::vector<int> action2robot_idx_;        // action slot -> robot(MJCF)，-1 = 无

    VecXf kp_, kd_;
    VecXf default_robot_;    // 24, MJCF 序
    VecXf default_native_;   // 24, native 序
    VecXf action_default_pos_;   // 12, 动作序的腿默认角
    std::vector<float> action_scale_;  // 16（前 12 腿 + 4 轮）

    float omega_scale_ = 0.25f;
    float dof_vel_scale_ = 0.05f;
    float ee_goal_clip_ = 3.0f;      // 训练侧 ee_goal 的 clip
    float action_clip_ = 100.0f;     // 训练侧 clip_actions

    VecXf joint_pos_obs_, joint_vel_obs_, last_action_obs_, current_action_,
          current_observation_, history_obs_, history_step_, robot_goal_;
    bool history_initialized_ = false;
    RobotAction robot_action_;

    Ort::Env env_;
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info{nullptr};
    std::array<const char*, 2> input_names_{"policy_obs", "history_flat"};
    std::array<const char*, 1> output_names_{"action"};
    std::array<int64_t, 2> obs_shape_{1, 1};
    std::array<int64_t, 2> history_shape_{1, 1};

public:
    /**
     * @param policy_name 仅供日志
     * @param policy_dir  含 policy.onnx + policy_layout.json 的目录
     *                    （环境变量 M20_POLICY_ONNX / M20_POLICY_LAYOUT 可覆盖单个文件）
     */
    M20PiperPolicyRunner(const std::string &policy_name, const std::string &policy_dir)
        : PolicyRunnerBase(policy_name), policy_dir_(policy_dir),
          env_(ORT_LOGGING_LEVEL_WARNING, "M20PiperPolicyRunner"),
          session_options_{}, session_{nullptr},
          memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {

        namespace fs = std::filesystem;
        onnx_path_ = (fs::path(policy_dir_) / "policy.onnx").string();
        layout_path_ = (fs::path(policy_dir_) / "policy_layout.json").string();
        if (const char *e = std::getenv("M20_POLICY_ONNX")) onnx_path_ = e;
        if (const char *e = std::getenv("M20_POLICY_LAYOUT")) layout_path_ = e;

        if (access(onnx_path_.c_str(), F_OK) != 0) {
            std::cerr << "[M20PiperPolicyRunner] policy not found: " << onnx_path_
                      << "\n  换策略：把训练侧 <run>/exported_deploy/ 整个目录放到"
                         " src/M20_sdk_deploy/policy/<run 名>/，或用 M20_POLICY_ONNX 指定。"
                      << std::endl;
            throw std::runtime_error("policy missing");
        }
        if (access(layout_path_.c_str(), F_OK) != 0) {
            std::cerr << "[M20PiperPolicyRunner] layout not found: " << layout_path_
                      << "\n  policy_layout.json 由训练侧 export_deploy_policy.py 导出，"
                         "必须和 policy.onnx 放在一起。" << std::endl;
            throw std::runtime_error("layout missing");
        }

        LoadLayout();
        InitTopology();
        InitDefaults();
        OpenSession();

        std::cout << "[M20PiperPolicyRunner] layout kind=history obs=" << obs_dim_
                  << " history=" << history_steps_ << "x" << history_step_dim_
                  << " latent=" << latent_dim_ << " action=" << action_dim_
                  << " (from " << layout_path_ << ")" << std::endl;
        std::cout << "[M20PiperPolicyRunner] action clip = ±" << action_clip_
                  << " (训练 clip_actions); 喂回 actions 观测的是**实际下发**的值"
                  << std::endl;

        robot_action_.kp = kp_;
        robot_action_.kd = kd_;
        robot_action_.tau_ff = VecXf::Zero(kRobotDof);
        robot_action_.goal_joint_pos = default_robot_;
        robot_action_.goal_joint_vel = VecXf::Zero(kRobotDof);

        current_observation_.setZero(obs_dim_);
        history_step_.setZero(history_step_dim_);
        history_obs_.setZero(history_obs_dim_);
        last_action_obs_.setZero(action_dim_);
        current_action_.setZero(action_dim_);
        joint_pos_obs_.setZero(kJointObsDof);
        joint_vel_obs_.setZero(kJointObsDof);
        robot_goal_ = default_robot_;

        SetDecimation(4);   // sim 5 ms x 4 = 20 ms 策略周期（50 Hz）
    }

    ~M20PiperPolicyRunner() override = default;

    void DisplayPolicyInfo() override {
        std::cout << "[M20PiperPolicyRunner] obs=" << obs_dim_
                  << " history=" << history_steps_ << "x" << history_step_dim_
                  << " action=" << action_dim_ << " robot_dof=" << kRobotDof
                  << " policy_dir=" << policy_dir_ << std::endl;
    }

    void OnEnter() override {
        run_cnt_ = 0;
        cmd_vel_input_.setZero();
        last_action_obs_.setZero(action_dim_);
        current_action_.setZero(action_dim_);
        robot_goal_ = default_robot_;
        history_initialized_ = false;      // 下一帧用同一帧填满整窗
        robot_action_.goal_joint_pos = default_robot_;
        robot_action_.goal_joint_vel.setZero();
    }

private:
    // ---------------------------------------------------------------
    // 布局
    // ---------------------------------------------------------------
    void LoadLayout() {
        std::ifstream f(layout_path_);
        if (!f.is_open()) throw std::runtime_error("cannot open layout json");
        json j;
        f >> j;

        const std::string kind = j.value("kind", std::string(""));
        if (kind != "history") {
            std::cerr << "[M20PiperPolicyRunner] layout kind='" << kind
                      << "'，本 runner 只支持 kind=history（history encoder + actor）。"
                         "如果用 play.py 导出的 actor-only policy.pt/onnx，latent 没有来源，"
                         "必须用 export_deploy_policy.py 重新导出。" << std::endl;
            throw std::runtime_error("unsupported layout kind");
        }
        obs_dim_ = j.at("policy_obs_dim").get<int>();
        history_steps_ = j.at("history_length").get<int>();
        history_step_dim_ = j.at("history_single_step_dim").get<int>();
        history_obs_dim_ = history_steps_ * history_step_dim_;
        action_dim_ = j.at("action_dim").get<int>();
        latent_dim_ = j.value("latent_dim", 0);

        // 本 runner 只处理这一种结构；差异直接报错而不是静默算错。
        const int expect_obs = 3 + 3 + 3 + kJointObsDof + kJointObsDof + action_dim_ +
                               kEeGoalDim + kBodyPoseDim;
        if (obs_dim_ != expect_obs) {
            std::cerr << "[M20PiperPolicyRunner] policy_obs_dim=" << obs_dim_
                      << " 与 '3+3+3+24+24+action(" << action_dim_ << ")+7+3="
                      << expect_obs << "' 不一致：布局与 runner 假设的观测结构不同。"
                      << std::endl;
            throw std::runtime_error("obs layout mismatch");
        }
        const int expect_step = 3 + 3 + kJointObsDof + kJointObsDof + action_dim_;
        if (history_step_dim_ != expect_step) {
            std::cerr << "[M20PiperPolicyRunner] history step=" << history_step_dim_
                      << "，期望 " << expect_step << "（= 3+3+24+24+action）" << std::endl;
            throw std::runtime_error("history layout mismatch");
        }
        if (action_dim_ != kLegDof + kWheelDof) {
            std::cerr << "[M20PiperPolicyRunner] action_dim=" << action_dim_
                      << "，本版策略应为 " << (kLegDof + kWheelDof)
                      << "（12 腿 + 4 轮，动作里没有机械臂）" << std::endl;
            throw std::runtime_error("action layout mismatch");
        }
        if (const char *e = std::getenv("M20_ACTION_CLIP")) {
            action_clip_ = std::atof(e);
        }
    }

    void OpenSession() {
        session_options_.SetIntraOpNumThreads(4);
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        session_ = Ort::Session(env_, onnx_path_.c_str(), session_options_);

        if (session_.GetInputCount() != 2 || session_.GetOutputCount() != 1) {
            std::cerr << "[M20PiperPolicyRunner] 期望 2 输入 / 1 输出，实际 "
                      << session_.GetInputCount() << " / " << session_.GetOutputCount()
                      << std::endl;
            throw std::runtime_error("policy I/O count mismatch");
        }
        // ⚠️ ORT C++ API 的两个生命周期陷阱（DEF-014）：
        //  1) GetInputNameAllocated 的 allocator 不能传 nullptr：
        //     AllocatedStringPtr 析构时会调用 allocator->Free() → 段错误；
        //  2) GetInputTypeInfo(...).GetTensorTypeAndShapeInfo() 返回的对象指向
        //     TypeInfo 临时量的内部数据，**必须当场取完 GetShape()**，
        //     存下来延迟调用就是 use-after-free（表现为 std::length_error）。
        Ort::AllocatorWithDefaultOptions allocator;
        const std::string n0 = session_.GetInputNameAllocated(0, allocator).get();
        const std::string n1 = session_.GetInputNameAllocated(1, allocator).get();
        const std::string no = session_.GetOutputNameAllocated(0, allocator).get();
        const auto s0 = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        const auto s1 = session_.GetInputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape();
        const auto so = session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        const bool ok = s0.size() == 2 && s1.size() == 2 && so.size() == 2 &&
                        s0[1] == obs_dim_ && s1[1] == history_obs_dim_ && so[1] == action_dim_;
        if (!ok) {
            std::cerr << "[M20PiperPolicyRunner] ONNX 形状与 policy_layout.json 不一致："
                      << " obs=" << (s0.size() == 2 ? s0[1] : -1)
                      << " history=" << (s1.size() == 2 ? s1[1] : -1)
                      << " action=" << (so.size() == 2 ? so[1] : -1)
                      << "；layout 说 " << obs_dim_ << " / " << history_obs_dim_
                      << " / " << action_dim_ << std::endl;
            throw std::runtime_error("policy/layout dim mismatch");
        }
        // 名字也要对上（顺序搞反 = 静默算错）
        if (n0 != input_names_[0] || n1 != input_names_[1] || no != output_names_[0]) {
            std::cerr << "[M20PiperPolicyRunner] ONNX 输入输出名与预期不同：收到 ["
                      << n0 << ", " << n1 << "] -> " << no << "；预期 ["
                      << input_names_[0] << ", " << input_names_[1] << "] -> "
                      << output_names_[0] << std::endl;
            throw std::runtime_error("policy I/O name mismatch");
        }
        obs_shape_ = {1, obs_dim_};
        history_shape_ = {1, history_obs_dim_};
    }

    // ---------------------------------------------------------------
    // 关节表（名称是唯一来源，索引全部按名字算出来）
    // ---------------------------------------------------------------
    static const std::vector<std::string>& RobotOrder() {
        // 本仓库 robot_interface / MuJoCo qpos 的顺序（每腿 hipx/hipy/knee/wheel）
        static const std::vector<std::string> v = {
            "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint", "fl_wheel_joint",
            "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint", "fr_wheel_joint",
            "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint", "hl_wheel_joint",
            "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint", "hr_wheel_joint",
            "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6",
            "gripper_joint1", "gripper_joint2"};
        return v;
    }

    static const std::vector<std::string>& NativeOrder() {
        // articulation 原生序（观测 joint_pos/joint_vel 的列序）
        // 来源：训练侧 probe_deploy_layout.py 在 M20_Piper_own 上的实测（2026-09-20）
        static const std::vector<std::string> v = {
            "fl_hipx_joint", "fr_hipx_joint", "hl_hipx_joint", "hr_hipx_joint",
            "arm_joint1",
            "fl_hipy_joint", "fr_hipy_joint", "hl_hipy_joint", "hr_hipy_joint",
            "arm_joint2",
            "fl_knee_joint", "fr_knee_joint", "hl_knee_joint", "hr_knee_joint",
            "arm_joint3",
            "fl_wheel_joint", "fr_wheel_joint", "hl_wheel_joint", "hr_wheel_joint",
            "arm_joint4", "arm_joint5", "arm_joint6",
            "gripper_joint1", "gripper_joint2"};
        return v;
    }

    static const std::vector<std::string>& ActionOrder() {
        static const std::vector<std::string> v = {
            "fl_hipx_joint", "fl_hipy_joint", "fl_knee_joint",
            "fr_hipx_joint", "fr_hipy_joint", "fr_knee_joint",
            "hl_hipx_joint", "hl_hipy_joint", "hl_knee_joint",
            "hr_hipx_joint", "hr_hipy_joint", "hr_knee_joint",
            "fl_wheel_joint", "fr_wheel_joint", "hl_wheel_joint", "hr_wheel_joint"};
        return v;
    }

    static std::vector<int> Permutation(const std::vector<std::string>& from,
                                        const std::vector<std::string>& to) {
        std::unordered_map<std::string, int> idx;
        for (size_t i = 0; i < from.size(); ++i) idx[from[i]] = static_cast<int>(i);
        std::vector<int> perm;
        perm.reserve(to.size());
        for (const auto &name : to) {
            auto it = idx.find(name);
            perm.push_back(it != idx.end() ? it->second : -1);
        }
        return perm;
    }

    void InitTopology() {
        robot_order_ = RobotOrder();
        native_order_ = NativeOrder();
        action_order_ = ActionOrder();
        if (native_order_.size() != static_cast<size_t>(kRobotDof)) {
            throw std::runtime_error("native joint order size != 24");
        }
        robot2native_idx_ = Permutation(robot_order_, native_order_);
        action2robot_idx_ = Permutation(robot_order_, action_order_);
        for (int i : robot2native_idx_) {
            if (i < 0) throw std::runtime_error("native joint order has unknown joint");
        }
        for (int i : action2robot_idx_) {
            if (i < 0) throw std::runtime_error("action joint order has unknown joint");
        }
    }

    void InitDefaults() {
        // 训练 `init_state.joint_pos`（= M20_Piper_own USD 默认姿态）
        default_robot_.resize(kRobotDof);
        default_robot_ << 0.0, -0.6, 1.0, 0.0,
                          0.0, -0.6, 1.0, 0.0,
                          0.0,  0.6, -1.0, 0.0,
                          0.0,  0.6, -1.0, 0.0,
                          0.0, 0.5, -0.5, 0.0, 0.0, 0.0,
                          0.0, 0.0;
        default_native_.resize(kRobotDof);
        for (int i = 0; i < kRobotDof; ++i) {
            default_native_(i) = default_robot_(robot2native_idx_[i]);
        }

        action_default_pos_.resize(kLegDof);
        for (int i = 0; i < kLegDof; ++i) {
            action_default_pos_(i) = default_robot_(action2robot_idx_[i]);
        }

        action_scale_.assign(kLegDof + kWheelDof, 0.25f);
        for (int i = 0; i < kLegDof; ++i) {
            if (action_order_[i].find("hipx") != std::string::npos) action_scale_[i] = 0.125f;
        }
        for (int i = kLegDof; i < kLegDof + kWheelDof; ++i) action_scale_[i] = 5.0f;

        // 训练执行器：腿 80/2；轮 kp=0 / kd=0.6（速度伺服）
        kp_.setZero(kRobotDof);
        kd_.setZero(kRobotDof);
        for (int r = 0; r < kRobotDof; ++r) {
            const std::string &n = robot_order_[r];
            if (n.find("_wheel_joint") != std::string::npos) {
                kp_(r) = 0.0f;  kd_(r) = 0.6f;
            } else if (n.compare(0, 3, "arm") == 0 || n.compare(0, 7, "gripper") == 0) {
                kp_(r) = 0.0f;  kd_(r) = 0.0f;   // 臂由 arm_controller 走独立话题
            } else {
                kp_(r) = 80.0f; kd_(r) = 2.0f;
            }
        }
    }

    VecXf OnnxInfer(const VecXf &obs, const VecXf &history) {
        Ort::Value t_obs = Ort::Value::CreateTensor<float>(
            memory_info, const_cast<float*>(obs.data()), obs.size(),
            obs_shape_.data(), obs_shape_.size());
        Ort::Value t_hist = Ort::Value::CreateTensor<float>(
            memory_info, const_cast<float*>(history.data()), history.size(),
            history_shape_.data(), history_shape_.size());
        std::vector<Ort::Value> inputs;
        inputs.emplace_back(std::move(t_obs));
        inputs.emplace_back(std::move(t_hist));
        auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_.data(),
                                    inputs.data(), 2, output_names_.data(), 1);
        float *a = outputs[0].GetTensorMutableData<float>();
        return Eigen::Map<Eigen::VectorXf>(a, action_dim_);
    }

    static Vec3f Clip3(const Vec3f &v, float lo, float hi) {
        return Vec3f(v.cwiseMin(hi).cwiseMax(lo));
    }

public:
    RobotAction getRobotAction(const RobotBasicState &ro, const UserCommand &uc) override {
        // ---- 观测 1/3：状态 ----
        const Vec3f base_omega_raw = ro.base_omega;
        const Vec3f base_omega = Clip3(base_omega_raw, -100.f, 100.f) * omega_scale_;
        const Vec3f gravity_dir = ro.base_rot_mat.inverse() * Vec3f(0., 0., -1.);
        const Vec3f projected_gravity = Clip3(gravity_dir, -100.f, 100.f);
        const Vec3f vel_cmd = Clip3(
            Vec3f(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale),
            -100.f, 100.f);

        // ---- 观测 2/3：24 个关节（原生序），joint_pos 把 4 个轮子列置零 ----
        for (int i = 0; i < kJointObsDof; ++i) {
            const int robot_idx = NativeToRobot(i);
            joint_pos_obs_(i) = ClipNumberF(ro.joint_pos(robot_idx) - default_native_(i),
                                            -100.f, 100.f);
            joint_vel_obs_(i) = ClipNumberF(ro.joint_vel(robot_idx), -100.f, 100.f) *
                                dof_vel_scale_;
        }
        for (int i = 0; i < kJointObsDof; ++i) {
            if (native_order_[i].find("_wheel_joint") != std::string::npos) {
                joint_pos_obs_(i) = 0.0f;   // 训练：joint_pos_rel_without_wheel
            }
        }

        VecXf ee_goal(kEeGoalDim);
        ee_goal << uc.ee_goal_pos[0], uc.ee_goal_pos[1], uc.ee_goal_pos[2],
                   uc.ee_goal_quat[0], uc.ee_goal_quat[1], uc.ee_goal_quat[2],
                   uc.ee_goal_quat[3];
        ee_goal = ee_goal.cwiseMin(ee_goal_clip_).cwiseMax(-ee_goal_clip_);
        VecXf body_pose(kBodyPoseDim);
        body_pose << uc.body_height, uc.body_pitch, uc.body_roll;

        // ---- 观测 3/3：上一步动作 ----
        current_observation_ << base_omega, projected_gravity, vel_cmd,
                                joint_pos_obs_, joint_vel_obs_, last_action_obs_,
                                ee_goal, body_pose;

        // ---- history 单步（70）：原始值，含轮子的 24 维 joint_pos ----
        history_step_.head(3) = base_omega_raw.cwiseMin(100.f).cwiseMax(-100.f);
        history_step_.segment(3, 3) = projected_gravity;
        for (int i = 0; i < kJointObsDof; ++i) {
            const int robot_idx = NativeToRobot(i);
            history_step_(6 + i) = ClipNumberF(ro.joint_pos(robot_idx) - default_native_(i),
                                               -100.f, 100.f);
            history_step_(6 + kJointObsDof + i) =
                    ClipNumberF(ro.joint_vel(robot_idx), -100.f, 100.f);
        }
        history_step_.tail(action_dim_) = last_action_obs_;

        if (!history_initialized_) {
            for (int s = 0; s < history_steps_; ++s) {
                history_obs_.segment(s * history_step_dim_, history_step_dim_) = history_step_;
            }
            history_initialized_ = true;
        } else {
            history_obs_.head(history_obs_dim_ - history_step_dim_) =
                    history_obs_.tail(history_obs_dim_ - history_step_dim_);
            history_obs_.tail(history_step_dim_) = history_step_;
        }

        current_action_ = OnnxInfer(current_observation_, history_obs_);

        // ---- 安全限幅（默认 = 训练 clip_actions，M20_ACTION_CLIP 可收紧）----
        current_action_ = current_action_.cwiseMin(action_clip_).cwiseMax(-action_clip_);

        // 喂回观测的必须是**实际下发**的动作（训练里 actions 观测就是同一个值）
        last_action_obs_ = current_action_;

        // ---- 动作 → 关节目标（MJCF 序 24 行；臂/夹爪保持默认且零增益）----
        robot_goal_ = default_robot_;
        robot_action_.goal_joint_vel.setZero();
        for (int a = 0; a < kLegDof + kWheelDof; ++a) {
            const int r = action2robot_idx_[a];
            if (a < kLegDof) {
                robot_goal_(r) += action_scale_[a] * current_action_(a);
            } else {
                robot_action_.goal_joint_vel(r) = action_scale_[a] * current_action_(a);
            }
        }
        robot_action_.goal_joint_pos = robot_goal_;

        if (const char *e = std::getenv("M20_PIPER_DEBUG"); e && run_cnt_ < 300) {
            std::ofstream dbg("policy_debug.txt", std::ios::app);
            if (dbg.is_open()) {
                dbg << "tick " << run_cnt_ << "\nobs\n";
                for (int i = 0; i < obs_dim_; ++i) dbg << current_observation_(i) << " ";
                dbg << "\naction\n";
                for (int i = 0; i < action_dim_; ++i) dbg << current_action_(i) << " ";
                dbg << "\n";
            }
        }

        ++run_cnt_;
        return robot_action_;
    }

private:
    int NativeToRobot(int native_idx) const {
        // native_order_[i] 在 robot_order_ 里的下标
        static std::unordered_map<std::string, int> idx;
        if (idx.empty()) {
            for (int r = 0; r < kRobotDof; ++r) idx[robot_order_[r]] = r;
        }
        return idx.at(native_order_[native_idx]);
    }

    static float ClipNumberF(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
};
