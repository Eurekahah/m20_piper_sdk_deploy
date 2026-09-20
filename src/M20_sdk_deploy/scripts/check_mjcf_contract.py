#!/usr/bin/env python3
"""L3' 参数对照：把 MuJoCo 侧的 MJCF 与训练侧的权威配置逐项比对。

训练侧的权威文件（在训练仓库 `loco-manip-unified-rl-agent` 里）：
  * `logs/rsl_rl/history_adaptation/2026-09-20_00-50-31/params/env.yaml`
    —— init_state 默认角、执行器 stiffness/damping/armature/effort/velocity 限幅、
       observations 的 scale/clip、sim.dt、decimation
  * 本仓库 `docs/sim2sim_layout_contract_zh.md` —— 人工核对过的接口契约

本脚本只读 MJCF，输出"训练侧期望 vs MJCF 实际"，并给出 PASS/FAIL/UNKNOWN：
  PASS    两边一致（容差见各条）
  FAIL    不一致（必须处理，否则 sim2sim 与训练不是同一个机器人）
  UNKNOWN 训练侧没有对应的可核对项，需要人工判定

用法::

  python3 src/M20_sdk_deploy/scripts/check_mjcf_contract.py [M20_Piper_own.xml]

退出码：0 = 没有 FAIL，1 = 有 FAIL（可以接进 tests/run_all.sh）。
"""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

DEFAULT_XML = (Path(__file__).resolve().parent / ".." / "M20_Piper_description"
               / "mjcf" / "M20_Piper_own.xml")

results: list[tuple[str, str, str]] = []          # (状态, 项, 说明)


def report(ok: bool | None, item: str, detail: str) -> None:
    status = "UNKNOWN" if ok is None else ("PASS" if ok else "FAIL")
    results.append((status, item, detail))


# ---------------------------------------------------------------------------
# 训练侧期望值（来源见文件头；改训练配置时同步改这里）
# ---------------------------------------------------------------------------
EXPECT_TIMESTEP_NOTE = "训练 sim.dt=0.005（本仓库用 0.2 ms 子步 × 5；控制周期才是矛盾点）"

# 默认关节角（env.yaml -> scene.robot.init_state.joint_pos）
EXPECT_DEFAULT = {
    "fl_hipx_joint": 0.0, "fr_hipx_joint": 0.0, "hl_hipx_joint": 0.0, "hr_hipx_joint": 0.0,
    "fl_hipy_joint": -0.6, "fr_hipy_joint": -0.6, "hl_hipy_joint": 0.6, "hr_hipy_joint": 0.6,
    "fl_knee_joint": 1.0, "fr_knee_joint": 1.0, "hl_knee_joint": -1.0, "hr_knee_joint": -1.0,
    "fl_wheel_joint": 0.0, "fr_wheel_joint": 0.0, "hl_wheel_joint": 0.0, "hr_wheel_joint": 0.0,
    "arm_joint1": 0.0, "arm_joint2": 0.5, "arm_joint3": -0.5,
    "arm_joint4": 0.0, "arm_joint5": 0.0, "arm_joint6": 0.0,
    "gripper_joint1": 0.0, "gripper_joint2": 0.0,
}

# 关节硬限位。权威来源是训练仓库的 URDF（docs/deploy_sim2sim_sim2real_zh.md 也这么写）：
#   deep_robotics_model/M20_Piper_own/urdf/M20_Piper_own.urdf
# 注意：hipx 的限位是**左右镜像**（fl/hl = [-0.436, 0.611]，fr/hr = [-0.611, 0.436]），
# 不是"前腿/后腿"镜像 —— 训练侧部署文档第 3 节那张表这一行写错了（见 DEF-011）。
EXPECT_RANGE = {
    "fl_hipx_joint": (-0.436, 0.611), "fr_hipx_joint": (-0.611, 0.436),
    "hl_hipx_joint": (-0.436, 0.611), "hr_hipx_joint": (-0.611, 0.436),
    "fl_hipy_joint": (-2.583, 2.286), "fr_hipy_joint": (-2.583, 2.286),
    "hl_hipy_joint": (-2.286, 2.583), "hr_hipy_joint": (-2.286, 2.583),
    "fl_knee_joint": (-2.792, 2.809), "fr_knee_joint": (-2.792, 2.809),
    "hl_knee_joint": (-2.809, 2.792), "hr_knee_joint": (-2.809, 2.792),
    "arm_joint1": (-2.618, 2.168), "arm_joint2": (0.0, 3.14), "arm_joint3": (-2.967, 0.0),
    "arm_joint4": (-1.745, 1.745), "arm_joint5": (-1.22, 1.22), "arm_joint6": (-2.094, 2.094),
    "gripper_joint1": (0.0, 0.035), "gripper_joint2": (-0.035, 0.0),
}

# 执行器（env.yaml -> scene.robot.actuators）
EXPECT_ACTUATOR = {
    # 组名 -> (力矩限幅, armature, 速度限幅 rad/s, stiffness, damping)
    "leg": (76.4, 0.0, 22.4, 80.0, 2.0),
    "wheel": (21.6, 0.00243216, 79.3, 0.0, 0.6),
    "arm": (100.0, 0.01, 3.0, 300.0, 20.0),
    "gripper": (10.0, 0.01, 1.0, 4000.0, 200.0),
}

WHEEL_RADIUS = 0.09
# 默认姿态下 gripper_base 在 base_link 系里的位姿（训练侧探针实测值）
EXPECT_GRIPPER_BASE_POS = np.array([0.3492, 0.0, 0.4327])
EXPECT_GRIPPER_BASE_QUAT_WXYZ = np.array([-0.7373, 0.0, -0.6756, 0.0])


def actuator_group(name: str) -> str:
    if "wheel" in name:
        return "wheel"
    if "gripper" in name:
        return "gripper"
    if name.startswith("arm_joint"):
        return "arm"
    return "leg"


def main() -> int:
    xml_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_XML
    xml_path = xml_path.resolve()
    if not xml_path.is_file():
        print(f"[mjcf-contract] MJCF not found: {xml_path}")
        return 1
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    print(f"[mjcf-contract] model = {xml_path}")
    print(f"[mjcf-contract] nq={model.nq} nu={model.nu} nbody={model.nbody} "
          f"timestep={model.opt.timestep} gravity={model.opt.gravity}")

    # ---- 1. 时间步 ----
    report(None, "timestep", f"MJCF={model.opt.timestep:g}；{EXPECT_TIMESTEP_NOTE}")

    # ---- 2. 关节名单（24 个：22 hinge + 2 slide 夹爪）----
    movable_types = (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)
    joint_names = [model.joint(i).name for i in range(model.njnt)]
    movable_names = [joint_names[i] for i in range(model.njnt)
                     if model.jnt_type[i] in movable_types]
    print(f"[mjcf-contract] MJCF movable joints ({len(movable_names)}): {movable_names}")
    expected_joint_set = set(EXPECT_DEFAULT)
    missing = expected_joint_set - set(movable_names)
    extra = set(movable_names) - expected_joint_set
    report(not missing and not extra, "joint set (24)",
           f"missing={sorted(missing)} extra={sorted(extra)}")

    # ---- 3. 默认姿态：MJCF 自己不做定义，由部署侧写死 EXPECT_DEFAULT ----
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    for i in range(model.njnt):
        if model.jnt_type[i] not in movable_types:
            continue
        adr = model.jnt_qposadr[i]
        data.qpos[adr] = EXPECT_DEFAULT.get(model.joint(i).name, 0.0)
    mujoco.mj_forward(model, data)
    report(None, "default pose 由部署侧设定",
           "MJCF 的 qpos0 是『全零』，站立默认角（hipy ±0.6 / knee ∓1.0 / arm2 0.5 / "
           "arm3 -0.5）必须由部署侧写入（`mujoco_simulation_ros2.py::_set_initial_pose`、"
           "`M20PiperPolicyRunner::InitDefaults`、训练 `init_state.joint_pos` 三处必须一致）")

    # ---- 4. 关节限位 ----
    bad_range = {}
    for i in range(model.njnt):
        name = model.joint(i).name
        if model.jnt_type[i] not in movable_types or name not in EXPECT_RANGE:
            continue
        lo, hi = EXPECT_RANGE[name]
        mlo, mhi = model.jnt_range[i]
        if model.jnt_limited[i] == 0:
            bad_range[name] = f"MJCF 无限位，期望 [{lo}, {hi}]"
        elif abs(mlo - lo) > 1e-3 or abs(mhi - hi) > 1e-3:
            bad_range[name] = f"MJCF [{mlo:.3f}, {mhi:.3f}] vs 期望 [{lo}, {hi}]"
    report(not bad_range, "joint limits", "匹配" if not bad_range else f"不一致 {bad_range}")

    # ---- 5. 执行器：ctrlrange（力矩）× gear 才是力矩；armature 必须写在 <joint> 上 ----
    bad_act = {}
    for i in range(model.nu):
        name = model.actuator(i).name
        grp = actuator_group(name)
        eff, armature, _vlim, _kp, _kd = EXPECT_ACTUATOR[grp]
        gear = abs(float(model.actuator_gear[i][0])) or 1.0
        ctrlrange = model.actuator_ctrlrange[i]
        tau_lo, tau_hi = float(ctrlrange[0]) * gear, float(ctrlrange[1]) * gear
        if model.actuator_ctrllimited[i] == 0:
            bad_act[name] = f"无 ctrlrange（期望 ±{eff} N·m）"
        elif abs(tau_hi - eff) > 1e-2 or abs(tau_lo + eff) > 1e-2:
            bad_act[name] = f"力矩范围 [{tau_lo:.3f}, {tau_hi:.3f}] vs 期望 ±{eff}"
        m_arm = float(model.dof_armature[model.jnt_dofadr[model.actuator_trnid[i][0]]])
        if abs(m_arm - armature) > 1e-6:
            bad_act.setdefault(name, "")
            bad_act[name] += f"；armature={m_arm:g} vs 期望 {armature:g}"
    report(not bad_act, "actuator effort limits + armature",
           "匹配" if not bad_act else f"不一致 {bad_act}")

    # ---- 5b. armature 到底有没有生效（写在 <motor> 上会被 MuJoCo 静默忽略）----
    nonzero_armature = {model.joint(i).name: float(model.dof_armature[model.jnt_dofadr[i]])
                        for i in range(model.njnt) if model.dof_armature[model.jnt_dofadr[i]] > 0}
    report(bool(nonzero_armature), "armature 实际生效",
           f"非零 armature 的关节 = {nonzero_armature}；"
           "（MuJoCo 只认 `<joint armature=...>`，写在 `<motor>` 上会被忽略 —— 见 DEF-011）")

    # ---- 6. 默认姿态几何：gripper_base 相对 base_link ----
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper_base")
    if base_id < 0 or ee_id < 0:
        report(False, "gripper_base FK", f"body 缺失 base_id={base_id} ee_id={ee_id}")
    else:
        rel_pos = data.xpos[ee_id] - data.xpos[base_id]
        q_base = data.xquat[base_id]      # wxyz, MuJoCo 约定
        q_ee = data.xquat[ee_id]
        # 相对旋转 = q_base^-1 * q_ee
        q_rel = np.zeros(4, dtype=np.float64)
        qb_inv = np.array([q_base[0], -q_base[1], -q_base[2], -q_base[3]], dtype=np.float64)
        mujoco.mju_mulQuat(q_rel, qb_inv, q_ee)
        angle = np.degrees(2.0 * np.arccos(np.clip(abs(q_rel[0]), -1.0, 1.0)))
        d_pos = float(np.linalg.norm(rel_pos - EXPECT_GRIPPER_BASE_POS))
        # 四元数可能整体差一个负号（同一个旋转），所以只比"旋转角度差"
        expect_quat = np.array(EXPECT_GRIPPER_BASE_QUAT_WXYZ, dtype=np.float64)
        expect_inv = np.array([expect_quat[0], -expect_quat[1], -expect_quat[2], -expect_quat[3]])
        q_delta = np.zeros(4, dtype=np.float64)
        mujoco.mju_mulQuat(q_delta, expect_inv, q_rel)
        ang_diff = np.degrees(2.0 * np.arccos(np.clip(abs(q_delta[0]), -1.0, 1.0)))
        ok = d_pos < 2e-3 and ang_diff < 0.5
        report(ok, "gripper_base FK at default pose",
               f"pos={np.round(rel_pos, 4)} (期望 {EXPECT_GRIPPER_BASE_POS}, 差 {d_pos:.2e} m)；"
               f"相对旋转角={angle:.3f}°，与期望四元数的角度差={ang_diff:.3f}°")

    # ---- 7. 默认姿态下"相对足端高度"（body_pose.height 的定义）----
    # 先按仿真节点的做法把机器人放到"轮子着地"的高度：最低轮心 z = 轮半径。
    wheel_geom_ids = [i for i in range(model.ngeom)
                      if model.geom(i).name.endswith("_wheel_collision")]
    wheel_body_ids = [i for i in range(model.nbody) if model.body(i).name.endswith("_wheel")]
    if wheel_geom_ids:
        lowest = float(np.min(data.geom_xpos[wheel_geom_ids, 2]))
        data.qpos[2] += WHEEL_RADIUS - lowest
        mujoco.mj_forward(model, data)
    if wheel_body_ids:
        wheel_z = data.xpos[wheel_body_ids, 2]
        rel_height = float(data.xpos[base_id, 2] - wheel_z.mean() + WHEEL_RADIUS)
        report(None, "body_pose.height 度量自检",
               f"着地后的默认姿态 rel_height = {rel_height:.4f} m"
               f"（base z={float(data.xpos[base_id, 2]):.4f}、轮心 z={np.round(wheel_z, 4)}）；"
               "训练侧默认命令/课程 s0 锚点 = 0.513，两者的差是接触压缩量")

    # ---- 8. body / site / sensor（仿真节点依赖的名字）----
    body_names = [model.body(i).name for i in range(model.nbody)]
    for need in ("base_link", "gripper_base"):
        report(need in body_names, f"body '{need}' exists", "")
    site_names = [model.site(i).name for i in range(model.nsite)]
    for need in ("imu_site", "base_site"):
        report(need in site_names, f"site '{need}' exists", f"sites={site_names}")
    sensor_names = [model.sensor(i).name for i in range(model.nsensor)]
    report(len(sensor_names) >= 3, "sensors (framequat/accelerometer/gyro)",
           f"sensors={sensor_names}（accelerometer/gyro 无名字；仿真节点按 "
           f"sensordata[0:4]=framequat、[4:7]=acc、[7:10]=gyro 取值，共需 10 个数）")

    # ---- 输出 ----
    width = max(len(i) for _, i, _ in results)
    n_fail = 0
    print("\n" + "=" * 100)
    for status, item, detail in results:
        if status == "FAIL":
            n_fail += 1
        print(f"  [{status:7s}] {item:<{width}}  {detail}")
    print("=" * 100)
    print(f"[mjcf-contract] {len(results)} 项："
          f"{sum(1 for s, _, _ in results if s == 'PASS')} PASS / "
          f"{n_fail} FAIL / {sum(1 for s, _, _ in results if s == 'UNKNOWN')} UNKNOWN")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
