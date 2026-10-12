"""纯 numpy 的 `scipy.spatial.transform.Rotation` 兼容子集（本仓库自加，非上游）。

**为什么需要**：真机（M20 上位机）**不能联网**，装不上 scipy；而 scipy 只被用来做
四元数/欧拉角换算（`from_quat` / `from_euler` / `as_euler` / `as_rotvec` / `inv` / `*`），
这些用 numpy 十几行就能做完。于是这里实现同一套 API 的**最小子集**：

    from scipy.spatial.transform import Rotation as R   # 有 scipy 就用
    ...
    try:
        from scipy.spatial.transform import Rotation as R
    except ImportError:                                  # 没有就退到本文件
        from xlevr._rotation_compat import Rotation as R

约定与 scipy 对齐（并用脚本逐项数值对照过，见 `docs/review/DEFECT_LOG_zh.md` DEF-043）：

* 四元数一律 **xyzw**（scipy 的约定）；
* `from_euler("xyz", [a,b,c])`（**小写 = 外旋/extrinsic**，绕固定轴依次作用）
  ⇒ `R = Rz(c)·Ry(b)·Rx(a)`；
* `from_euler("XYZ", [a,b,c])`（**大写 = 内旋/intrinsic**）⇒ `R = Rx(a)·Ry(b)·Rz(c)`；
* `A * B` = 先做 B 再做 A（与 scipy 同）；
* `as_rotvec()` = 轴角向量（轴 × 角度）。

只用到了上面这些功能；遇到没实现的组合会**直接报错**（不做静默近似）。
"""

from __future__ import annotations

import math

import numpy as np

_AXIS = {"x": np.array([1.0, 0.0, 0.0]),
         "y": np.array([0.0, 1.0, 0.0]),
         "z": np.array([0.0, 0.0, 1.0])}


def _normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64).reshape(4)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])   # 退化为单位旋转（scipy 亦然）
    return q / n


def _mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """xyzw 四元数乘法：q1 ∘ q2（先作用 q2）。"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def _from_axis_angle(axis: str, angle: float) -> np.ndarray:
    a = _AXIS[axis.lower()]
    h = 0.5 * float(angle)
    s = math.sin(h)
    return np.array([a[0] * s, a[1] * s, a[2] * s, math.cos(h)])


def _to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Rotation:
    """scipy `Rotation` 的最小兼容子集（内部存 xyzw 四元数）。"""

    __slots__ = ("_q",)

    def __init__(self, quat_xyzw) -> None:
        self._q = _normalize(quat_xyzw)

    # ---------------- 构造 ----------------
    @classmethod
    def from_quat(cls, q_xyzw) -> "Rotation":
        return cls(q_xyzw)

    @classmethod
    def from_euler(cls, seq: str, angles, degrees: bool = False) -> "Rotation":
        seq = str(seq)
        if len(seq) == 1:
            if isinstance(angles, (int, float)):
                vals = [float(angles)]
            else:
                vals = [float(v) for v in np.asarray(angles).reshape(-1)]
            if len(vals) != 1:
                raise ValueError("单轴 from_euler 需要 1 个角度")
            a = math.radians(vals[0]) if degrees else vals[0]
            return cls(_from_axis_angle(seq, a))
        if len(seq) != 3:
            raise NotImplementedError("只支持单轴或三轴序列，收到 %r" % seq)
        vals = [float(v) for v in np.asarray(angles).reshape(-1)]
        if len(vals) != 3:
            raise ValueError("三轴 from_euler 需要 3 个角度")
        if degrees:
            vals = [math.radians(v) for v in vals]
        qs = [_from_axis_angle(ax, ang) for ax, ang in zip(seq, vals)]
        intrinsic = seq.isupper()
        # 外旋(小写)：R = Rz(c)·Ry(b)·Rx(a)；内旋(大写)：R = Rx(a)·Ry(b)·Rz(c)
        out = qs[0]
        if intrinsic:
            for q in qs[1:]:
                out = _mul(out, q)
        else:
            out = qs[0]
            for q in qs[1:]:
                out = _mul(q, out)      # 左侧累乘 ⇒ 最后得到 Rz·Ry·Rx
        return cls(out)

    # ---------------- 导出 ----------------
    def as_quat(self) -> np.ndarray:
        return self._q.copy()

    def as_matrix(self) -> np.ndarray:
        return _to_matrix(self._q)

    def as_rotvec(self) -> np.ndarray:
        x, y, z, w = self._q
        if w < 0.0:                     # 取短弧（scipy 也是这么做的）
            x, y, z, w = -x, -y, -z, -w
        v = np.array([x, y, z])
        s = float(np.linalg.norm(v))
        if s < 1e-12:
            return np.zeros(3)
        angle = 2.0 * math.atan2(s, w)
        return v / s * angle

    def as_euler(self, seq: str, degrees: bool = False) -> np.ndarray:
        seq = str(seq)
        if len(seq) != 3:
            raise NotImplementedError("as_euler 只实现了三轴序列，收到 %r" % seq)
        R = _to_matrix(self._q)
        if seq.isupper():
            # 内旋 XYZ：R = Rx(a)·Ry(b)·Rz(c)
            b = math.asin(max(-1.0, min(1.0, R[0, 2])))
            if abs(R[0, 2]) < 1.0 - 1e-9:
                a = math.atan2(-R[1, 2], R[2, 2])
                c = math.atan2(-R[0, 1], R[0, 0])
            else:                        # 万向锁：按 scipy 的约定取 a、c=0
                a = math.atan2(R[2, 1], R[1, 1])
                c = 0.0
        else:
            # 外旋 xyz：R = Rz(c)·Ry(b)·Rx(a)
            b = math.asin(max(-1.0, min(1.0, -R[2, 0])))
            if abs(R[2, 0]) < 1.0 - 1e-9:
                a = math.atan2(R[2, 1], R[2, 2])
                c = math.atan2(R[1, 0], R[0, 0])
            else:
                a = math.atan2(-R[1, 2], R[1, 1])
                c = 0.0
        out = np.array([a, b, c])
        return np.degrees(out) if degrees else out

    # ---------------- 运算 ----------------
    def inv(self) -> "Rotation":
        x, y, z, w = self._q
        return Rotation(np.array([-x, -y, -z, w]))

    def __mul__(self, other: "Rotation") -> "Rotation":
        if not isinstance(other, Rotation):
            raise TypeError("只能和 Rotation 相乘")
        return Rotation(_mul(self._q, other._q))

    def __repr__(self) -> str:
        return "Rotation(quat_xyzw=%s)" % np.round(self._q, 6).tolist()
