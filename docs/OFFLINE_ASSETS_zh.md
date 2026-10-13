# 离线资源清单：部署前要**下载**哪些东西

**文档职责**：把所有"仓库里不自带、必须先在**有外网**的机器上下载再搬过去"的文件/包
集中列一处——包含放哪儿、怎么下、怎么自检、缺了会出现什么症状。
（每条的外部依赖一旦有变化，同步更新本文件 + `review/DEFECT_LOG_zh.md` 对应条目。）

**为什么需要这份清单**：本项目链路上一共四类机器，**全都没有外网**

| 机器 | 外网 | 说明 |
|---|---|---|
| 你的笔记本 / 任意联网机器 | ✅ | **只有在这里才能下载**（下载后 scp 到别处） |
| 开发机（仿真宿主机 + 容器 `m20_piper_ros`） | ❌ | 跑 sim2sim；**拉不到公网 CDN / pip 源** |
| M20 上位机 `10.21.31.103`（WiFi 侧 `10.21.41.1`） | ❌ | 跑真机部署 + VR 节点（ROS 2 Foxy / Python 3.8 / aarch64） |
| Jetson `10.21.31.201` | ❌ | 跑 `agx_arm_ctrl` |
| VR 头显 | ❌ | **什么都不用下载**：页面和引擎全部由上位机提供（见第 2.1 条） |

---

## 1. 清单总表（截至 `2026-10-13`）

| # | 要下载的东西 | 给谁用 | 放到哪 | 大小 | 下载方式 | 缺了会怎样 |
|---|---|---|---|---|---|---|
| 1 | **`aframe.min.js` 1.7.1** | 头显页面（`web-ui`） | 仓库 `src/M20_sdk_deploy/third_party/XLeVR/web-ui/` | 1247 KB | `bash src/M20_sdk_deploy/scripts/fetch_aframe.sh`（联网机器） | 头显里页面"卡住"：只剩中间一张 `VR Teleoperation` 卡片，**点哪都没反应**（`DEF-058`） |
| 2 | **`PyYAML-6.0.3-cp38-cp38-manylinux2014_aarch64….whl`** | 上位机 `xlevr/config.py` | 上位机 `~/`（或仓库 `tmp/wheels/`） | 777 KB | 见第 2.2 条（`pip download`） | `vr_teleop_node.py` 起不来：`ModuleNotFoundError: No module named 'yaml'` |
| 3 | **`websockets-13.1-cp38-cp38-manylinux_2_17_aarch64….whl`** | 上位机 `xlevr/inputs/vr_ws_server.py` | 同上 | 161 KB | 同上 | 起不来：`No module named 'websockets'` |
| 4 | **`pylibs_yaml_websockets_cp38.tar.gz`** | 上位机**没有 pip** 时的兜底（两个包的纯 Python 源码） | 同上 | 165 KB | 已备好（同 2.2 条的文件说明） | 同上（改用解压 + `PYTHONPATH`） |

> 2/3/4 三份**这个仓库里已经备好**在 `tmp/wheels/` —— 但 `tmp/` 被 `.gitignore` 忽略，
> **换机器 / 重新 clone 之后要重下**（重启不丢；`/tmp` 下的那份会随重启丢，别放那）。

### 明确**不用**下载的（省得白折腾）

| 东西 | 结论 | 原因 |
|---|---|---|
| `scipy` | ❌ 不需要 | 2026-10-10 起 VR 侧旋转换算改用纯 numpy 实现（`third_party/XLeVR/xlevr/_rotation_compat.py`）；有 scipy 时仍优先用它，但**没有也能跑** |
| `numpy` | ❌ 不需要 | 上位机自带 `numpy 1.17.4`（Ubuntu 20.04 apt 版），VR/IK 只用基础接口 |
| `pynput` | ❌ 不需要 | 只在上游 `XLeVR/requirements.txt` / 独立的 `vr_monitor.py` 里用；我们的部署路径（`vr_teleop_node.py` → `vr_teleop_device.py`）不 import 它 |
| `torch` | ❌ 不需要 | 只在训练/仿真侧；部署侧策略走 ONNX Runtime（C++，仓库已自带 `third_party/onnxruntime`） |
| ROS 2（上位机 Foxy / 容器 Humble） | ❌ 不需要 | 上位机与容器 `m20_piper_ros` 里**已经装好**；只有"在**新**机器上重建开发环境"时才要联网（拉 ROS 基础镜像 + `pip install mujoco` 等），那属于重建环境、不是本部署流程的一步 |

---

## 2. 逐项细节

### 2.1 头显页面的 A-Frame（`aframe.min.js`，1247 KB）

头显打开的 `https://<上位机IP>:8443` 是**纯前端页面**，VR 场景由 A-Frame 渲染；
这个文件**必须离线自带**（原来写的是 `https://aframe.io/...` 的公网 CDN 地址，
内网拉不到 ⇒ 就是 `DEF-058` 那个"卡住"现象）。

```bash
# 【联网机器】仓库根目录：自动从 6 个镜像里挑一个能下的
bash src/M20_sdk_deploy/scripts/fetch_aframe.sh
bash src/M20_sdk_deploy/scripts/fetch_aframe.sh --check     # 自检：够大 / 不是 HTML 错误页 / 含 AFRAME
```

镜像顺序（脚本内置，也可 `--url <任意地址>` 指定）：
`aframe.io/releases/1.7.1/` → `cdn.jsdelivr.net/npm/aframe@1.7.1/dist/aframe-master.min.js`
→ `unpkg.com/aframe@1.7.1/dist/aframe-master.min.js` → `cdnjs` → `cdn.bootcdn.net` → `cdn.staticfile.org`。

**手工两条路**（脚本跑不了时）：

* 浏览器打开 <https://aframe.io/releases/1.7.1/aframe.min.js> → 另存为 `aframe.min.js`；
* 或者有 node 的机器：`npm pack aframe@1.7.1` → 解压 `aframe-1.7.1.tgz` →
  取 `package/dist/aframe-master.min.js` 改名成 `aframe.min.js`。

**放到哪、怎么验**：

```bash
# 开发机（sim2sim 直接读仓库里这份）：放好即可
ls -l src/M20_sdk_deploy/third_party/XLeVR/web-ui/aframe.min.js

# 上位机：跟着同步脚本过去（脚本会打印 "A-Frame : 已就位"）或单文件传
bash src/M20_sdk_deploy/scripts/sync_to_robot.sh
# scp src/M20_sdk_deploy/third_party/XLeVR/web-ui/aframe.min.js \
#     user@10.21.31.103:~/M20_sdk_deploy/src/M20_sdk_deploy/third_party/XLeVR/web-ui/

# 服务端有没有在发这个文件（200 = 就位，404 = 缺）
curl -k -s -o /dev/null -w "%{http_code}\n" https://10.21.31.103:8443/aframe.min.js
```

三个"一眼确认"信号：① `vr_teleop_node.py` 启动横幅里的
`头显页面: 已自带离线 A-Frame（xxxx KB）`；② 头显页面**顶部红条消失**、
出现 `Enter VR` / `Start Controller Tracking` 按钮；③ 上面的 `curl` 返回 `200`。

> **建议把这个文件提交进 git**（和 `web-ui/` 其它文件一样属于自带资源），
> 否则每换一台机器、每次重新 clone 都要重跑一次 `fetch_aframe.sh`。

### 2.2 上位机的 Python 依赖：`PyYAML` + `websockets`（aarch64 / cp38）

部署路径上**真正 import 的只有这两个**（`grep -rn "import yaml\|import websockets" src/M20_sdk_deploy`）。

```bash
# 【联网机器】按上位机的 Python 版本/架构下 wheel（上位机 = aarch64 + Python 3.8）
python3 -m pip download PyYAML websockets \
  --only-binary=:all: --platform manylinux2014_aarch64 \
  --python-version 38 --implementation cp --abi cp38 -d tmp/wheels

# 【搬到上位机】
scp tmp/wheels/*.whl tmp/wheels/*.tar.gz user@10.21.31.103:~/

# 【上位机】方式 A：有 pip，离线安装（--no-index = 不上网找包）
python3 -m pip install --no-index --no-deps ~/PyYAML-*.whl ~/websockets-*.whl

# 【上位机】方式 B：没有 pip / wheel 装不上（纯 Python 兜底，起节点前挂 PYTHONPATH）
mkdir -p ~/pylibs && tar xzf ~/pylibs_yaml_websockets_cp38.tar.gz -C ~/pylibs
export PYTHONPATH=~/pylibs:$PYTHONPATH

# 【上位机】自检（两条都过才算好）
python3 -V                                                   # 应为 3.8
python3 -c "import numpy, yaml, websockets; print('deps ok')"
```

**这个仓库里现成的三份**（`tmp/wheels/`，`2026-10-12` 下的，可直接用）：

| 文件（完整名） | 大小 |
|---|---|
| `PyYAML-6.0.3-cp38-cp38-manylinux2014_aarch64.manylinux_2_17_aarch64.manylinux_2_28_aarch64.whl` | 777 KB |
| `websockets-13.1-cp38-cp38-manylinux_2_17_aarch64.manylinux2014_aarch64.whl` | 161 KB |
| `pylibs_yaml_websockets_cp38.tar.gz`（两个包的纯 Python 源码，无 pip 时用） | 165 KB |

> 上位机 `python3 -V` **不是 3.8** 时，按实际版本重下（把 `--python-version 38` 改成 3x、
> `--abi cp38` 改成对应值）。纯 Python 的偷懒替代：在开发机容器里
> `python3 -c "import yaml,websockets;print(yaml.__file__, websockets.__file__)"`，
> 直接把这两个目录拷到上位机挂 `PYTHONPATH`——注意**版本要支持上位机的 Python**
> （例如 websockets ≥14 要求 Python ≥3.9，别拷错了）。

细节与"为什么不用 scipy"的来龙去脉见 `DEPLOY_REAL_VR_zh.md` §1.5 与
`review/DEFECT_LOG_zh.md` 的 `DEF-056`（原 DEF-043）。

---

## 3. 现场搬运的三条命令（复制用）

```bash
# ① 联网机器：下载（A-Frame 1247 KB + 两个 Python 包，合计约 2.3 MB）
bash src/M20_sdk_deploy/scripts/fetch_aframe.sh
bash src/M20_sdk_deploy/scripts/fetch_aframe.sh --check
python3 -m pip download PyYAML websockets --only-binary=:all: \
  --platform manylinux2014_aarch64 --python-version 38 --implementation cp --abi cp38 -d tmp/wheels

# ② 同步到上位机（A-Frame 随代码走；wheel 单独 scp 到家目录）
bash src/M20_sdk_deploy/scripts/sync_to_robot.sh
scp tmp/wheels/*.whl tmp/wheels/*.tar.gz user@10.21.31.103:~/

# ③ 上位机上一次性验完
python3 -c "import numpy, yaml, websockets; print('deps ok')"
curl -k -s -o /dev/null -w "aframe.min.js -> %{http_code}\n" https://127.0.0.1:8443/aframe.min.js
```

> 密码是**一个单引号** `'`；嫌每次都输就 `ssh-copy-id user@10.21.31.103`。

---

## 4. 换机器 / 重新 clone 之后要重做什么

| 东西 | 会不会跟着 git 走 | 换机器后 |
|---|---|---|
| `web-ui/aframe.min.js` | ❌ 不在 git 里（除非你提交它） | 重跑 `scripts/fetch_aframe.sh` |
| `tmp/wheels/*.whl`、`*.tar.gz` | ❌ `tmp/` 被 gitignore | 按 §2.2 重下（或从旧机器拷） |
| 证书 `cert.pem` / `key.pem`、仿真 mesh、onnxruntime、策略 `policy/*/` | ✅ 在 git 里 | 无需操作 |

---

## 5. 相关的缺陷/条目

| 条目 | 关系 |
|---|---|
| `DEF-058` | 头显页面依赖公网 CDN 拉 A-Frame ⇒ 内网里页面"卡住"、点哪都没反应（已修：离线自带 + 页内红条 + `fetch_aframe.sh`） |
| `DEF-057` | VR 节点打印 `https://localhost:8443`（离线枚举网卡 + `M20_VR_IP`） |
| `DEF-056` | 上位机不能联网 ⇒ VR 节点因 scipy 缺失起不来（已改：纯 numpy 兜底，scipy 不再是必需品） |
