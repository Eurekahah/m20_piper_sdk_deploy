#!/usr/bin/env bash
# 把 M20_sdk_deploy 同步到真机（**在开发机上运行**，不是在容器里）。
#
# 为什么不用 `scp -r`：整包 215 MB / 4539 个文件，其中
#   * 仿真专用 mesh（M20_description/、M20_Piper_description/）= 47 MB / 2202 个 STL；
#   * 开发机专用的 onnxruntime x86 库 = 62 MB；
#   * gamepad 里自带的 fmt 示例/测试 = 3.9 MB。
# scp 每次都把这 100+ MB、几千个文件重新传一遍（慢在"文件个数"的 ssh 开销上）。
#
# 本脚本默认：
#   1) 用 rsync（只传改动过的文件 —— 第二次起通常只有几个 .py/.hpp，几 KB）；
#   2) 排除上面那些真机运行/编译都不需要的目录；
#   3) 没有 rsync 就退化成 `tar + ssh`（仍然带同样的排除）。
#
# 用法（在仓库根目录）：
#   bash src/M20_sdk_deploy/scripts/sync_to_robot.sh                # 增量同步（日常就用这个）
#   bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --dry-run      # 只列出"会传什么"
#   bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --with-meshes  # 连仿真 mesh 一起传
#   bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --full         # 不排除任何东西（等价旧 scp）
#   bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --delete       # 远端与本地完全一致（会删远端多余文件！）
#   bash src/M20_sdk_deploy/scripts/sync_to_robot.sh --build        # 同步完顺手在真机上 colcon build
#
# 环境变量：
#   ROBOT=user@10.21.31.103                    真机地址（第二 WiFi 用 10.21.41.1）
#   DEST=~/M20_sdk_deploy/src/M20_sdk_deploy   远端目标目录
set -euo pipefail

ROBOT=${ROBOT:-user@10.21.31.103}
DEST=${DEST:-'~/M20_sdk_deploy/src/M20_sdk_deploy'}
SRC_REL=src/M20_sdk_deploy

WITH_MESHES=0; FULL=0; DRY=0; DELETE=0; BUILD=0
for a in "$@"; do
  case "$a" in
    --with-meshes) WITH_MESHES=1 ;;
    --full)        FULL=1 ;;
    --dry-run)     DRY=1 ;;
    --delete)      DELETE=1 ;;
    --build)       BUILD=1 ;;
    -h|--help)     sed -n '2,26p' "$0"; exit 0 ;;
    *) echo "未知参数: $a（-h 看用法）" >&2; exit 2 ;;
  esac
done

REPO_ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$REPO_ROOT"
[ -d "$SRC_REL" ] || { echo "找不到 $SRC_REL（请在仓库根目录运行；当前 $REPO_ROOT）" >&2; exit 1; }

# 排除清单：这些目录真机运行与编译都不需要，只有"在真机上跑仿真/参数对照"才要。
EXC=(
  --exclude='/__pycache__/' --exclude='*.pyc'
  --exclude='/third_party/onnxruntime/x86/'
  --exclude='/third_party/gamepad/example/'
)
if [ "$WITH_MESHES" = 0 ]; then
  EXC+=( --exclude='/M20_description/' --exclude='/M20_Piper_description/' )
fi
if [ "$FULL" = 1 ]; then EXC=(); fi

echo "[sync] 源      : $REPO_ROOT/$SRC_REL"
echo "[sync] 目标    : $ROBOT:$DEST"
if [ ${#EXC[@]} -eq 0 ]; then
  echo "[sync] 排除    : 无（--full，等价旧 scp）"
else
  echo "[sync] 排除    : onnxruntime/x86、gamepad/example、__pycache__"
  [ "$WITH_MESHES" = 0 ] && echo "[sync]            M20_description/、M20_Piper_description/（仿真专用 mesh）"
fi
[ "$WITH_MESHES" = 1 ] && echo "[sync] 注意    : --with-meshes ⇒ 会传 2200+ 个 STL（慢）"

# ---- 选工具：rsync 优先，否则 tar over ssh ----
LOCAL_RSYNC=0; REMOTE_RSYNC=0
command -v rsync >/dev/null 2>&1 && LOCAL_RSYNC=1
if [ "$LOCAL_RSYNC" = 1 ]; then
  if ssh -o BatchMode=yes -o ConnectTimeout=5 "$ROBOT" 'command -v rsync >/dev/null 2>&1' 2>/dev/null; then
    REMOTE_RSYNC=1
  fi
fi

if [ "$LOCAL_RSYNC" = 1 ] && [ "$REMOTE_RSYNC" = 1 ]; then
  echo "[sync] 方式    : rsync（增量，只传改动）"
  FLAGS=(-az --human-readable --info=progress2 "${EXC[@]}")
  [ "$DRY" = 1 ] && FLAGS+=(--dry-run --itemize-changes)
  [ "$DELETE" = 1 ] && FLAGS+=(--delete)
  rsync "${FLAGS[@]}" "$SRC_REL/" "$ROBOT:$DEST/"
elif [ "$DRY" = 1 ]; then
  echo "[sync] --dry-run 需要：① 本机有 rsync（本机有=$LOCAL_RSYNC）② 远端也有 rsync" >&2
  echo "       远端没装 rsync 时不支持预演（直接跑不加 --dry-run 会走 tar 兜底）。" >&2
  echo "       先确认能连上：ssh -o BatchMode=yes $ROBOT true" >&2
  exit 1
else
  echo "[sync] 方式    : tar + ssh（每次都会重传未排除的部分）"
  TEXC=()
  for e in "${EXC[@]}"; do
    # ⚠️ rsync 的 `/xxx/` 是"相对传输根"的锚定写法，**不能照搬给 tar**：
    # tar 的成员名带了 `M20_sdk_deploy/` 前缀，且带尾斜杠的模式在部分版本上匹配不到子树
    # （实测照搬会 4514 个文件全传）。这里前后斜杠都去掉，实测有效（2038 个文件）。
    case "$e" in
      --exclude=*) p="${e#--exclude=}"; p="${p#/}"; p="${p%/}"; TEXC+=( "$p" ) ;;
    esac
  done
  TAR_EXC=()
  if [ ${#TEXC[@]} -gt 0 ]; then
    for e in "${TEXC[@]}"; do TAR_EXC+=( "--exclude=$e" ); done
  fi
  tar czf - "${TAR_EXC[@]}" -C "$(dirname "$SRC_REL")" "$(basename "$SRC_REL")" \
    | ssh "$ROBOT" "mkdir -p ~/M20_sdk_deploy/src && tar xzf - -C ~/M20_sdk_deploy/src"
  echo "[sync] 完成（远端解到 ~/M20_sdk_deploy/src/）"
  # 自检：排除没生效的话远端会多出几千个文件（tar 的 exclude 语义和 rsync 不一样，容易踩）
  N=$(ssh "$ROBOT" 'find ~/M20_sdk_deploy/src/M20_sdk_deploy -type f 2>/dev/null | wc -l')
  echo "[sync] 远端文件数 = $N（排除生效时约 2000；~4500 说明排除没生效）"
fi

if [ "$BUILD" = 1 ] && [ "$DRY" = 0 ]; then
  echo "[sync] 在真机上编译（arm 版，必须显式 -DSIM2SIM=OFF）..."
  ssh -t "$ROBOT" 'cd ~/M20_sdk_deploy && source /opt/ros/foxy/setup.bash && colcon build --packages-select m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=arm -DSIM2SIM=OFF'
fi

cat <<'EOT'

[sync] 下一步（在真机上）：
  cd ~/M20_sdk_deploy && source /opt/ros/foxy/setup.bash && source install/setup.bash
  首次或改了 C++：colcon build --packages-select m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=arm -DSIM2SIM=OFF
  只改了 Python（arm_controller / arm_teleop / vr_teleop）：不用重编，直接起节点
完整流程见 docs/DEPLOY_REAL_VR_zh.md
EOT
