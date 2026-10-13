#!/usr/bin/env bash
# 把 A-Frame 1.7.1 下到 third_party/XLeVR/web-ui/aframe.min.js。
#
# **必须在一台能上外网的机器上运行**（开发机容器通常也不行）。
#
# 为什么需要它：头显页面 web-ui/index.html 要靠 A-Frame 渲染 <a-scene>。
# 早先这个脚本是从 https://aframe.io/releases/1.7.1/aframe.min.js 现拉的 CDN 地址，
# 但真机（上位机）和头显都在内网、没有外网 ⇒ 脚本拉不到 ⇒ 场景不初始化：
# 头显里只剩中间那张 "VR Teleoperation" 卡片，什么按钮都没有（DEF-058）。
# 所以引擎必须**离线自带**。
#
# 用法：
#   bash src/M20_sdk_deploy/scripts/fetch_aframe.sh                 # 下载到仓库的 web-ui/
#   bash src/M20_sdk_deploy/scripts/fetch_aframe.sh --dest /path/to/web-ui
#   bash src/M20_sdk_deploy/scripts/fetch_aframe.sh --url <任意镜像/自建地址>
#   bash src/M20_sdk_deploy/scripts/fetch_aframe.sh --check         # 只检查仓库里有没有、能不能用
#
# 下完之后的两种用法：
#   * 开发机（sim2sim 直接读仓库里的文件）：无需再操作；
#   * 真机/上位机：`bash src/M20_sdk_deploy/scripts/sync_to_robot.sh` 会一起同步过去
#     （只同步这一个文件也行：scp <仓库>/src/M20_sdk_deploy/third_party/XLeVR/web-ui/aframe.min.js \
#        user@10.21.31.103:~/M20_sdk_deploy/src/M20_sdk_deploy/third_party/XLeVR/web-ui/）
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
DEST_DIR="$REPO_ROOT/src/M20_sdk_deploy/third_party/XLeVR/web-ui"
URLS=(
  # 官方 release（就是原来 index.html 里引用的那个文件）
  "https://aframe.io/releases/1.7.1/aframe.min.js"
  # npm 包内的构建产物（jsdelivr / unpkg）
  "https://cdn.jsdelivr.net/npm/aframe@1.7.1/dist/aframe-master.min.js"
  "https://unpkg.com/aframe@1.7.1/dist/aframe-master.min.js"
  # cdnjs（bootcdn / staticfile 是国内可达的 cdnjs 镜像，头显/上位机能连外网时也能用）
  "https://cdnjs.cloudflare.com/ajax/libs/aframe/1.7.1/aframe.min.js"
  "https://cdn.bootcdn.net/ajax/libs/aframe/1.7.1/aframe.min.js"
  "https://cdn.staticfile.org/aframe/1.7.1/aframe.min.js"
)

CUSTOM_URL=""; CHECK_ONLY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --dest) DEST_DIR="$2"; shift 2 ;;
    --url)  CUSTOM_URL="$2"; shift 2 ;;
    --check) CHECK_ONLY=1; shift ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "未知参数: $1（-h 看用法）" >&2; exit 2 ;;
  esac
done
[ -n "$CUSTOM_URL" ] && URLS=("$CUSTOM_URL")
TARGET="$DEST_DIR/aframe.min.js"

# 判定"这份文件能不能用"：够大 + 是 A-Frame 的 JS（不是 404/登录页那种 HTML）
check_file() {
  local f="$1"
  [ -f "$f" ] || { echo "缺少 $f"; return 1; }
  local size; size=$(wc -c < "$f")
  if [ "$size" -lt 300000 ]; then
    echo "文件太小（$size 字节，正常约 1247 KB）：可能下到了错误页而不是引擎"; return 1
  fi
  if head -c 200 "$f" | grep -qi '<!doctype\|<html'; then
    echo "文件是 HTML（$size 字节）：说明下到的是错误页/登录页"; return 1
  fi
  grep -q "AFRAME" "$f" || { echo "内容里找不到 AFRAME 字样"; return 1; }
  echo "OK：$f（$(( size / 1024 )) KB）"
  return 0
}

if [ "$CHECK_ONLY" = 1 ]; then
  check_file "$TARGET" && exit 0 || exit 1
fi

command -v curl >/dev/null 2>&1 || { echo "需要 curl" >&2; exit 1; }
mkdir -p "$DEST_DIR"
TMP=$(mktemp -d)
trap 'rm -f "$TMP/aframe.min.js"; rmdir "$TMP" 2>/dev/null || true' EXIT

ok=0
for u in "${URLS[@]}"; do
  echo "[fetch] 尝试 $u"
  if curl -fsSL --connect-timeout 10 --max-time 120 -o "$TMP/aframe.min.js" "$u"; then
    if check_file "$TMP/aframe.min.js"; then cp -f "$TMP/aframe.min.js" "$TARGET"; ok=1; break; fi
  else
    echo "[fetch]   失败（网络/镜像不可达）"
  fi
done
if [ "$ok" != 1 ]; then
  cat >&2 <<'EOF'

[fetch] 所有镜像都失败了。手工办法（任选其一）：
  1) 用能上外网的浏览器打开 https://aframe.io/releases/1.7.1/aframe.min.js
     → 另存为 aframe.min.js → 放到 third_party/XLeVR/web-ui/ 下；
  2) 或者用 npm（有 node 的机器）：npm pack aframe@1.7.1
     → 解压 aframe-1.7.1.tgz → 取 package/dist/aframe-master.min.js 改名成 aframe.min.js；
  3) 下完自检：bash src/M20_sdk_deploy/scripts/fetch_aframe.sh --check
EOF
  exit 1
fi

echo
echo "[fetch] 已就位: $TARGET"
echo "[fetch] 下一步："
echo "        真机：bash src/M20_sdk_deploy/scripts/sync_to_robot.sh   # 会把这个文件一起同步"
echo "        单文件：scp \"$TARGET\" user@10.21.31.103:~/M20_sdk_deploy/src/M20_sdk_deploy/third_party/XLeVR/web-ui/"
echo "        然后头显里刷新 https://<上位机IP>:8443 即可（顶部红色的报错条消失就对了）"
