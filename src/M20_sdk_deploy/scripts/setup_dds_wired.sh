#!/usr/bin/env bash
# 把 ROS 2 的 DDS 发现**钉死到有线网卡 + 单播对端**，解决跨机器互相看不到的问题。
#
# 背景：默认的 Fast DDS 会在**所有**网卡上做组播发现（Jetson 上还有 Wi-Fi、
# docker0、l4tbr0、usb0…）。交换机挡了组播、或者多网卡选错接口时，就会出现
# "本机 echo 正常、对端 node list 里看不到我"这种半通状态。
#
# 用法（两台机器各跑一次，参数换成自己的）：
#   source src/M20_sdk_deploy/scripts/setup_dds_wired.sh <本机有线IP> <对端有线IP> <有线网卡名>
#   # 例（Jetson）: source .../setup_dds_wired.sh 10.21.31.201 10.21.31.103 end0
#   # 例（M20）  : source .../setup_dds_wired.sh 10.21.31.103 10.21.31.201 eth0
#
# 脚本只设 3 个环境变量（在当前 shell 生效，不改系统）：
#   FASTRTPS_DEFAULT_PROFILES_FILE  -> 生成的 profile（只留有线网卡 + 只找对端）
#   ROS_LOCALHOST_ONLY=0            -> 必须允许跨机器
#   RMW_IMPLEMENTATION              -> 没设过才填 rmw_fastrtps_cpp
# ⚠️ **不动 ROS_DOMAIN_ID**：机器人自带服务用的是它自己的域（你那边就是默认 0），
#    改域会让 rl_deploy 看不到 /JOINTS_DATA。两台机器**各自**用同一个值即可，
#    本脚本只把它打印出来让你核对。
# 设完**重启**你的节点（agx launch / adapter / rl_deploy 都要在新 shell 里起）。

set -e

if [ $# -lt 3 ]; then
  echo "用法: source $0 <本机有线IP> <对端有线IP> <有线网卡名>"
  return 2 2>/dev/null || exit 2
fi

SELF_IP="$1"; PEER_IP="$2"; IFACE="$3"
OUT="${FASTRTPS_PROFILE_OUT:-/tmp/fastdds_wired.xml}"

cat > "$OUT" <<XML
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_wired</transport_id>
      <type>UDPv4</type>
      <!-- 只用有线网卡：把组播/单播都限制在这张卡上，避免被 Wi-Fi/docker0 带偏 -->
      <interfaceWhiteList>
        <address>${SELF_IP}</address>
      </interfaceWhiteList>
      <sendBufferSize>65536</sendBufferSize>
      <receiveBufferSize>65536</receiveBufferSize>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="wired" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_wired</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <builtin>
        <discovery_config>
          <discoveryProtocol>SIMPLE</discoveryProtocol>
          <!-- 只找这两个 IP，不依赖组播 -->
          <initialPeersList>
            <locator><udpv4><address>${SELF_IP}</address></udpv4></locator>
            <locator><udpv4><address>${PEER_IP}</address></udpv4></locator>
          </initialPeersList>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
XML

export FASTRTPS_DEFAULT_PROFILES_FILE="$OUT"
export ROS_LOCALHOST_ONLY=0
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"
export RMW_IMPLEMENTATION

echo "[dds] profile  -> $OUT"
echo "[dds] 本机 ${SELF_IP}（网卡 ${IFACE}），对端 ${PEER_IP}"
echo "[dds] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<空=默认0>}  RMW=$RMW_IMPLEMENTATION  LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "[dds] ⚠️ 两台机器的 ROS_DOMAIN_ID 必须一致（本脚本不改它；机器人自带服务用默认 0）"
echo "[dds] 请确认网卡名/IP： ip -brief addr    （上面写的网卡名仅用于提示）"
echo "[dds] 现在**重启**相关节点（agx launch / adapter / rl_deploy）让它们继承这些变量。"
echo "[dds] 验证： ros2 daemon stop; ros2 node list  应能看到对方的节点"
