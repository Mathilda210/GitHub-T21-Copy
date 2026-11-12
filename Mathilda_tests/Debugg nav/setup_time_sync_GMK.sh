#!/usr/bin/env bash
set -euo pipefail

echo "========================================="
echo " GMK Master 时间同步安装"
echo "========================================="

# 无论有无外网、有无 RTC：先把 AI 的时间接到 GMK，再起 PTP Master。
# NTP 只在 AI 做；GMK 不自己 NTP。运行期 PTP 永远 GMK Master。
USE_NTP_ON_MASTER="${USE_NTP_ON_MASTER:-0}"
USE_RTC_ON_GMK="${USE_RTC_ON_GMK:-auto}"
NTP_WAIT_SEC="${NTP_WAIT_SEC:-15}"
NTP_ROUTE_WAIT_SEC="${NTP_ROUTE_WAIT_SEC:-20}"
FOLLOW_WAIT_SEC="${FOLLOW_WAIT_SEC:-70}"
NTP_SERVERS="${NTP_SERVERS:-10.0.41.10 ntp.aliyun.com ntp1.aliyun.com time1.cloud.tencent.com ntp.ntsc.ac.cn}"
NTP_PROBE_IPS="${NTP_PROBE_IPS:-10.0.41.10 223.5.5.5 203.107.6.88}"
IFACE="${IFACE:-eth0}"
LINK_NET_PREFIX="${LINK_NET_PREFIX:-192.168.238.}"
TIME_SYNC_PHC_WAIT_SEC="${TIME_SYNC_PHC_WAIT_SEC:-8}"
TIME_SYNC_OFFSET_NS="${TIME_SYNC_OFFSET_NS:-1000000}"

has_phc() {
  local iface="$1"
  [[ -d "/sys/class/net/${iface}" ]] &&
    compgen -G "/sys/class/net/${iface}/device/ptp/ptp*" >/dev/null
}

iface_ptp_dev() {
  local iface="$1" ptp_path
  ptp_path="$(echo /sys/class/net/"${iface}"/device/ptp/ptp[0-9]*)"
  if [[ ! -e "${ptp_path}" ]]; then
    return 1
  fi
  echo "/dev/$(basename "${ptp_path}")"
}

unit_is_enabled() {
  local state
  state="$(systemctl is-enabled "$1" 2>/dev/null || true)"
  [[ "${state}" == "enabled" || "${state}" == "enabled-runtime" ]]
}

if unit_is_enabled paxini-ai.target; then
  echo "[ERROR] 本机已 enable paxini-ai.target，拒绝安装 GMK Master 时间同步"
  exit 1
fi

if [[ ! -d "/sys/class/net/${IFACE}" ]]; then
  echo "[ERROR] 网卡不存在: ${IFACE}。AI/GMK 板间 PTP 应走 eth0 / 192.168.238.0/24"
  ip -br link || true
  ip -br addr || true
  exit 1
fi

if ! has_phc "${IFACE}"; then
  echo "[ERROR] IFACE=${IFACE} 没有 PTP 硬件时钟，不能使用硬件时间戳"
  ip -br link || true
  exit 1
fi

if ! ip -4 addr show "${IFACE}" | grep -q "${LINK_NET_PREFIX}"; then
  echo "[WARN] ${IFACE} 尚未配置 ${LINK_NET_PREFIX}x 地址，继续安装。请确认板间口是 eth0 / 192.168.238.0/24"
  ip -br addr || true
fi

PTP_DEV="$(iface_ptp_dev "${IFACE}")" || true
if [[ -z "${PTP_DEV}" || ! -e "${PTP_DEV}" ]]; then
  echo "[ERROR] 无法解析 ${IFACE} 的 PHC 设备"
  ls -l "/sys/class/net/${IFACE}/device/ptp/" || true
  exit 1
fi

PTP4L_BIN="$(command -v ptp4l || true)"
PHC2SYS_BIN="$(command -v phc2sys || true)"
PHC_CTL_BIN="$(command -v phc_ctl || true)"

if [[ -z "${PTP4L_BIN}" || -z "${PHC2SYS_BIN}" ]]; then
  echo "[ERROR] 未找到 ptp4l 或 phc2sys，请先安装 linuxptp"
  exit 1
fi

IFACE_WAIT_SEC="${IFACE_WAIT_SEC:-45}"
GMK_TIMEOUT_SEC=$((IFACE_WAIT_SEC + 30))

echo "[INFO] IFACE=${IFACE}"
echo "[INFO] PTP_DEV=${PTP_DEV}"
echo "[INFO] ptp4l=${PTP4L_BIN}"
echo "[INFO] phc2sys=${PHC2SYS_BIN}"
echo "[INFO] USE_NTP_ON_MASTER=${USE_NTP_ON_MASTER}"
echo "[INFO] USE_RTC_ON_GMK=${USE_RTC_ON_GMK}"
echo "[INFO] NTP_WAIT_SEC=${NTP_WAIT_SEC}"
echo "[INFO] NTP_ROUTE_WAIT_SEC=${NTP_ROUTE_WAIT_SEC}"
echo "[INFO] FOLLOW_WAIT_SEC=${FOLLOW_WAIT_SEC}"
echo "[INFO] IFACE_WAIT_SEC=${IFACE_WAIT_SEC}"
echo "[INFO] TIME_SYNC_PHC_WAIT_SEC=${TIME_SYNC_PHC_WAIT_SEC}"
echo "[INFO] TIME_SYNC_OFFSET_NS=${TIME_SYNC_OFFSET_NS}"
echo "[INFO] NTP_SERVERS=${NTP_SERVERS}"
echo "[INFO] NTP_PROBE_IPS=${NTP_PROBE_IPS}"

# 1) 清理旧服务，避免竞争。不 pkill，以免误杀安装脚本自身。
for s in \
  time-sync-master.service \
  ptp4l-master.service \
  phc2sys-master.service \
  time-sync-gmk.service \
  ptp4l-gmk-master.service \
  phc2sys-gmk-master.service \
  ptp4l-gmk-follow.service \
  phc2sys-gmk-follow.service \
  gmk-time-role.service \
  time-sync-slave.service \
  ptp4l-slave.service \
  phc2sys-slave.service \
  time-sync-ai.service \
  ptp4l-ai-slave.service \
  phc2sys-ai-slave.service
do
  sudo systemctl disable --now "$s" 2>/dev/null || true
done

for u in fake-hwclock.service fake-hwclock-save.service fake-hwclock.timer; do
  sudo systemctl stop "$u" 2>/dev/null || true
  sudo systemctl disable "$u" 2>/dev/null || true
  sudo systemctl mask "$u" 2>/dev/null || true
done

# 2) 写 PTP 配置（强制 Master）
sudo mkdir -p /etc/linuxptp
sudo tee /etc/linuxptp/ptp4l_gmk_master.conf >/dev/null <<'EOF'
[global]
twoStepFlag 1
time_stamping hardware
network_transport UDPv4
delay_mechanism E2E
domainNumber 0
serverOnly 1
priority1 10
priority2 10
clockClass 6
clockAccuracy 0x20
offsetScaledLogVariance 0x21
logAnnounceInterval 1
logSyncInterval 0
logMinDelayReqInterval 0
announceReceiptTimeout 3
summary_interval 1
# 部分网卡 TX 时间戳轮询默认 1ms 太短，会 MASTER -> FAULTY
tx_timestamp_timeout 100
EOF

sudo tee /etc/linuxptp/ptp4l_gmk_follow.conf >/dev/null <<'EOF'
[global]
twoStepFlag 1
time_stamping hardware
network_transport UDPv4
delay_mechanism E2E
domainNumber 0
slaveOnly 1
priority1 248
priority2 248
clockClass 248
clockAccuracy 0xFE
offsetScaledLogVariance 0xFFFF
logAnnounceInterval 1
logSyncInterval 0
logMinDelayReqInterval 0
announceReceiptTimeout 5
summary_interval 1
step_threshold 0.5
tx_timestamp_timeout 100
EOF

# 3) NTP 与环境文件。PTP 的 ExecStart 从这里读 IFACE / PTP_DEV。
sudo mkdir -p /etc/systemd/timesyncd.conf.d
sudo tee /etc/systemd/timesyncd.conf.d/99-time-sync-gmk.conf >/dev/null <<EOF
[Time]
NTP=${NTP_SERVERS}
FallbackNTP=
PollIntervalMinSec=16
PollIntervalMaxSec=1024
EOF

sudo tee /etc/default/time-sync-gmk >/dev/null <<EOF
IFACE=${IFACE}
LINK_NET_PREFIX=${LINK_NET_PREFIX}
PTP_DEV=${PTP_DEV}
PTP4L_BIN=${PTP4L_BIN}
PHC2SYS_BIN=${PHC2SYS_BIN}
PHC_CTL_BIN=${PHC_CTL_BIN}
USE_NTP_ON_MASTER=${USE_NTP_ON_MASTER}
USE_RTC_ON_GMK=${USE_RTC_ON_GMK}
NTP_WAIT_SEC=${NTP_WAIT_SEC}
NTP_ROUTE_WAIT_SEC=${NTP_ROUTE_WAIT_SEC}
FOLLOW_WAIT_SEC=${FOLLOW_WAIT_SEC}
IFACE_WAIT_SEC=${IFACE_WAIT_SEC}
TIME_SYNC_PHC_WAIT_SEC=${TIME_SYNC_PHC_WAIT_SEC}
TIME_SYNC_OFFSET_NS=${TIME_SYNC_OFFSET_NS}
NTP_SERVERS="${NTP_SERVERS}"
NTP_PROBE_IPS="${NTP_PROBE_IPS}"
EOF

# 4) 启动脚本：可选 NTP 必须在 PTP 启动前结束并 mask，二者不重叠。
sudo tee /usr/local/sbin/time_sync_gmk_bootstrap.sh >/dev/null <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
source /etc/default/time-sync-gmk || true

LOCK=/run/time_sync_gmk.lock
exec 9>"${LOCK}"
flock -n 9 || { echo "[WARN] GMK bootstrap already running"; exit 0; }

STAMP=/run/time-sync-gmk.bootstrapped
if [[ -f "${STAMP}" ]]; then
  up="$(cut -d. -f1 /proc/uptime 2>/dev/null || echo 0)"
  prev="$(cat "${STAMP}" 2>/dev/null || echo 0)"
  if [[ "${prev}" =~ ^[0-9]+$ ]] && (( up >= prev && up - prev < 30 )); then
    echo "[INFO] GMK bootstrap 刚完成，跳过重复执行"
    exit 0
  fi
fi

echo "[INFO] GMK bootstrap start"

mask_ntp_units() {
  local u
  for u in systemd-timesyncd.service chronyd.service chrony.service ntp.service ntpd.service \
           fake-hwclock.service fake-hwclock-save.service fake-hwclock.timer; do
    systemctl stop "$u" 2>/dev/null || true
    systemctl disable "$u" 2>/dev/null || true
    systemctl mask "$u" 2>/dev/null || true
  done
}

# 只探测 IP，避免 DNS 卡住。GMK 的 NTP 走 AI（default via 192.168.238.71）。
ntp_ip_reachable() {
  local s
  for s in ${NTP_PROBE_IPS:-} ${NTP_SERVERS:-}; do
    [[ "${s}" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || continue
    ping -c1 -W1 "${s}" >/dev/null 2>&1 && return 0
  done
  return 1
}

wait_ptp_iface() {
  local i
  echo "[INFO] 等待 PTP 网卡 ${IFACE:-eth0} 就绪（不对外网）"
  for i in $(seq 1 "${IFACE_WAIT_SEC:-30}"); do
    if ip link show "${IFACE:-eth0}" 2>/dev/null | grep -Eq 'state UP|LOWER_UP'; then
      if [[ -z "${LINK_NET_PREFIX:-}" ]] ||
         ip -4 addr show "${IFACE:-eth0}" 2>/dev/null | grep -q "${LINK_NET_PREFIX}"; then
        echo "[INFO] ${IFACE:-eth0} 已就绪"
        return 0
      fi
    fi
    sleep 1
  done
  echo "[WARN] ${IFACE:-eth0} ${IFACE_WAIT_SEC:-30}s 内未就绪，仍尝试启动 PTP"
}

timedatectl set-timezone Asia/Shanghai || true
systemctl stop chronyd.service chrony.service ntp.service ntpd.service fake-hwclock.service 2>/dev/null || true
systemctl disable chronyd.service chrony.service ntp.service ntpd.service fake-hwclock.service 2>/dev/null || true
systemctl mask chronyd.service chrony.service ntp.service ntpd.service fake-hwclock.service 2>/dev/null || true

clock_looks_valid() {
  local y
  y="$(date -u +%Y 2>/dev/null || echo 0)"
  [[ "${y}" =~ ^[0-9]+$ ]] && (( y >= 2024 ))
}

phc_unix_sec() {
  local bin="${PHC_CTL_BIN:-phc_ctl}"
  "${bin}" "${PTP_DEV}" get 2>/dev/null | awk '{
    for (i = 1; i <= NF; i++) {
      if ($i == "is" && $(i+1) ~ /^[0-9]+(\.[0-9]+)?$/) {
        split($(i+1), a, ".")
        print a[1]
        exit
      }
    }
  }'
}

phc_looks_valid() {
  local sec
  sec="$(phc_unix_sec)"
  [[ "${sec}" =~ ^[0-9]+$ ]] && (( sec >= 1704067200 ))
}

rtc_present() {
  [[ -e /sys/class/rtc/rtc0 ]] || [[ -e /dev/rtc0 ]] || [[ -e /dev/rtc ]]
}

# auto/1：有 RTC 设备才用；0：强制不用（没电池也可设 0）。
gmk_rtc_enabled() {
  case "${USE_RTC_ON_GMK:-auto}" in
    0|no|false|off) return 1 ;;
    *) rtc_present ;;
  esac
}

rtc_looks_valid() {
  local out y
  gmk_rtc_enabled || return 1
  out="$(hwclock --utc --show 2>/dev/null || true)"
  y="$(printf '%s\n' "${out}" | grep -oE '[12][0-9]{3}' | head -1)"
  [[ -n "${y}" ]] && (( y >= 2024 ))
}

sync_sys_to_rtc() {
  gmk_rtc_enabled || return 0
  hwclock --systohc --utc || true
  echo "[INFO] 已把系统钟写入 RTC"
}

sync_rtc_to_sys() {
  rtc_looks_valid || return 1
  hwclock --hctosys --utc || true
  echo "[INFO] 已从 RTC 读取系统钟: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
}

start_gmk_master() {
  systemctl stop phc2sys-gmk-follow.service ptp4l-gmk-follow.service 2>/dev/null || true
  systemctl restart ptp4l-gmk-master.service
  systemctl restart phc2sys-gmk-master.service
}

start_gmk_follow() {
  systemctl stop phc2sys-gmk-master.service ptp4l-gmk-master.service 2>/dev/null || true
  systemctl stop phc2sys-gmk-follow.service 2>/dev/null || true
  systemctl restart ptp4l-gmk-follow.service
}

wait_ptp_iface

# NTP 只允许在 AI 开机时跑一次。GMK 永远不碰 NTP，避免和 PTP Master 双写。
echo "[INFO] GMK 不启动 NTP，墙钟只来自 AI（follow 一次）或本机 RTC"
mask_ntp_units
timedatectl set-ntp false 2>/dev/null || true

if ! clock_looks_valid; then
  if gmk_rtc_enabled; then
    if sync_rtc_to_sys; then
      :
    else
      echo "[WARN] 有 RTC 但时间无效（无电池或未校过），不从 RTC 读取"
    fi
  else
    echo "[INFO] 未使用 RTC（无设备或 USE_RTC_ON_GMK=0）"
  fi
fi

if [[ -z "${PTP_DEV:-}" || ! -e "${PTP_DEV}" ]]; then
  echo "[ERROR] PTP_DEV=${PTP_DEV:-unset} 不存在，拒绝启动 phc2sys"
  exit 1
fi

# 角色脚本后台跟 AI（最多 70s）再升 Master，oneshot 不等待，避免安装卡住。
systemctl restart gmk-time-role.service
sleep 2
if ! systemctl is-active --quiet gmk-time-role.service; then
  echo "[ERROR] gmk-time-role 未能保持 active"
  systemctl status gmk-time-role.service --no-pager -n 20 || true
  exit 1
fi
if systemctl is-active --quiet ptp4l-gmk-master.service; then
  echo "[INFO] GMK 已是 Master，墙钟=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
else
  echo "[INFO] 正在后台跟随 AI 取时，随后自动升 Master，当前 $(date -u +%Y-%m-%dT%H:%M:%SZ)"
fi

echo "[INFO] GMK 时间角色已启动 IFACE=${IFACE} PTP_DEV=${PTP_DEV} role=${role_ready:-pending}"

phc_offset_ns() {
  journalctl -u phc2sys-gmk-master.service -n 40 --no-pager 2>/dev/null |
    awk '/offset/ {
      for (i = 1; i <= NF; i++) if ($i == "offset") val = $(i + 1)
    }
    END {
      if (val == "") exit 1
      a = val + 0
      if (a < 0) a = -a
      printf "%.0f\n", a
    }'
}

phc_converged() {
  local offset limit
  systemctl is-active --quiet phc2sys-gmk-master.service || return 1
  offset="$(phc_offset_ns)" || return 1
  [[ -n "${offset}" ]] || return 1
  limit="${TIME_SYNC_OFFSET_NS:-1000000}"
  awk -v o="${offset}" -v l="${limit}" 'BEGIN { exit !(o <= l) }'
}

if systemctl is-active --quiet phc2sys-gmk-master.service; then
  wait_sec="${TIME_SYNC_PHC_WAIT_SEC:-30}"
  echo "[INFO] 等待 PHC 驯服 CLOCK_REALTIME 且 |offset| <= ${TIME_SYNC_OFFSET_NS:-1000000} ns，最多 ${wait_sec}s"
  locked=no
  for _ in $(seq 1 "${wait_sec}"); do
    if phc_converged; then
      locked=yes
      echo "[INFO] PHC 已接近收敛: offset=$(phc_offset_ns) ns"
      break
    fi
    sleep 1
  done
  if [[ "${locked}" != "yes" ]]; then
    echo "[WARN] ${wait_sec} 秒内 PHC 未接近收敛，继续放行业务。当前 offset=$(phc_offset_ns 2>/dev/null || echo unset) ns"
    systemctl status phc2sys-gmk-master.service --no-pager -n 20 || true
  fi
fi

cut -d. -f1 /proc/uptime >"${STAMP}" 2>/dev/null || echo 0 >"${STAMP}"
echo "[INFO] GMK bootstrap done"
EOF
sudo chmod +x /usr/local/sbin/time_sync_gmk_bootstrap.sh

sudo tee /usr/local/sbin/time_sync_gmk_role.sh >/dev/null <<'EOF'
#!/usr/bin/env bash
# 无论 AI 是否刚 NTP、是否 1970：先跟 AI 把墙钟拷过来，再永远当 PTP Master。
set -euo pipefail
source /etc/default/time-sync-gmk || true

: "${PTP_DEV:=/dev/ptp0}"
: "${PHC_CTL_BIN:=phc_ctl}"

clock_looks_valid() {
  local y
  y="$(date -u +%Y 2>/dev/null || echo 0)"
  [[ "${y}" =~ ^[0-9]+$ ]] && (( y >= 2024 ))
}

port_state() {
  pmc -u -b 0 'GET PORT_DATA_SET' 2>/dev/null | awk '/portState/ {print $2; exit}'
}

start_gmk_master() {
  systemctl stop phc2sys-gmk-follow.service ptp4l-gmk-follow.service 2>/dev/null || true
  systemctl restart ptp4l-gmk-master.service
  systemctl restart phc2sys-gmk-master.service
}

start_gmk_follow() {
  systemctl stop phc2sys-gmk-master.service ptp4l-gmk-master.service 2>/dev/null || true
  systemctl stop phc2sys-gmk-follow.service 2>/dev/null || true
  systemctl restart ptp4l-gmk-follow.service
}

sync_sys_to_rtc() {
  case "${USE_RTC_ON_GMK:-auto}" in
    0|no|false|off) return 0 ;;
  esac
  [[ -e /sys/class/rtc/rtc0 || -e /dev/rtc0 || -e /dev/rtc ]] || return 0
  if hwclock --systohc --utc; then
    echo "[INFO] 已写入 RTC $(hwclock --utc --show 2>/dev/null | head -1)"
  else
    echo "[WARN] hwclock --systohc 失败，下次无外网开机可能仍是旧时间"
  fi
}

seed_wait="${FOLLOW_WAIT_SEC:-70}"
echo "[INFO] 无论本机 RTC 是否有效，先跟 AI 取时，最多 ${seed_wait}s，当前 $(date -u +%Y-%m-%dT%H:%M:%SZ)"
start_gmk_follow
got_ai=no
for _ in $(seq 1 "${seed_wait}"); do
  st="$(port_state)"
  if [[ "${st}" == "SLAVE" ]]; then
    if ! systemctl is-active --quiet phc2sys-gmk-follow.service; then
      systemctl restart phc2sys-gmk-follow.service
      echo "[INFO] 已锁定 AI，PHC -> CLOCK_REALTIME"
    fi
    sleep 1
    got_ai=yes
    echo "[INFO] 已从 AI 取时 $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    sync_sys_to_rtc
    break
  fi
  sleep 1
done
if [[ "${got_ai}" != "yes" ]]; then
  echo "[WARN] ${seed_wait}s 内未见到 AI，用当前墙钟升 Master"
fi

# 无外网/1970 也要升 Master：PTP 运行期永远 GMK 主。
start_gmk_master
if clock_looks_valid; then
  sync_sys_to_rtc
fi
echo "[INFO] 此后 PTP 永远 Master 墙钟=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
while true; do
  if ! systemctl is-active --quiet ptp4l-gmk-master.service; then
    echo "[WARN] ptp4l-gmk-master 退出，重拉"
    start_gmk_master
  fi
  sleep 5
done
EOF
sudo chmod +x /usr/local/sbin/time_sync_gmk_role.sh

# 5) systemd 服务。ptp4l/phc2sys 不 enable，只由 bootstrap 拉起。
sudo tee /etc/systemd/system/ptp4l-gmk-master.service >/dev/null <<'EOF'
[Unit]
Description=GMK PTP4L Master
# PTP 只走板间 eth0，不依赖外网 network-online
After=sysinit.target
Conflicts=ptp4l-gmk-follow.service phc2sys-gmk-follow.service

[Service]
Type=simple
EnvironmentFile=-/etc/default/time-sync-gmk
# 部分 systemd 不展开 ExecStart 里的 ${VAR}，会把字面量当可执行文件 -> 203/EXEC
ExecStart=/bin/bash -c 'exec "${PTP4L_BIN}" -i "${IFACE}" -H -m -f /etc/linuxptp/ptp4l_gmk_master.conf'
Restart=always
RestartSec=2
EOF

sudo tee /etc/systemd/system/phc2sys-gmk-master.service >/dev/null <<'EOF'
[Unit]
Description=GMK PHC2SYS (CLOCK_REALTIME -> eth0 PHC)
After=ptp4l-gmk-master.service
Requires=ptp4l-gmk-master.service
Conflicts=ptp4l-gmk-follow.service phc2sys-gmk-follow.service

[Service]
Type=simple
EnvironmentFile=-/etc/default/time-sync-gmk
ExecStart=/bin/bash -c 'exec "${PHC2SYS_BIN}" -s CLOCK_REALTIME -c "${PTP_DEV}" -w -R 8 -S 0.5 -m'
Restart=always
RestartSec=2
EOF

sudo tee /etc/systemd/system/ptp4l-gmk-follow.service >/dev/null <<'EOF'
[Unit]
Description=GMK PTP4L Follow AI (temporary slave)
After=sysinit.target
Conflicts=ptp4l-gmk-master.service phc2sys-gmk-master.service

[Service]
Type=simple
EnvironmentFile=-/etc/default/time-sync-gmk
ExecStart=/bin/bash -c 'exec "${PTP4L_BIN}" -i "${IFACE}" -H -m -f /etc/linuxptp/ptp4l_gmk_follow.conf'
Restart=always
RestartSec=2
EOF

sudo tee /etc/systemd/system/phc2sys-gmk-follow.service >/dev/null <<'EOF'
[Unit]
Description=GMK PHC2SYS Follow AI (PHC -> CLOCK_REALTIME)
After=ptp4l-gmk-follow.service
Requires=ptp4l-gmk-follow.service
Conflicts=ptp4l-gmk-master.service phc2sys-gmk-master.service

[Service]
Type=simple
EnvironmentFile=-/etc/default/time-sync-gmk
ExecStart=/bin/bash -c 'exec "${PHC2SYS_BIN}" -s "${IFACE}" -c CLOCK_REALTIME -w -R 8 -S 0.5 -m'
Restart=always
RestartSec=2
EOF

sudo tee /etc/systemd/system/gmk-time-role.service >/dev/null <<'EOF'
[Unit]
Description=GMK PTP role (take AI NTP once, then stay Master)
After=sysinit.target

[Service]
Type=simple
EnvironmentFile=-/etc/default/time-sync-gmk
ExecStart=/usr/local/sbin/time_sync_gmk_role.sh
Restart=always
RestartSec=2
EOF

sudo tee /etc/systemd/system/time-sync-gmk.service >/dev/null <<EOF
[Unit]
Description=GMK Time Sync Bootstrap
# 不绑定 network-online：无外网时该 target 会长时间阻塞或失败
After=sysinit.target

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/time_sync_gmk_bootstrap.sh
RemainAfterExit=true
TimeoutStartSec=${GMK_TIMEOUT_SEC}

[Install]
WantedBy=multi-user.target
EOF

# 6) 生效
sudo systemctl daemon-reload
sudo systemctl enable time-sync-gmk.service
echo "[INFO] 启动 time-sync-gmk（eth0 已就绪时约几秒，跟 AI 取时在后台）"
sudo /usr/local/sbin/time_sync_gmk_bootstrap.sh
sudo systemctl start time-sync-gmk.service

echo "=== GMK RESULT ==="
echo "IFACE=${IFACE} PTP_DEV=${PTP_DEV}"
timeout 5 timedatectl || true
date || true
systemctl is-enabled time-sync-gmk.service || true
systemctl is-enabled ptp4l-gmk-master.service phc2sys-gmk-master.service 2>/dev/null || true
systemctl is-active ptp4l-gmk-master.service phc2sys-gmk-master.service ptp4l-gmk-follow.service gmk-time-role.service time-sync-gmk.service || true
systemctl is-enabled systemd-timesyncd chronyd ntp ntpd 2>/dev/null || true
ps -ef | grep -E "ptp4l|phc2sys|timesyncd" | grep -v grep || true
sudo systemctl status gmk-time-role.service --no-pager -n 20 || true
sudo systemctl status ptp4l-gmk-master.service --no-pager -n 15 || true
sudo systemctl status ptp4l-gmk-follow.service --no-pager -n 15 || true
sudo systemctl status phc2sys-gmk-master.service --no-pager -n 15 || true
if command -v pmc >/dev/null; then
  timeout 3 pmc -u -b 0 'GET PARENT_DATA_SET' || true
  timeout 3 pmc -u -b 0 'GET CURRENT_DATA_SET' || true
fi
echo "[INFO] 若此刻仍是 follow，属正常：角色脚本最多 70s 内从 AI 取时后升 Master"
echo "[INFO] 查看: journalctl -u gmk-time-role.service -n 30 --no-pager"
echo "========================================="
echo "GMK 安装完成"
echo "========================================="
