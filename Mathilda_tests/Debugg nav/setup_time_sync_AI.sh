#!/usr/bin/env bash
set -euo pipefail

echo "========================================="
echo " AI Slave 时间同步安装"
echo "========================================="

IFACE="${IFACE:-eth0}"
LINK_NET_PREFIX="${LINK_NET_PREFIX:-192.168.238.}"
TIME_SYNC_LOCK_WAIT_SEC="${TIME_SYNC_LOCK_WAIT_SEC:-20}"
TIME_SYNC_OFFSET_NS="${TIME_SYNC_OFFSET_NS:-1000000}"
TIME_SYNC_STEP_GUARD_SEC="${TIME_SYNC_STEP_GUARD_SEC:-2}"
IFACE_WAIT_SEC="${IFACE_WAIT_SEC:-15}"
# AI 开机 NTP 只校一次，写入 RTC 后立刻关掉；稳定运行只允许 PTP 改 CLOCK_REALTIME。
USE_NTP_ON_AI="${USE_NTP_ON_AI:-1}"
NTP_WAIT_SEC="${NTP_WAIT_SEC:-8}"
NTP_ROUTE_WAIT_SEC="${NTP_ROUTE_WAIT_SEC:-3}"
# 1970 / 墙钟无效时多等一会儿外网，再做这一次 NTP。
NTP_INVALID_WAIT_SEC="${NTP_INVALID_WAIT_SEC:-45}"
NTP_SERVERS="${NTP_SERVERS:-10.0.41.10 ntp.aliyun.com ntp1.aliyun.com time1.cloud.tencent.com ntp.ntsc.ac.cn}"
# 探测是否有外网（含 DNS IP）。223.5.5.5 / 1.1.1.1 不当 NTP 服务器。
NTP_PROBE_IPS="${NTP_PROBE_IPS:-10.0.41.10 223.5.5.5 203.107.6.88 1.1.1.1}"
# 真正 UDP/123 查询。阿里 NTP，不含 DNS。
NTP_QUERY_IPS="${NTP_QUERY_IPS:-10.0.41.10 203.107.6.88 120.25.115.20 120.25.108.11}"

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

if unit_is_enabled paxini-mc.target; then
  echo "[ERROR] 本机已 enable paxini-mc.target，拒绝安装 AI Slave 时间同步"
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

echo "[INFO] IFACE=${IFACE}"
echo "[INFO] PTP_DEV=${PTP_DEV}"
echo "[INFO] ptp4l=${PTP4L_BIN}"
echo "[INFO] phc2sys=${PHC2SYS_BIN}"
echo "[INFO] TIME_SYNC_LOCK_WAIT_SEC=${TIME_SYNC_LOCK_WAIT_SEC}"
echo "[INFO] TIME_SYNC_OFFSET_NS=${TIME_SYNC_OFFSET_NS}"
echo "[INFO] TIME_SYNC_STEP_GUARD_SEC=${TIME_SYNC_STEP_GUARD_SEC}"
echo "[INFO] IFACE_WAIT_SEC=${IFACE_WAIT_SEC}"
echo "[INFO] USE_NTP_ON_AI=${USE_NTP_ON_AI}"
echo "[INFO] NTP_WAIT_SEC=${NTP_WAIT_SEC}"
echo "[INFO] NTP_ROUTE_WAIT_SEC=${NTP_ROUTE_WAIT_SEC}"
echo "[INFO] NTP_INVALID_WAIT_SEC=${NTP_INVALID_WAIT_SEC}"
echo "[INFO] NTP_SERVERS=${NTP_SERVERS}"
echo "[INFO] NTP_PROBE_IPS=${NTP_PROBE_IPS}"
echo "[INFO] NTP_QUERY_IPS=${NTP_QUERY_IPS}"

# 1) 清理旧服务，避免竞争。不 pkill，以免误杀安装脚本自身。
for s in \
  time-sync-master.service \
  ptp4l-master.service \
  phc2sys-master.service \
  time-sync-gmk.service \
  ptp4l-gmk-master.service \
  phc2sys-gmk-master.service \
  time-sync-slave.service \
  ptp4l-slave.service \
  phc2sys-slave.service \
  time-sync-ai.service \
  ptp4l-ai-slave.service \
  phc2sys-ai-slave.service
do
  sudo systemctl disable --now "$s" 2>/dev/null || true
done

# 2) 安装时先关掉常驻 NTP / fake-hwclock。开机由 bootstrap 校一次再 mask，避免和 PTP 双写。
for u in systemd-timesyncd.service chronyd.service chrony.service ntp.service ntpd.service \
         fake-hwclock.service fake-hwclock-save.service fake-hwclock.timer; do
  sudo systemctl stop "$u" 2>/dev/null || true
  sudo systemctl disable "$u" 2>/dev/null || true
  sudo systemctl mask "$u" 2>/dev/null || true
done

sudo mkdir -p /etc/systemd/timesyncd.conf.d
sudo tee /etc/systemd/timesyncd.conf.d/99-time-sync-ai.conf >/dev/null <<EOF
[Time]
NTP=${NTP_SERVERS}
FallbackNTP=
PollIntervalMinSec=16
PollIntervalMaxSec=1024
EOF

# 3) 写 PTP 配置。不是 slaveOnly：GMK 未起时 AI 可暂当 Master 把 NTP/RTC 时间顶住。
#    GMK 升 Master（priority1=10）后 BMC 会把 AI 自动切成 Slave。
sudo mkdir -p /etc/linuxptp
sudo tee /etc/linuxptp/ptp4l_ai.conf >/dev/null <<'EOF'
[global]
twoStepFlag 1
time_stamping hardware
network_transport UDPv4
delay_mechanism E2E
domainNumber 0
priority1 128
priority2 128
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

sudo tee /etc/default/time-sync-ai >/dev/null <<EOF
IFACE=${IFACE}
LINK_NET_PREFIX=${LINK_NET_PREFIX}
PTP_DEV=${PTP_DEV}
PTP4L_BIN=${PTP4L_BIN}
PHC2SYS_BIN=${PHC2SYS_BIN}
PHC_CTL_BIN=${PHC_CTL_BIN}
TIME_SYNC_LOCK_WAIT_SEC=${TIME_SYNC_LOCK_WAIT_SEC}
TIME_SYNC_OFFSET_NS=${TIME_SYNC_OFFSET_NS}
TIME_SYNC_STEP_GUARD_SEC=${TIME_SYNC_STEP_GUARD_SEC}
IFACE_WAIT_SEC=${IFACE_WAIT_SEC}
USE_NTP_ON_AI=${USE_NTP_ON_AI}
NTP_WAIT_SEC=${NTP_WAIT_SEC}
NTP_ROUTE_WAIT_SEC=${NTP_ROUTE_WAIT_SEC}
NTP_INVALID_WAIT_SEC=${NTP_INVALID_WAIT_SEC}
NTP_SERVERS="${NTP_SERVERS}"
NTP_PROBE_IPS="${NTP_PROBE_IPS}"
NTP_QUERY_IPS="${NTP_QUERY_IPS}"
EOF

sudo tee /usr/local/sbin/time_sync_ai_ntp_once.sh >/dev/null <<'EOF'
#!/usr/bin/env bash
# 一次 UDP NTP，不启动 timesyncd。被 bootstrap / phc2sys HOLD 调用。
# 成功 exit 0 并写入 RTC；失败 exit 1。
set -euo pipefail
source /etc/default/time-sync-ai || true

clock_looks_valid() {
  local y
  y="$(date -u +%Y 2>/dev/null || echo 0)"
  [[ "${y}" =~ ^[0-9]+$ ]] && (( y >= 2024 ))
}

ntp_has_uplink() {
  local s line
  for s in ${NTP_PROBE_IPS:-} ${NTP_QUERY_IPS:-} ${NTP_SERVERS:-}; do
    [[ "${s}" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] || continue
    line="$(ip route get "${s}" 2>/dev/null | head -1)" || true
    [[ -n "${line}" ]] || continue
    # 板间 PTP、链路本地 169.254 都到不了 NTP
    echo "${line}" | grep -q "192.168.238." && continue
    echo "${line}" | grep -q "169.254." && continue
    echo "${line}" | grep -q " dev ${IFACE:-eth0} " && continue
    echo "[INFO] NTP 路由: ${line}"
    return 0
  done
  return 1
}

ntp_collect_ips() {
  local s ip
  for s in ${NTP_QUERY_IPS:-} ${NTP_SERVERS:-}; do
    if [[ "${s}" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
      printf '%s\n' "${s}"
      continue
    fi
    ip="$(timeout 2 getent ahostsv4 "${s}" 2>/dev/null | awk '/^[0-9]+\./ {print $1; exit}' || true)"
    [[ -n "${ip}" ]] && printf '%s\n' "${ip}"
  done | awk 'NF && !seen[$0]++'
}

ntp_oneshot_set_clock() {
  local hosts out host ts
  hosts="$(ntp_collect_ips | tr '\n' ' ')"
  hosts="${hosts% }"
  [[ -n "${hosts}" ]] || return 1
  echo "[INFO] NTP one-shot 查询: ${hosts}"

  if command -v python3 >/dev/null; then
    out="$(NTP_TRY="${hosts}" NTP_WAIT="${NTP_WAIT_SEC:-8}" python3 - <<'PY'
import os, socket, struct, sys, time
UNIX = 2208988800
hosts = os.environ.get("NTP_TRY", "").split()
deadline = time.time() + int(os.environ.get("NTP_WAIT", "8"))
pkt = b"\x1b" + 47 * b"\x00"
last_err = "no reply"
while time.time() < deadline:
    for h in hosts:
        s = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(1.0)
            s.sendto(pkt, (h, 123))
            data = s.recvfrom(512)[0]
            if len(data) >= 48:
                t = struct.unpack("!12I", data[:48])[10] - UNIX
                if t >= 1704067200:
                    print("%s %d" % (h, int(t)))
                    sys.exit(0)
                last_err = "%s: ntp unix=%s" % (h, t)
        except Exception as e:
            last_err = "%s: %s" % (h, e)
        finally:
            if s is not None:
                s.close()
    time.sleep(1)
sys.stderr.write("NTP one-shot failed: %s\n" % last_err)
sys.exit(1)
PY
)" || return 1
    host="${out%% *}"
    ts="${out##* }"
    [[ "${ts}" =~ ^[0-9]+$ ]] || return 1
    date -u -s "@${ts}" >/dev/null || return 1
    echo "[INFO] NTP one-shot ${host} -> $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    return 0
  fi

  if command -v ntpdate >/dev/null; then
    # shellcheck disable=SC2086
    ntpdate -u -t 1 ${hosts} && return 0
  fi
  return 1
}

time_sync_ai_ntp_apply() {
  ntp_has_uplink || return 1
  ntp_oneshot_set_clock || return 1
  hwclock --systohc --utc || true
  echo "[INFO] 已写入 AI RTC，NTP 结束"
  return 0
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  time_sync_ai_ntp_apply
fi
EOF
sudo chmod +x /usr/local/sbin/time_sync_ai_ntp_once.sh

# 4) 启动脚本：mask NTP 后起 PTP。闸门只短等 SLAVE，超时放行，phc2sys 继续在后台收敛。
sudo tee /usr/local/sbin/time_sync_ai_bootstrap.sh >/dev/null <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
source /etc/default/time-sync-ai || true

LOCK=/run/time_sync_ai.lock
exec 9>"${LOCK}"
flock -n 9 || { echo "[WARN] AI bootstrap already running"; exit 0; }

STAMP=/run/time-sync-ai.bootstrapped
# 不能用 date mtime：1970 时 now-mtime 为负，会误判“刚完成”而跳过 NTP。
if [[ -f "${STAMP}" ]]; then
  y="$(date -u +%Y 2>/dev/null || echo 0)"
  up="$(cut -d. -f1 /proc/uptime 2>/dev/null || echo 0)"
  prev="$(cat "${STAMP}" 2>/dev/null || echo 0)"
  if [[ "${y}" =~ ^[0-9]+$ ]] && (( y >= 2024 )) &&
     [[ "${prev}" =~ ^[0-9]+$ ]] && (( up >= prev && up - prev < 30 )); then
    echo "[INFO] AI bootstrap 刚完成，跳过重复执行"
    exit 0
  fi
fi

echo "[INFO] AI bootstrap start"

mask_ntp_units() {
  local u
  for u in systemd-timesyncd.service chronyd.service chrony.service ntp.service ntpd.service \
           fake-hwclock.service fake-hwclock-save.service fake-hwclock.timer; do
    systemctl stop "$u" 2>/dev/null || true
    systemctl disable "$u" 2>/dev/null || true
    systemctl mask "$u" 2>/dev/null || true
  done
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

# ntp_has_uplink / ntp_oneshot / time_sync_ai_ntp_apply
# shellcheck disable=SC1091
source /usr/local/sbin/time_sync_ai_ntp_once.sh

run_ntp_once() {
  local wait_sec i
  echo "[INFO] AI 开机 NTP 只查一次，随后关掉，不与 PTP 同时改时钟"
  hwclock --hctosys --utc || true
  if clock_looks_valid && ! ntp_has_uplink; then
    echo "[INFO] 无外网且墙钟有效，使用 AI RTC $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    return 0
  fi
  wait_sec="${NTP_ROUTE_WAIT_SEC:-3}"
  if ! clock_looks_valid; then
    wait_sec="${NTP_INVALID_WAIT_SEC:-45}"
    echo "[INFO] 墙钟无效，最多等 WiFi/外网 ${wait_sec}s 做一次 NTP"
  fi
  for i in $(seq 1 "${wait_sec}"); do
    if ntp_has_uplink; then
      if time_sync_ai_ntp_apply; then
        return 0
      fi
      echo "[WARN] 有路由但 NTP 无应答"
      break
    fi
    if (( i == 1 || i % 10 == 0 )); then
      echo "[INFO] 等待 NTP 路由 ${i}/${wait_sec}s"
    fi
    sleep 1
  done
  echo "[WARN] 未从 NTP 校时，使用当前墙钟 $(date -u +%Y-%m-%dT%H:%M:%SZ)"
}

clock_looks_valid() {
  local y
  y="$(date -u +%Y 2>/dev/null || echo 0)"
  [[ "${y}" =~ ^[0-9]+$ ]] && (( y >= 2024 ))
}

is_ptp_slave() {
  local state
  if command -v pmc >/dev/null; then
    state="$(pmc -u -b 0 'GET PORT_DATA_SET' 2>/dev/null | awk '/portState/ {print $2; exit}')"
    [[ "${state}" == "SLAVE" ]]
    return
  fi
  journalctl -u ptp4l-slave.service -n 40 --no-pager 2>/dev/null | grep -q 'to SLAVE'
}

abs_offset_ns() {
  local raw rms
  if command -v pmc >/dev/null; then
    raw="$(pmc -u -b 0 'GET CURRENT_DATA_SET' 2>/dev/null | awk '/offsetFromMaster/ {print $2; exit}')"
    if [[ -n "${raw}" ]]; then
      awk -v v="${raw}" 'BEGIN {
        a = v + 0
        if (a < 0) a = -a
        if (v ~ /[.]/ && a < 10) a = a * 1e9
        printf "%.0f\n", a
      }'
      return
    fi
  fi
  rms="$(journalctl -u ptp4l-slave.service -n 30 --no-pager 2>/dev/null | awk '/rms[[:space:]]+[0-9]+/ {for (i=1;i<=NF;i++) if ($i=="rms") {print $(i+1); exit}}')"
  if [[ -n "${rms}" ]]; then
    echo "${rms}"
    return
  fi
  echo ""
}

offset_converged() {
  local offset limit
  offset="$(abs_offset_ns)"
  limit="${TIME_SYNC_OFFSET_NS:-1000000}"
  [[ -n "${offset}" ]] || return 1
  awk -v o="${offset}" -v l="${limit}" 'BEGIN { exit !(o <= l) }'
}

timedatectl set-timezone Asia/Shanghai || true
systemctl stop chronyd.service chrony.service ntp.service ntpd.service 2>/dev/null || true
systemctl disable chronyd.service chrony.service ntp.service ntpd.service 2>/dev/null || true
systemctl mask chronyd.service chrony.service ntp.service ntpd.service 2>/dev/null || true

if [[ "${USE_NTP_ON_AI:-1}" == "1" ]]; then
  run_ntp_once
else
  hwclock --hctosys --utc || true
fi
mask_ntp_units
timedatectl set-ntp false 2>/dev/null || true
wait_ptp_iface

systemctl restart ptp4l-slave.service
sleep 2
systemctl restart phc2sys-slave.service

echo "[INFO] AI 墙钟=$(date -u +%Y-%m-%dT%H:%M:%SZ) $(clock_looks_valid && echo 有效 || echo 无效)，PTP 已启动；GMK 会来取此时钟再升 Master"
st="$(timeout 3 pmc -u -b 0 'GET PORT_DATA_SET' 2>/dev/null | awk '/portState/ {print $2; exit}')"
echo "[INFO] 当前 portState=${st:-unknown}"
if [[ "${st}" == "MASTER" ]]; then
  echo "[INFO] AI 暂为 PTP Master，供 GMK 取时"
elif [[ "${st}" == "SLAVE" ]]; then
  echo "[INFO] 已见到 GMK；phc2sys 仅在本机墙钟无效或与 GMK 差≤2s 时跟随"
fi

cut -d. -f1 /proc/uptime >"${STAMP}" 2>/dev/null || echo 0 >"${STAMP}"
echo "[INFO] AI bootstrap done"
EOF
sudo chmod +x /usr/local/sbin/time_sync_ai_bootstrap.sh

sudo tee /usr/local/sbin/time_sync_ai_phc2sys.sh >/dev/null <<'EOF'
#!/usr/bin/env bash
# NTP 只在 AI。PTP 运行期 GMK 永远是 Master，AI 是 Slave。
# MASTER: 仅 GMK 未起时，把 NTP 墙钟顶在 PHC 上给 GMK 取。
# SLAVE:  跟随 GMK；本机 1970 时跟 GMK 恢复。
# HOLD:   本机已有 NTP 墙钟且与 GMK 差 >2s（GMK 还没取到），先不改 CLOCK_REALTIME。
set -euo pipefail
source /etc/default/time-sync-ai || true

: "${PHC2SYS_BIN:=phc2sys}"
: "${PHC_CTL_BIN:=phc_ctl}"
: "${IFACE:=eth0}"
: "${PTP_DEV:=/dev/ptp0}"
: "${TIME_SYNC_STEP_GUARD_SEC:=2}"

child=""
current=""

cleanup() {
  if [[ -n "${child}" ]]; then
    kill "${child}" 2>/dev/null || true
    wait "${child}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

port_state() {
  pmc -u -b 0 'GET PORT_DATA_SET' 2>/dev/null | awk '/portState/ {print $2; exit}'
}

phc_rt_offset_ns() {
  local raw
  # 典型输出: offset from CLOCK_REALTIME is 37000000000ns
  raw="$("${PHC_CTL_BIN}" "${PTP_DEV}" cmp 2>/dev/null | awk '{
    for (i = 1; i <= NF; i++) {
      if ($i ~ /^-?[0-9]+ns$/) { gsub(/ns/, "", $i); print $i; exit }
      if ($i ~ /^-?[0-9]+$/ && $(i-1) == "is") { print $i; exit }
    }
  }')"
  [[ -n "${raw}" ]] || return 1
  printf '%s\n' "${raw}"
}

utc_offset_sec() {
  local props utc
  props="$(pmc -u -b 0 'GET TIME_PROPERTIES_DATA_SET' 2>/dev/null || true)"
  utc="$(printf '%s\n' "${props}" | awk '/currentUtcOffset / {print $2; exit}')"
  if [[ "${utc}" =~ ^[0-9]+$ ]]; then
    echo "${utc}"
    return
  fi
  echo ""
}

clock_looks_valid() {
  local y
  y="$(date -u +%Y 2>/dev/null || echo 0)"
  [[ "${y}" =~ ^[0-9]+$ ]] && (( y >= 2024 ))
}

# PHC 可能是 TAI 或 UTC。正常锁定时 phc_ctl 约为 0 或 ±currentUtcOffset（约 ±37s）。
# 取 |raw|、|raw-utc|、|raw+utc| 的最小者，避免符号把 37s 算成 74s。
LAST_PHC_CMP=""
slave_step_safe() {
  local raw utc abs guard
  raw="$(phc_rt_offset_ns 2>/dev/null || true)"
  if [[ -z "${raw}" ]]; then
    LAST_PHC_CMP="phc_ctl 无偏移"
    return 1
  fi
  utc="$(utc_offset_sec)"
  if [[ -z "${utc}" ]]; then
    utc=37
  fi
  abs="$(awk -v r="${raw}" -v u="${utc}" 'BEGIN {
    r += 0
    u *= 1e9
    a = r; if (a < 0) a = -a
    d1 = r - u; if (d1 < 0) d1 = -d1
    d2 = r + u; if (d2 < 0) d2 = -d2
    m = a
    if (d1 < m) m = d1
    if (d2 < m) m = d2
    printf "%.0f\n", m
  }')"
  LAST_PHC_CMP="phc_cmp=${raw}ns utcOffset=${utc}s residual=${abs}ns"
  guard=$(( ${TIME_SYNC_STEP_GUARD_SEC:-2} * 1000000000 ))
  if awk -v a="${abs}" -v g="${guard}" 'BEGIN { exit !(a <= g) }'; then
    return 0
  fi
  if clock_looks_valid; then
    return 1
  fi
  LAST_PHC_CMP="${LAST_PHC_CMP} local=invalid，跟随 GMK"
  return 0
}

start_mode() {
  local mode="$1"
  cleanup
  child=""
  case "${mode}" in
    slave)
      "${PHC2SYS_BIN}" -s "${IFACE}" -c CLOCK_REALTIME -w -R 8 -S 0.5 -m &
      child=$!
      echo "[INFO] phc2sys mode=slave pid=${child} ${LAST_PHC_CMP}"
      ;;
    master)
      "${PHC2SYS_BIN}" -s CLOCK_REALTIME -c "${PTP_DEV}" -w -R 8 -S 0.5 -m &
      child=$!
      echo "[INFO] phc2sys mode=master pid=${child} (CLOCK_REALTIME -> PHC)"
      ;;
    hold)
      echo "[INFO] mode=hold ${LAST_PHC_CMP:-port=${st:-unknown}}（停 phc2sys，让 ptp4l 接管 PHC）"
      ;;
  esac
  current="${mode}"
}

while true; do
  st="$(port_state)"
  want=""
  case "${st}" in
    SLAVE)
      if slave_step_safe; then
        want=slave
      else
        want=hold
      fi
      ;;
    MASTER) want=master ;;
    *) want=hold ;;
  esac
  if [[ -n "${want}" && "${want}" != "${current}" ]]; then
    start_mode "${want}"
  fi
  if [[ -n "${child}" ]] && ! kill -0 "${child}" 2>/dev/null; then
    echo "[WARN] phc2sys 退出，重拉 mode=${current:-unknown}"
    current=""
    child=""
  fi
  sleep 1
done
EOF
sudo chmod +x /usr/local/sbin/time_sync_ai_phc2sys.sh

# 5) systemd 服务。ptp4l/phc2sys 不 enable，只由 bootstrap 拉起。
sudo tee /etc/systemd/system/ptp4l-slave.service >/dev/null <<'EOF'
[Unit]
Description=PTP4L AI Clock (slave to GMK, temporary master if GMK absent)
# PTP 只走板间 eth0，不依赖外网 network-online
After=sysinit.target

[Service]
Type=simple
EnvironmentFile=-/etc/default/time-sync-ai
# 部分 systemd 不展开 ExecStart 里的 ${VAR}，会把字面量当可执行文件 -> 203/EXEC
ExecStart=/bin/bash -c 'exec "${PTP4L_BIN}" -i "${IFACE}" -H -m -f /etc/linuxptp/ptp4l_ai.conf'
Restart=always
RestartSec=2
EOF

sudo tee /etc/systemd/system/phc2sys-slave.service >/dev/null <<'EOF'
[Unit]
Description=AI PHC2SYS (direction follows PTP port state)
After=ptp4l-slave.service
Requires=ptp4l-slave.service

[Service]
Type=simple
EnvironmentFile=-/etc/default/time-sync-ai
ExecStart=/usr/local/sbin/time_sync_ai_phc2sys.sh
Restart=always
RestartSec=2
EOF

sudo tee /etc/systemd/system/time-sync-ai.service >/dev/null <<'EOF'
[Unit]
Description=AI Time Sync Bootstrap
# 不绑定 network-online：无外网时该 target 会长时间阻塞或失败
After=sysinit.target

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/time_sync_ai_bootstrap.sh
RemainAfterExit=true
TimeoutStartSec=90

[Install]
WantedBy=multi-user.target
EOF

# 6) 生效
sudo systemctl daemon-reload
sudo systemctl enable time-sync-ai.service
echo "[INFO] 启动 time-sync-ai（无外网直接用 RTC，有 NTP 才校一次）"
sudo /usr/local/sbin/time_sync_ai_bootstrap.sh
sudo systemctl start time-sync-ai.service

echo "=== AI RESULT ==="
echo "IFACE=${IFACE} PTP_DEV=${PTP_DEV}"
timeout 5 timedatectl || true
date || true
systemctl is-enabled time-sync-ai.service || true
systemctl is-enabled ptp4l-slave.service phc2sys-slave.service 2>/dev/null || true
systemctl is-active ptp4l-slave.service phc2sys-slave.service time-sync-ai.service || true
systemctl is-enabled systemd-timesyncd chronyd ntp ntpd 2>/dev/null || true
ps -ef | grep -E "ptp4l|phc2sys|timesyncd" | grep -v grep || true
sudo systemctl status ptp4l-slave.service --no-pager -n 20 || true
sudo systemctl status phc2sys-slave.service --no-pager -n 20 || true
if command -v pmc >/dev/null; then
  timeout 3 pmc -u -b 0 'GET PARENT_DATA_SET' || true
  timeout 3 pmc -u -b 0 'GET CURRENT_DATA_SET' || true
  timeout 3 pmc -u -b 0 'GET PORT_DATA_SET' || true
fi
echo "========================================="
echo "AI 安装完成"
echo "========================================="
