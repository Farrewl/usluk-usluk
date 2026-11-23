#!/usr/bin/env bash
# ==============================================================================
# Aterkia ASV - Competition Launcher (QGC / MAVLink Forwarding + Relay + Bridge + Core)
# ==============================================================================
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"

export PUBLISHER_TOKEN="${PUBLISHER_TOKEN:-aterkia_pub_secure_2026}"
export ADMIN_TOKEN="${ADMIN_TOKEN:-aterkia_admin_secure_2026}"
export MONITOR_DIR="$REPO_DIR/data/monitor"
export RELAY_URL="ws://127.0.0.1:3000/ws"

# MAVLink endpoint options (Pixhawk on serial or UDP SITL / QGC forwarding)
# Sesuaikan port serial Pixhawk atau endpoint UDP (misal mavproxy / QGC)
MAV_PORT="${MAV_PORT:-/dev/ttyACM0}"
BAUD_RATE="${BAUD_RATE:-57600}"
QGC_UDP_TARGET="${QGC_UDP_TARGET:-127.0.0.1:14550}"

echo "[1/4] Menjalankan WebSocket Relay..."
./telemetry/relay/target/release/asv-relay > /tmp/asv_relay.log 2>&1 &
RELAY_PID=$!

echo "[2/4] Menjalankan Telemetry Bridge..."
./telemetry/bridge/target/release/asv-bridge > /tmp/asv_bridge.log 2>&1 &
BRIDGE_PID=$!

echo "[3/4] Menjalankan MAVProxy / MAVLink Router (untuk QGC & Offboard sync)..."
# MAVProxy menghubungkan Pixhawk serial ke QGC via UDP dan internal loop
if command -v mavproxy.py &> /dev/null; then
    echo "Menggunakan MAVProxy untuk jembatan QGC & Offboard..."
    mavproxy.py --master="$MAV_PORT,$BAUD_RATE" --out="udp:$QGC_UDP_TARGET" --out="udp:127.0.0.1:14551" > /tmp/asv_mavproxy.log 2>&1 &
    MAV_PID=$!
else
    echo "MAVProxy tidak ditemukan, menggunakan jalur MAVLink internal app/navigator..."
    MAV_PID=""
fi

echo "[4/4] Menjalankan Program Utama Kapal (Aterkia ASV Core)..."
python3 main.py > /tmp/asv_navigator.log 2>&1 &
NAV_PID=$!

echo "=========================================================="
echo " Aterkia ASV SIAP BERTANDING (Otonom + QGC + Relay Aktif)"
echo " - Relay PID   : $RELAY_PID"
echo " - Bridge PID  : $BRIDGE_PID"
echo " - MAVLink PID : ${MAV_PID:-N/A}"
echo " - Nav Core PID: $NAV_PID"
echo " Log tersimpan di /tmp/asv_*.log"
echo "=========================================================="

cleanup() {
    echo "Menghentikan semua servis ASV..."
    kill $RELAY_PID $BRIDGE_PID $NAV_PID ${MAV_PID:-} 2>/dev/null || true
    exit 0
}

trap cleanup INT TERM
wait
