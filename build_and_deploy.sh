#!/bin/bash
set -euo pipefail

# ============================================================================
# build_and_deploy.sh — Milesight concentratord 全链路构建部署脚本
#
# 解决的缓存问题：
# 1. cross 派生 Docker 镜像缓存 → 删除后重建
# 2. 宿主机 /usr/local/ 旧 .a 文件 → 同步更新
# 3. cargo fingerprint 缓存 → 清除 .fingerprint + 二进制
# 4. Docker 容器内旧二进制 → 先删后下载 + MD5 校验
# 5. BusyBox wget URL 特殊字符 → 用引号包裹 + 退出码检查
# ============================================================================

HAL_DIR="/root/sx1302_hal_ms"
CS_DIR="/root/chirpstack-concentratord"
TARGET="aarch64-unknown-linux-musl"
BINARY="chirpstack-concentratord-sx1302"
BIN_PATH="${CS_DIR}/target/${TARGET}/release/${BINARY}"
OSS_KEY="kevin/${BINARY}-milesight-coldstart"
OSS_BUCKET="ursalink-resource-center"
OSS_ENDPOINT="oss-us-west-1.aliyuncs.com"
OSS_URL="https://${OSS_BUCKET}.${OSS_ENDPOINT}/${OSS_KEY}"

# Required env vars:
#   OSS_ACCESS_KEY_ID     — Alibaba Cloud AccessKey ID
#   OSS_ACCESS_KEY_SECRET — Alibaba Cloud AccessKey Secret
#
# Gateway list: HOST ROLE
GATEWAYS=(
    "192.168.45.67 relay"
    "192.168.45.169 border"
)
GW_USER="root"
GW_PASS="LoRaWAN@2018"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log()  { echo -e "${GREEN}[BUILD]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
fail() { echo -e "${RED}[FAIL]${NC} $1"; exit 1; }

# ============================================================================
# Step 1: Build HAL library
# ============================================================================
log "Step 1/7: Building HAL library..."

docker run --rm \
    -v "${HAL_DIR}:/hal/sx1302_hal_ms" \
    cross-milesight-patched:latest \
    bash -c "cd /hal/sx1302_hal_ms && ARCH=arm CROSS_COMPILE=aarch64-linux-musl- make clean libloragw 2>&1 | tail -3" \
    || fail "HAL build failed"

HAL_MD5=$(md5sum "${HAL_DIR}/libloragw/libloragw.a" | cut -d' ' -f1)
log "HAL .a MD5: ${HAL_MD5}"

# ============================================================================
# Step 2: Sync .a to ALL locations (host + Docker image)
# ============================================================================
log "Step 2/7: Syncing .a to all locations..."

# Host locations
cp "${HAL_DIR}/libloragw/libloragw.a" "/usr/local/aarch64-linux-musl/lib/libloragw-sx1302.a"
cp -r "${HAL_DIR}/libloragw/inc/"* "/usr/local/aarch64-linux-musl/include/libloragw-sx1302/" 2>/dev/null || true

HOST_MD5=$(md5sum /usr/local/aarch64-linux-musl/lib/libloragw-sx1302.a | cut -d' ' -f1)
[ "${HAL_MD5}" = "${HOST_MD5}" ] || fail "Host .a MD5 mismatch! HAL=${HAL_MD5} Host=${HOST_MD5}"

# Docker image (for cross builds)
docker rm -f hal-sync-tmp 2>/dev/null || true
docker run -d --name hal-sync-tmp cross-milesight-patched:latest sleep 60 >/dev/null
docker cp "${HAL_DIR}/libloragw/libloragw.a" hal-sync-tmp:/usr/local/aarch64-linux-musl/lib/libloragw-sx1302.a
docker cp "${HAL_DIR}/libloragw/inc/." hal-sync-tmp:/usr/local/aarch64-linux-musl/include/libloragw-sx1302/
docker commit hal-sync-tmp cross-milesight-patched:latest >/dev/null
docker rm -f hal-sync-tmp >/dev/null

DOCKER_MD5=$(docker run --rm cross-milesight-patched:latest md5sum /usr/local/aarch64-linux-musl/lib/libloragw-sx1302.a | cut -d' ' -f1)
[ "${HAL_MD5}" = "${DOCKER_MD5}" ] || fail "Docker .a MD5 mismatch! HAL=${HAL_MD5} Docker=${DOCKER_MD5}"

log "All .a files synced (MD5: ${HAL_MD5})"

# ============================================================================
# Step 3: Purge cross Docker image cache
# ============================================================================
log "Step 3/7: Purging cross Docker image cache..."

OLD_IMAGES=$(docker images "localhost/cross-rs/cross-custom-chirpstack-concentratord:${TARGET}-*" -q 2>/dev/null)
if [ -n "${OLD_IMAGES}" ]; then
    echo "${OLD_IMAGES}" | xargs -r docker rmi -f 2>/dev/null || true
    log "Removed $(echo "${OLD_IMAGES}" | wc -l) cached cross images"
else
    log "No cached cross images to remove"
fi

# ============================================================================
# Step 4: Clean cargo build artifacts + rebuild concentratord
# ============================================================================
log "Step 4/7: Building concentratord (clean)..."

cd "${CS_DIR}"

# Remove binary + fingerprints to force relink
rm -f "${BIN_PATH}"
rm -rf "target/${TARGET}/release/.fingerprint/libloragw-sx1302"*
rm -rf "target/${TARGET}/release/.fingerprint/chirpstack-concentratord-sx1302"*
rm -rf "target/${TARGET}/release/deps/libloragw_sx1302"*
rm -rf "target/${TARGET}/release/deps/libchirpstack_concentratord_sx1302"*

# Touch build.rs to trigger rebuild
touch libloragw-sx1302/build.rs

export PATH="${HOME}/.cargo/bin:${PATH}"
cross build --target "${TARGET}" --release -p chirpstack-concentratord-sx1302 2>&1 | grep -E "Compiling|Finished|error"

[ -f "${BIN_PATH}" ] || fail "Binary not produced!"

NEW_MD5=$(md5sum "${BIN_PATH}" | cut -d' ' -f1)
log "Binary MD5: ${NEW_MD5}"

# Verify new C strings are in the binary
COLD_START_COUNT=$(strings "${BIN_PATH}" | grep -c "cold-start" || true)
if [ "${COLD_START_COUNT}" -eq 0 ]; then
    fail "Binary does NOT contain new C code! Check link-search path in build.rs"
fi
log "Verified: ${COLD_START_COUNT} 'cold-start' strings in binary"

# ============================================================================
# Step 5: Upload to OSS
# ============================================================================
log "Step 5/7: Uploading to OSS..."

python3 -c "
import oss2
auth = oss2.Auth(os.environ['OSS_ACCESS_KEY_ID'], os.environ['OSS_ACCESS_KEY_SECRET'])
bucket = oss2.Bucket(auth, 'https://${OSS_ENDPOINT}', '${OSS_BUCKET}')
result = bucket.put_object_from_file('${OSS_KEY}', '${BIN_PATH}')
assert result.status == 200, f'OSS upload failed: {result.status}'
# Verify uploaded file
meta = bucket.head_object('${OSS_KEY}')
print(f'Uploaded: {meta.content_length} bytes, etag: {meta.etag}')
" || fail "OSS upload failed"

log "OSS URL: ${OSS_URL}"

# ============================================================================
# Step 6: Deploy to gateways
# ============================================================================
log "Step 6/7: Deploying to gateways..."

deploy_gateway() {
    local HOST=$1
    local ROLE=$2

    echo -e "\n${GREEN}--- Deploying to ${HOST} (${ROLE}) ---${NC}"

    # Use paramiko for reliable SSH
    python3 << PYEOF
import paramiko, time, sys

def gw_exec(host, cmd, timeout=120):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(host, username='${GW_USER}', password='${GW_PASS}',
                timeout=10, look_for_keys=False, allow_agent=False)
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode().strip()
    err = stderr.read().decode().strip()
    rc = stdout.channel.recv_exit_status()
    ssh.close()
    return out, err, rc

dcmd = '/usr/bin/docker/docker'
host = '${HOST}'
expected_md5 = '${NEW_MD5}'

# 6a: Stop container
print("  [1/5] Stopping container...")
gw_exec(host, f'{dcmd} stop chirpstack-mesh 2>/dev/null')

# 6b: Start container (to get a fresh filesystem)
gw_exec(host, f'{dcmd} start chirpstack-mesh')
time.sleep(2)

# 6c: Remove old binary + download new one
print("  [2/5] Downloading new binary...")
out, err, rc = gw_exec(host,
    f"""{dcmd} exec chirpstack-mesh sh -c '
        rm -f /opt/chirpstack/binaries/chirpstack-concentratord-sx1302 &&
        wget -q -O /opt/chirpstack/binaries/chirpstack-concentratord-sx1302 "${OSS_URL}" &&
        chmod +x /opt/chirpstack/binaries/chirpstack-concentratord-sx1302
    '""")
if rc != 0:
    print(f"  DOWNLOAD FAILED: {err[:200]}")
    sys.exit(1)

# 6d: Verify MD5
out, err, rc = gw_exec(host,
    f'{dcmd} exec chirpstack-mesh md5sum /opt/chirpstack/binaries/chirpstack-concentratord-sx1302')
actual_md5 = out.split()[0] if out else "NONE"
print(f"  [3/5] MD5: {actual_md5}")
if actual_md5 != expected_md5:
    print(f"  MD5 MISMATCH! Expected={expected_md5} Got={actual_md5}")
    sys.exit(1)

# 6e: Hot-switch: start pkt_fwd, wait, kill -9
print("  [4/5] Hot-switch: pkt_fwd init → kill -9...")
gw_exec(host, f'{dcmd} stop chirpstack-mesh')
gw_exec(host, '/etc/init.d/lora_pkt_fwd start')
time.sleep(5)
gw_exec(host, 'killall -9 lora_pkt_fwd 2>/dev/null')
gw_exec(host, 'ubus call service delete \'{"name":"lora_pkt_fwd"}\' 2>/dev/null')
gw_exec(host, '/etc/init.d/lora_pkt_fwd disable 2>/dev/null')
time.sleep(1)

# 6f: Start concentratord
print("  [5/5] Starting concentratord...")
gw_exec(host, f'{dcmd} start chirpstack-mesh')
time.sleep(15)

# 6g: Verify
out, _, _ = gw_exec(host,
    f'{dcmd} exec chirpstack-mesh sh -c "cat /tmp/mesh.log 2>/dev/null | grep \'stats event\' | tail -1"')
print(f"  Stats: {out[:200] if out else 'NO STATS YET'}")

out, _, _ = gw_exec(host,
    f'{dcmd} exec chirpstack-mesh sh -c "cat /tmp/mesh.log 2>/dev/null | grep -c \'Frame received\'"')
print(f"  Frames received: {out}")

print(f"  ✅ ${HOST} (${ROLE}) deployed successfully")
PYEOF
}

for gw in "${GATEWAYS[@]}"; do
    deploy_gateway ${gw}
done

# ============================================================================
# Step 7: Final verification
# ============================================================================
log "Step 7/7: Final verification (waiting 30s for stats)..."
sleep 30

for gw in "${GATEWAYS[@]}"; do
    HOST=$(echo ${gw} | cut -d' ' -f1)
    ROLE=$(echo ${gw} | cut -d' ' -f2)

    python3 << PYEOF
import paramiko
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('${HOST}', username='${GW_USER}', password='${GW_PASS}',
            timeout=10, look_for_keys=False, allow_agent=False)
dcmd = '/usr/bin/docker/docker'
_, stdout, _ = ssh.exec_command(
    f'{dcmd} exec chirpstack-mesh sh -c "cat /tmp/mesh.log 2>/dev/null | grep \'stats event\' | tail -2"')
print(f"${HOST} (${ROLE}): {stdout.read().decode().strip()[:200]}")
ssh.close()
PYEOF
done

log "=== Build & Deploy Complete ==="
log "Binary: ${BIN_PATH}"
log "MD5:    ${NEW_MD5}"
log "OSS:    ${OSS_URL}"
