#!/usr/bin/env bash

# 自动远程登录并在目标主机启动指定命令。
# 默认使用 sshpass 进行密码式登录；若未安装，则回退为普通 ssh（需已配置密钥）。
# 远端以 nohup 后台运行，日志写入 /tmp/vda5050_gateway.log。

set -euo pipefail

HOST="192.168.1.102"
USER="zl"
PASS="${PASS:-zl1234}"
REMOTE_CMD="/opt/xmover/vda5050_gateway/vda5050_gateway.launch"
REMOTE_LOG="/tmp/vda5050_gateway.log"

echo "[INFO] Connecting to ${USER}@${HOST} and starting: ${REMOTE_CMD}" 

SSH_OPTS=(
  -o StrictHostKeyChecking=no
  -o ServerAliveInterval=30
  -o ServerAliveCountMax=3
  -o ConnectTimeout=10
)

REMOTE_RUN_CMD="bash -lc 'nohup \"${REMOTE_CMD}\" >> \"${REMOTE_LOG}\" 2>&1 & echo \$! > /tmp/vda5050_gateway.pid'"

if command -v sshpass >/dev/null 2>&1; then
  echo "[INFO] Using sshpass for password-based login"
  sshpass -p "${PASS}" ssh "${SSH_OPTS[@]}" "${USER}@${HOST}" "${REMOTE_RUN_CMD}"
else
  echo "[WARN] sshpass not found. Falling back to plain ssh (use key-based auth or install sshpass)."
  ssh "${SSH_OPTS[@]}" "${USER}@${HOST}" "${REMOTE_RUN_CMD}"
fi

echo "[OK] Remote command started. Remote log: ${REMOTE_LOG}"
