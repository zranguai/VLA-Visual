#!/usr/bin/env bash

set -u

LOCAL_HOST="127.0.0.1"
LOCAL_PORT="6969"
REMOTE_HOST="localhost"
REMOTE_PORT="6969"
SSH_HOST="139.196.33.0"
SSH_PORT="1017"
SSH_USER="root"
IDENTITY_FILE="${HOME}/.ssh/id_rsa"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="${SCRIPT_DIR}/autossh.log"

if ! command -v autossh >/dev/null 2>&1; then
  echo "错误：未安装 autossh。" >&2
  exit 1
fi

if [[ ! -r "${IDENTITY_FILE}" ]]; then
  echo "错误：SSH 私钥不存在或不可读：${IDENTITY_FILE}" >&2
  exit 1
fi

if nc -z "${LOCAL_HOST}" "${LOCAL_PORT}" >/dev/null 2>&1; then
  echo "本地端口 ${LOCAL_HOST}:${LOCAL_PORT} 已在监听。"
  exit 0
fi

: >"${LOG_FILE}"

AUTOSSH_GATETIME=0 \
AUTOSSH_LOGFILE="${LOG_FILE}" \
autossh -M 0 \
  -f \
  -N \
  -L "${LOCAL_HOST}:${LOCAL_PORT}:${REMOTE_HOST}:${REMOTE_PORT}" \
  "${SSH_USER}@${SSH_HOST}" \
  -p "${SSH_PORT}" \
  -i "${IDENTITY_FILE}" \
  -o "BatchMode yes" \
  -o "ConnectTimeout 10" \
  -o "ServerAliveInterval 10" \
  -o "ServerAliveCountMax 3" \
  -o "TCPKeepAlive yes" \
  -o "ExitOnForwardFailure yes" \
  -o "StrictHostKeyChecking accept-new" \
  -o "LogLevel ERROR" \
  2>>"${LOG_FILE}"

for _ in {1..10}; do
  if nc -z "${LOCAL_HOST}" "${LOCAL_PORT}" >/dev/null 2>&1; then
    echo "SSH 隧道已建立：${LOCAL_HOST}:${LOCAL_PORT} -> ${SSH_HOST}:${REMOTE_PORT}"
    exit 0
  fi
  sleep 1
done

echo "错误：SSH 隧道未能在 10 秒内建立，日志如下：" >&2
cat "${LOG_FILE}" >&2
exit 1
