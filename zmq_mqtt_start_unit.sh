#!/usr/bin/env bash

# 启动 Terminator，在一个窗口中水平并列 3 个等宽终端（目标每列 45），
# 并在各面板中进入指定目录与执行相应命令。无需 tmux。

set -euo pipefail

TITLE="ZMQ/MQTT Unit"
PANE_WIDTH="${PANE_WIDTH:-45}"  # 每个面板目标宽度（列数）
GEOM_COLS=$(( PANE_WIDTH * 3 ))   # 窗口列数（不含边框，近似）
GEOM_ROWS="${GEOM_ROWS:-30}"     # 窗口行数，可按需调整

# 路径与命令（可用环境变量覆盖）
PANE1_DIR="${PANE1_DIR:-$HOME/Documents/mqtt_ws/cpp_version}"
PANE1_CMD="${PANE1_CMD:-bash build.sh}"

PANE2_DIR="${PANE2_DIR:-$HOME/Documents/mqtt_ws/cpp_version/tests/scripts}"
PANE2_CMD="${PANE2_CMD:-}"  # 仅进入目录

PANE3_DIR="${PANE3_DIR:-$HOME/Documents/zmq_ws/cpp_version}"
PANE3_CMD="${PANE3_CMD:-bash start_production.sh}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[ERROR] 需要命令 '$1'，但未找到。请先安装。" >&2
    exit 1
  fi
}

require_cmd terminator

# 生成一个临时 Terminator 配置（不修改你现有配置）并定义布局与每个面板的启动命令
TMP_CONF="$(mktemp /tmp/terminator.zmq_mqtt_unit.XXXXXX.conf)"

cat >"${TMP_CONF}" <<EOF
[global_config]
  enabled_plugins = []
[keybindings]
[profiles]
  [[pane1]]
    use_custom_command = True
    custom_command = bash -lc 'cd "${PANE1_DIR}" && ${PANE1_CMD}; exec bash'
  [[pane2]]
    use_custom_command = True
    custom_command = bash -lc 'cd "${PANE2_DIR}"; exec bash'
  [[pane3]]
    use_custom_command = True
    custom_command = bash -lc 'cd "${PANE3_DIR}" && ${PANE3_CMD}; exec bash'
[layouts]
  [[zmq_mqtt_unit]]
    # 顶层水平分割：左（pane1） + 右（再水平分成 pane2/pane3）
    [[[child0]]]
      type = HPaned
      parent = ""
      position = 33
    [[[child1]]]
      type = Terminal
      parent = child0
      profile = pane1
    [[[child2]]]
      type = HPaned
      parent = child0
      position = 66
    [[[child3]]]
      type = Terminal
      parent = child2
      profile = pane2
    [[[child4]]]
      type = Terminal
      parent = child2
      profile = pane3
EOF

# 近似设置窗口几何尺寸，以便每面板 ~45 列
GEOM_OPT="--geometry=${GEOM_COLS}x${GEOM_ROWS}"

echo "[INFO] 启动 Terminator，窗口标题：${TITLE}"
terminator -u "${TMP_CONF}" -l zmq_mqtt_unit -T "${TITLE}" ${GEOM_OPT} &

sleep 1
if ! pgrep -f "terminator.*-l zmq_mqtt_unit" >/dev/null 2>&1; then
  echo "[WARN] Terminator 未保持前台显示，可能被窗口管理器最小化或命令失败。"
fi
