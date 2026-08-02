#!/usr/bin/env bash
# Stream the Gazebo GUI to the browser via noVNC.
#
# Architecture:
#   Xvfb :99  ->  x11vnc :5900  ->  websockify :6080  ->  browser noVNC client
#
# Run once before launching the sim (or let arm_full.launch.py / arm_sim.launch.py
# with stream_gui:=true start it automatically). Idempotent: re-running keeps
# the existing servers and just re-prints status.
#
# Usage:  bash scripts/stream_gazebo_gui.sh [start|status|stop]
set -euo pipefail

DISPLAY_NUM=99
VNC_PORT=5900
WEB_PORT=6080
SCREEN="${SCREEN:-1280x800x24}"
NOVNC_DIR="${NOVNC_DIR:-/usr/share/novnc}"
PID_DIR="${PID_DIR:-/tmp/gz_stream}"

mkdir -p "$PID_DIR"

log() { echo "[gz-stream] $*"; }

find_pid() {
  case "$1" in
    xvfb)       pgrep -f "Xvfb :$DISPLAY_NUM " | head -1 ;;
    x11vnc)     pgrep -f "x11vnc.*-rfbport $VNC_PORT" | head -1 ;;
    websockify) pgrep -f "websockify.* $WEB_PORT " | head -1 ;;
  esac
}

is_running() {
  local pid
  pid="$(find_pid "$1")"
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

adopt() {
  # Record the actual running pid (handles orphans left without pidfiles)
  local pid
  pid="$(find_pid "$1")"
  if [ -n "$pid" ]; then
    echo "$pid" > "$PID_DIR/$1.pid"
    log "$1 found running (pid $pid), reusing it"
  fi
}

start_xvfb() {
  if is_running xvfb; then
    adopt xvfb
    log "Xvfb already running (pid $(cat "$PID_DIR/xvfb.pid"))"
    return
  fi
  Xvfb ":$DISPLAY_NUM" -screen 0 "$SCREEN" +extension GLX +render -nolisten tcp \
    >/dev/null 2>&1 &
  echo $! > "$PID_DIR/xvfb.pid"
  sleep 1
  log "Xvfb :$DISPLAY_NUM started (pid $(cat "$PID_DIR/xvfb.pid"))"
}

start_x11vnc() {
  if is_running x11vnc; then
    adopt x11vnc
    log "x11vnc already running (pid $(cat "$PID_DIR/x11vnc.pid"))"
    return
  fi
  # Unset Wayland vars: x11vnc 0.9.16 bails out when WAYLAND_DISPLAY is set
  # (e.g. WSLg container env) even though we target plain Xvfb.
  env -u WAYLAND_DISPLAY -u XDG_SESSION_TYPE -u WAYLAND_SOCKET \
    DISPLAY=":$DISPLAY_NUM" x11vnc -nopw -forever -shared -quiet \
    -rfbport "$VNC_PORT" -display ":$DISPLAY_NUM" \
    >"$PID_DIR/x11vnc.log" 2>&1 &
  echo $! > "$PID_DIR/x11vnc.pid"
  sleep 1
  log "x11vnc on :$VNC_PORT started (pid $(cat "$PID_DIR/x11vnc.pid"))"
}

start_websockify() {
  if is_running websockify; then
    adopt websockify
    log "websockify already running (pid $(cat "$PID_DIR/websockify.pid"))"
    return
  fi
  websockify --web "$NOVNC_DIR" "$WEB_PORT" localhost:$VNC_PORT \
    >"$PID_DIR/websockify.log" 2>&1 &
  echo $! > "$PID_DIR/websockify.pid"
  sleep 1
  log "websockify on :$WEB_PORT -> :$VNC_PORT started (pid $(cat "$PID_DIR/websockify.pid"))"
}

status() {
  echo "== Gazebo GUI stream status =="
  for p in xvfb x11vnc websockify; do
    if is_running "$p"; then
      adopt "$p"
      echo "  $p: RUNNING (pid $(cat "$PID_DIR/$p.pid"))"
    else
      echo "  $p: stopped"
    fi
  done
  echo "  noVNC client: http://localhost:$WEB_PORT/vnc.html?autoconnect=1&resize=scale"
}

stop() {
  log "stopping stream servers"
  for p in xvfb x11vnc websockify; do
    if is_running "$p"; then
      kill "$(cat "$PID_DIR/$p.pid")" 2>/dev/null || true
      rm -f "$PID_DIR/$p.pid"
      log "stopped $p"
    fi
  done
}

case "${1:-start}" in
  start)
    start_xvfb
    start_x11vnc
    start_websockify
    status
    ;;
  status) status ;;
  stop)   stop ;;
  *) echo "usage: $0 [start|status|stop]"; exit 1 ;;
esac
