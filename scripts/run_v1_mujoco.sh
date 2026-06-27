#!/usr/bin/env bash
set -e

cd "$(dirname "$0")/.."

source .venv/bin/activate

export DISPLAY=${DISPLAY:-:0}
export MUJOCO_GL=${MUJOCO_GL:-glfw}

pkill -9 -f dora 2>/dev/null || true
pkill -9 -f mujoco 2>/dev/null || true

dora build dataflow-vr-mujoco-v1.yaml
dora run dataflow-vr-mujoco-v1.yaml
