#!/usr/bin/env bash
set -e

cd ~/dora_openarm/dora-openarm-data-collection

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/dora_openarm/ros2_dora_py310/bin/activate

export PYTHONPATH="$PWD/nodes/dora-openarm-ros2-v1/src:$PYTHONPATH"

exec python -m dora_openarm_ros2_v1.main
