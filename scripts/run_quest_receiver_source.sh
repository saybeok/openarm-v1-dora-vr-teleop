#!/usr/bin/env bash
set -e

cd ~/dora_openarm/dora-openarm-data-collection
source .venv/bin/activate

export PYTHONPATH="$PWD/nodes/dora-openarm-vr/src:$PYTHONPATH"

exec python nodes/dora-openarm-vr/src/node/dora_openarm_quest_receiver/main.py
