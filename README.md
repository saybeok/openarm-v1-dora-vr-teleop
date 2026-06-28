# openarm-v1-dora-vr-teleop

OpenArm v1에서 Dora 기반 VR teleoperation을 테스트하기 위한 수정/설정 기록입니다.

기존 Dora/OpenArm VR 예제는 OpenArm v2 / OpenArm Cell / lifter 포함 MuJoCo scene 기준으로 구성되어 있었습니다.  
이 저장소는 OpenArm v1 양팔 모델에서 Quest VR 입력을 받아 MuJoCo v1 scene에서 동작하도록 수정한 내용을 정리합니다.

## 현재 상태

- Meta Quest VR pose 수신 확인
- Dora dataflow 실행 확인
- OpenArm v1 MuJoCo scene 로딩 확인
- `--ctrl` 제거 후 qpos direct mode에서 양팔 움직임 확인
- v1 gripper joint 2개 구조에 맞게 gripper qpos 동기화 수정
- 실물 OpenArm v1 연결은 아직 진행 전

## 참고 원본 프로젝트

- OpenArm: https://github.com/enactic/openarm
- OpenArm ROS2: https://github.com/enactic/openarm_ros2
- Dora OpenArm: https://github.com/enactic/dora-openarm
- OpenArm MuJoCo: https://github.com/enactic/openarm_mujoco
- Dora OpenArm Data Collection: https://github.com/enactic/dora-openarm-data-collection

## License / Attribution

This project is based on modifications and configuration work around the original OpenArm, Dora OpenArm, and OpenArm MuJoCo projects.

The original OpenArm-related projects are licensed under Apache License 2.0.  
Please keep the original copyright and license notices when redistributing modified code.

## Current physical robot status

- OpenArm v1 physical robot teleoperation confirmed.
- Quest → Dora IK → ROS2 bridge → joint_trajectory_controller chain works.
- Initial physical test was performed with gripper disabled.
- Recommended default is `DRY_RUN=1` for safety before each run.

## Status

This branch is an experimental OpenArm v1 VR teleoperation prototype using Dora, Meta Quest, MuJoCo, and ROS2.

It successfully demonstrates:
- Quest VR input reception
- Dora-based IK pipeline
- MuJoCo v1 preview using direct qpos updates
- ROS2 bridge to OpenArm v1 joint trajectory controllers
- Physical robot motion through CAN/ROS2

Known limitations:
- Physical robot tracking does not perfectly match MuJoCo preview.
- VR pose noise and IK output jitter make it unsuitable for high-quality imitation learning data collection.
- Gripper action control requires separate direct trigger handling.
- The project is kept as a technical reference and experiment archive, not a production data-collection stack.
