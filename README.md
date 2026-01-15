# Robot Workspace

ROS2 멀티-레포지토리 로봇 워크스페이스

## 새 팀원 퀵스타트

```bash
# 1. 사전 요구사항: ROS2 및 vcstool 설치
sudo apt install python3-vcstool

# 2. 워크스페이스 생성 및 meta_repos 클론
mkdir -p ~/robot_ws && cd ~/robot_ws
git clone https://github.com/ingjae/meta_repos.git

# 3. 담당 로봇에 맞는 저장소 클론
vcs import < meta_repos/robot.repos   # Robot1 팀
vcs import < meta_repos/robot2.repos  # Robot2 팀

# 4. ROS2 의존성 설치
rosdep install --from-paths . --ignore-src -r -y

# 5. 빌드
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install

# 6. 환경 설정 (매 터미널마다 실행, 또는 .bashrc에 추가)
source ~/robot_ws/install/setup.bash

# 7. 실행 테스트
ros2 launch robot_main robot.launch.py    # Robot1
ros2 launch robot2_main robot2.launch.py  # Robot2
```

## 저장소 구조

| 저장소 | 설명 | 사용 |
|--------|------|------|
| `meta_repos` | .repos 매니페스트 파일 | 공통 |
| `robot_interfaces` | 커스텀 msg, srv, action 정의 | 공통 |
| `robot_common` | 공용 유틸리티 라이브러리 | 공통 |
| `robot_tools` | 개발/디버깅 도구 | 공통 |
| `robot_main` | Robot1 메인 노드 및 launch | Robot1 |
| `robot2_main` | Robot2 메인 노드 및 launch | Robot2 |

## .repos 파일 구조

```
meta_repos/
├── base.repos    # 공통 패키지만 (robot_common, robot_interfaces, robot_tools)
├── robot.repos   # 공통 + robot_main
└── robot2.repos  # 공통 + robot2_main
```

## 실행

### Robot1

```bash
ros2 launch robot_main robot.launch.py
ros2 launch robot_main robot.launch.py robot_name:=my_robot
```

### Robot2

```bash
ros2 launch robot2_main robot2.launch.py
ros2 launch robot2_main robot2.launch.py robot_name:=my_robot2
```

### 개별 노드 실행

```bash
# Robot1
ros2 run robot_main robot_node.py
ros2 run robot_main sensor_node.py

# Robot2
ros2 run robot2_main robot2_node.py
ros2 run robot2_main sensor_node.py
```

### 디버깅 도구

```bash
# 토픽 모니터링
ros2 run robot_tools topic_monitor.py --ros-args -p topic_name:=robot_status

# 키보드 텔레옵
ros2 run robot_tools cmd_vel_teleop.py
```

## vcstool 사용법

```bash
# 모든 저장소 상태 확인
vcs status

# 모든 저장소 pull
vcs pull

# 변경사항 확인
vcs diff

# 커밋 로그 확인
vcs log -l 3

# 현재 상태 export
vcs export > workspace.repos

# 커스텀 git 명령 실행
vcs custom --args <git-command>
```

## 커스텀 인터페이스

### Messages
- `RobotStatus.msg` - 로봇 상태 (state, battery, pose)
- `SensorData.msg` - 센서 데이터 (distances, temperature)

### Services
- `GetStatus.srv` - 로봇 상태 조회
- `SetMode.srv` - 동작 모드 설정 (MANUAL/AUTO/STANDBY)

### Actions
- `MoveToGoal.action` - 목표 위치로 이동

## 빌드 옵션

```bash
# 특정 패키지만 빌드
colcon build --packages-select robot_main

# 의존성 포함 빌드
colcon build --packages-up-to robot_main

# Release 빌드
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 테스트 실행
colcon test
colcon test-result --verbose
```
