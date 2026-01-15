# Robot Workspace

ROS2 멀티-레포지토리 로봇 워크스페이스

## 새 팀원 퀵스타트

```bash
# 1. 사전 요구사항: ROS2 및 vcstool 설치
sudo apt install python3-vcstool

# 2. 워크스페이스 생성 및 모든 저장소 클론
mkdir -p ~/robot_ws && cd ~/robot_ws
git clone https://github.com/ingjae/meta_repos.git
vcs import < meta_repos/robot.repos

# 3. ROS2 의존성 설치
rosdep install --from-paths . --ignore-src -r -y

# 4. 빌드
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install

# 5. 환경 설정 (매 터미널마다 실행, 또는 .bashrc에 추가)
source ~/robot_ws/install/setup.bash

# 6. 실행 테스트
ros2 launch robot_main robot.launch.py
```

## 저장소 구조

| 저장소 | 설명 |
|--------|------|
| `meta_repos` | .repos 매니페스트 파일 |
| `robot_interfaces` | 커스텀 msg, srv, action 정의 |
| `robot_common` | 공용 유틸리티 라이브러리 |
| `robot_main` | 메인 로봇 노드 및 launch 파일 |
| `robot_tools` | 개발/디버깅 도구 |

## 설치

### 1. 워크스페이스 클론

```bash
mkdir -p ~/robot_ws && cd ~/robot_ws
vcs import < https://raw.githubusercontent.com/ingjae/meta_repos/main/robot.repos
```

또는 직접 export된 repos 사용:

```bash
vcs import << EOF
repositories:
  robot_common:
    type: git
    url: https://github.com/ingjae/robot_common.git
    version: main
  robot_interfaces:
    type: git
    url: https://github.com/ingjae/robot_interfaces.git
    version: main
  robot_main:
    type: git
    url: https://github.com/ingjae/robot_main.git
    version: main
  robot_tools:
    type: git
    url: https://github.com/ingjae/robot_tools.git
    version: main
EOF
```

### 2. 의존성 설치

```bash
cd ~/robot_ws
rosdep install --from-paths . --ignore-src -r -y
```

### 3. 빌드

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 실행

### 로봇 시스템 실행

```bash
ros2 launch robot_main robot.launch.py
```

파라미터 지정:

```bash
ros2 launch robot_main robot.launch.py robot_name:=my_robot
```

### 개별 노드 실행

```bash
# 로봇 노드
ros2 run robot_main robot_node.py

# 센서 노드
ros2 run robot_main sensor_node.py
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
