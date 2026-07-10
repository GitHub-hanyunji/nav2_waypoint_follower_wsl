# Nav2 Waypoint / Single-Point Navigation Client

Nav2(Navigation2)의 Action 서버(`navigate_to_pose`, `follow_waypoints`)에 goal을 전송하여 로봇을 특정 지점 또는 여러 경유점(waypoint)을 따라 자율주행시키는 ROS 2 Action Client 패키지입니다.

## 구성 노드

| 파일 | 노드 이름 | 설명 |
|---|---|---|
| `singlePoint_publisher.cpp` | `single_goal_client` | 지정된 단일 좌표(x, y, yaw)로 `NavigateToPose` action goal을 1회 전송합니다. |
| `waypoint_publisher.cpp` | `lab_waypoint_client` | 미리 정의된 waypoint 리스트를 `FollowWaypoints` action goal로 전송하며, 키보드 입력을 통해 한 바퀴 주행 / 무한 반복 주행 / 일시정지 / 재개 / 정지를 제어합니다. |

## 의존성 (Dependencies)

- ROS 2 (Humble 이상 권장)
- [Nav2 (Navigation2)](https://navigation.ros.org/)
- `rclcpp`
- `rclcpp_action`
- `nav2_msgs`
- `geometry_msgs`
- `tf2`

`package.xml`에 아래 의존성이 포함되어 있어야 합니다.

```xml
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>nav2_msgs</depend>
<depend>geometry_msgs</depend>
<depend>tf2</depend>
```

## 빌드 방법

```bash
cd ~/your_ws/src
git clone <이 저장소 URL>
cd ~/your_ws
colcon build --packages-select nav2_waypoint_follow_wsl
source install/setup.bash
```



## 실행 방법

사전에 Nav2 스택(AMCL, map_server, bt_navigator 등)이 실행되어 있고 로봇이 맵 상에서 localize 되어 있어야 합니다.

### 1. 단일 목표점 주행

```bash
ros2 run nav2_waypoint_follow_wsl singlePoint_publisher
```

- 코드 내부에 하드코딩된 좌표 `(-2.0, 0.0, -3.13)` (x, y, yaw[rad])로 이동합니다.
- 목표 좌표를 바꾸려면 `send_single_goal(x, y, yaw)` 호출부를 수정하세요.

### 2. Waypoint 순환 주행

```bash
ros2 run nav2_waypoint_follow_wsl waypoint_publisher
```

실행 후 터미널에서 아래 키를 입력해 주행을 제어할 수 있습니다.

| 키 | 동작 |
|---|---|
| `1` | 한 바퀴만 주행 |
| `i` | 무한 반복 주행 (한 바퀴 완료 후 2초 대기 후 자동 재시작) |
| `s` | 현재 바퀴 완료 후 정지 (다음 바퀴 시작 안 함) |
| `q` | 즉시 정지 (현재 위치까지 진행한 waypoint 저장, action goal 취소) |
| `r` | `q`로 정지한 지점부터 주행 재개 |

## Waypoint 목록

`waypoint_publisher.cpp`의 `waypoints` 벡터에 아래와 같이 연구실 내부 순환 경로가 정의되어 있습니다. (단위: m, rad)

```
(-2.00,  0.00, -3.13)
(-3.10, -0.55, -2.03)
(-3.25, -1.44, -1.51)
(-3.03, -2.64, -0.41)
(-1.71, -3.02,  0.20)
(-0.70, -2.81,  0.28)
( 0.41, -2.23,  1.17)
( 0.00,  0.00,  3.14)
```

경로를 바꾸려면 이 벡터를 수정한 뒤 다시 빌드하면 됩니다.

## 참고 사항

- 모든 goal의 `frame_id`는 `"map"`으로 고정되어 있으며, Nav2가 정상 동작 중이어야 goal이 수락됩니다.
- `waypoint_publisher`는 100ms 주기 타이머로 키보드 입력을 폴링하며, `termios` 기반 non-blocking 입력(Linux/macOS) 또는 `_kbhit`/`_getch`(Windows)를 사용합니다.
- Action 서버(`navigate_to_pose`, `follow_waypoints`)가 5초 이내에 응답하지 않으면 에러 로그를 출력하고 goal 전송을 중단합니다.

## 📄 License

원하는 라이선스를 명시하세요. (예: MIT, Apache-2.0)
