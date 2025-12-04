# coordinate_transformation

ROS2 픽셀-월드 좌표 변환 및 로봇 제어 패키지

## 🎯 주요 기능

- **픽셀-월드 좌표 변환**: 카메라 이미지 픽셀 좌표를 3D 월드 좌표로 변환
- **타겟 추적**: YOLO 검출 결과의 중심점을 자동으로 받아 처리
- **로봇 제어 인터페이스**: 변환된 좌표를 로봇 제어 시스템으로 전송
- **TF 브로드캐스팅**: 변환된 좌표를 TF로 발행하여 시각화 지원
- **서비스 기반 통신**: ROS2 서비스로 모듈화된 구조

## 📦 패키지 구성

### 노드

#### 1. `pixel_to_world`
카메라 픽셀 좌표를 월드 좌표로 변환하는 메인 노드

**기능:**
- YOLO 노드로부터 타겟 중심점 수신 (서비스)
- 카메라 내부 파라미터 및 이미지 정보 구독
- 픽셀 좌표 → 카메라 좌표 → 월드 좌표 변환
- 변환된 좌표를 TF로 브로드캐스트
- 로봇 제어 서버로 좌표 전송

**파라미터:**
- `image_topic` (기본값: `/camera/camera/color/image_raw`) - Color 이미지 토픽
- `camera_info_topic` (기본값: `/camera/camera/color/camera_info`) - 카메라 정보 토픽
- `target_frame` (기본값: `world`) - 타겟 프레임 이름

**서비스 서버:**
- `/send_target_center` (system_interfaces/srv/TargetCenter) - YOLO로부터 타겟 중심점 수신

**서비스 클라이언트:**
- `/move_to_point` (system_interfaces/srv/MoveToPoint) - 로봇 제어 서버로 좌표 전송

**구독 토픽:**
- `/camera/camera/color/image_raw` (sensor_msgs/Image) - Color 이미지
- `/camera/camera/color/camera_info` (sensor_msgs/CameraInfo) - 카메라 내부 파라미터

**TF 발행:**
- `world` → `target_frame` - 변환된 타겟 위치

#### 2. `move_to_point_server`
`pixel_to_world` 노드 테스트를 위한 서비스 서버

**기능:**
- `pixel_to_world` 노드로부터 월드 좌표 수신
- 수신된 좌표를 로그로 출력하여 변환 결과 확인
- 실제 로봇 제어가 없을 때 테스트용으로 사용

**서비스 서버:**
- `/move_to_point` (system_interfaces/srv/MoveToPoint) - 타겟 위치 수신 (테스트용)

## 🔧 의존성

### ROS2 패키지
```bash
sudo apt install ros-${ROS_DISTRO}-rclpy \
                 ros-${ROS_DISTRO}-sensor-msgs \
                 ros-${ROS_DISTRO}-geometry-msgs \
                 ros-${ROS_DISTRO}-cv-bridge \
                 ros-${ROS_DISTRO}-tf2-ros \
                 ros-${ROS_DISTRO}-tf2-geometry-msgs

# 워크스페이스 의존 패키지
# - system_interfaces: 커스텀 서비스 정의
```

### Python 패키지
```bash
pip3 install numpy opencv-python
```

## 🚀 빌드 및 설치

```bash
cd ~/colcon_ws

# 의존 패키지 먼저 빌드
colcon build --packages-select system_interfaces

# coordinate_transformation 빌드
colcon build --packages-select coordinate_transformation

source install/setup.bash
```

## 📖 사용법

### 🎬 기본 실행

#### 1. 테스트 서버 실행 (pixel_to_world 테스트용)
```bash
ros2 run coordinate_transformation move_to_point_server
```

#### 2. 픽셀-월드 변환 노드 실행
```bash
# 기본 실행
ros2 run coordinate_transformation pixel_to_world

# 파라미터 설정
ros2 run coordinate_transformation pixel_to_world --ros-args \
  -p image_topic:=/my_camera/color/image_raw \
  -p camera_info_topic:=/my_camera/color/camera_info \
  -p target_frame:=base_link
```

### 🔄 통합 실행 (yolo_realsense 패키지와 함께)

`yolo_realsense` 패키지의 런치 파일을 사용하면 자동 실행됩니다:

```bash
ros2 launch yolo_realsense yolo_all.launch.py
```

이 명령은 다음을 모두 실행합니다:
1. RealSense 카메라
2. YOLO Depth Viewer
3. Pixel to World 변환 노드

### 📡 서비스 호출 예시

#### 타겟 중심점 전송 (일반적으로 YOLO 노드가 자동 호출)
```bash
ros2 service call /send_target_center system_interfaces/srv/TargetCenter \
  "{center_u: 320, center_v: 240, depth: 1.5}"
```

#### 변환된 좌표 확인 (테스트 서버)
```bash
# move_to_point_server를 실행하고 있으면
# 로그에서 pixel_to_world가 보낸 월드 좌표를 확인할 수 있습니다
# 예: "📥 MoveToPoint 요청 수신: x = 0.523, y = 0.031, z = 0.980"
```

## 🧮 좌표 변환 원리

### 1. 픽셀 → 카메라 좌표
```
X_cam = (u - cx) * depth / fx
Y_cam = (v - cy) * depth / fy
Z_cam = depth
```

### 2. 카메라 → 월드 좌표
```
P_world = R_world_cam @ P_cam + T_cam
```

여기서:
- `(u, v)`: 픽셀 좌표
- `depth`: 깊이 값 (미터)
- `(cx, cy)`: 주점 (principal point)
- `(fx, fy)`: 초점 거리 (focal length)
- `R_world_cam`: 카메라-월드 회전 행렬
- `T_cam`: 카메라 위치 (월드 기준)

### 3. 카메라 설정

현재 기본 설정:
```python
# 카메라 위치 (월드 좌표계)
camera_position = [0.53, 0.03, 0.98]  # [x, y, z] 미터

# 카메라 회전 (오일러 각)
theta_x = π       # X축 회전 (180도)
theta_y = 0.0     # Y축 회전 (0도)
theta_z = π/2     # Z축 회전 (90도)
```

## 🔍 TF 시각화

RViz에서 변환된 좌표를 시각화할 수 있습니다:

```bash
# RViz 실행
rviz2

# Fixed Frame을 'world'로 설정
# Add → TF → OK
# target_frame이 표시됨
```

## 📊 워크플로우

```
┌─────────────────┐
│  YOLO Detector  │
│  (yolo_realsense)│
└────────┬────────┘
         │ TargetCenter 서비스 호출
         │ (중심점 u, v, depth)
         ↓
┌─────────────────────┐
│  pixel_to_world     │
│  1. 픽셀→카메라 좌표 │
│  2. 카메라→월드 좌표 │
│  3. TF 브로드캐스트  │
└────────┬────────────┘
         │ MoveToPoint 서비스 호출
         │ (월드 좌표 x, y, z)
         ↓
┌─────────────────────┐
│ move_to_point_server│
│  (테스트/실제 로봇)  │
│  - 테스트: 좌표 출력 │
│  - 실제: 로봇 제어   │
└─────────────────────┘
```

## 🛠️ 커스터마이징

### 카메라 위치/회전 변경

`pixel2world.py` 파일에서 카메라 파라미터를 수정하세요:

```python
# 카메라 위치 (미터 단위)
self.camera_position = np.array([x, y, z])

# 회전 각도 (라디안)
theta_x = ...  # X축 회전
theta_y = ...  # Y축 회전
theta_z = ...  # Z축 회전
```

### 실제 로봇 제어 연결

테스트 서버 대신 실제 로봇 제어 서버를 만들려면:

**방법 1: move_to_point_server.py 수정**
```python
def handle_move_to_point(self, request, response):
    target = request.target_position
    
    # 현재: 테스트용 로그 출력
    self.get_logger().info(f'좌표 수신: x={target.x:.3f}, y={target.y:.3f}, z={target.z:.3f}')
    
    # 실제 로봇 제어 코드로 변경
    # 예: MoveIt, dsr, navigation2 등
    # move_group.set_position_target([target.x, target.y, target.z])
    # result = move_group.go()
    
    response.success = True  # 또는 result에 따라
    response.message = f"로봇이 ({target.x:.3f}, {target.y:.3f}, {target.z:.3f})로 이동했습니다"
    return response
```

**방법 2: 별도의 로봇 제어 노드 생성**
```bash
# 기존 테스트 서버 대신 실제 로봇 제어 노드 실행
ros2 run your_robot_package robot_control_server
```

## 🛠️ 트러블슈팅

### 서비스가 연결되지 않을 때
```bash
# 서비스 목록 확인
ros2 service list

# 서비스 타입 확인
ros2 service type /send_target_center
ros2 service type /move_to_point
```

### 카메라 정보가 수신되지 않을 때
```bash
# 카메라 토픽 확인
ros2 topic list | grep camera

# CameraInfo 확인
ros2 topic echo /camera/camera/color/camera_info --once
```

### TF가 표시되지 않을 때
```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# 특정 TF 확인
ros2 run tf2_ros tf2_echo world target_frame
```

## 📝 라이선스
Apache License 2.0

## 👤 메인테이너
- jiwoo (jiwoo@todo.todo)

## 🔗 관련 패키지
- `yolo_realsense`: YOLO 객체 검출 및 깊이 측정
- `system_interfaces`: 커스텀 메시지/서비스 정의
