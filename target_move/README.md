# target_move

ROS 2 패키지로, MoveIt2를 사용하여 로봇 팔의 엔드 이펙터를 지정된 위치로 이동시키는 서비스를 제공합니다.

## 개요

이 패키지는 Doosan 로봇(e0509)의 manipulator를 제어하기 위한 서비스 서버를 제공합니다. 클라이언트가 목표 위치(x, y, z)를 서비스 요청으로 보내면, MoveIt2를 사용하여 경로를 계획하고 실행합니다.

## 기능

- **서비스 기반 제어**: `system_interfaces/srv/MoveToPoint` 서비스를 통해 목표 위치 수신
- **자동 경로 계획**: MoveIt2를 사용한 충돌 회피 경로 계획
- **고정된 엔드 이펙터 방향**: 엔드 이펙터가 항상 아래를 향하도록 설정 (X축 기준 180도 회전)
- **Planning Group**: `manipulator` 그룹 사용
- **좌표계**: `base_link` 기준

## 의존성

- ROS 2 Jazzy
- MoveIt2
- `system_interfaces` (custom service 정의)
- `dsr_moveit_config_e0509` (Doosan E0509 로봇 설정)

## 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select target_move
source install/setup.bash
```

## 실행

### 서비스 서버 실행

```bash
ros2 launch target_move target_move_dsr.launch.py
```

이 명령은 다음을 수행합니다:
- MoveIt 설정 로드 (e0509 모델)
- target_move_node 실행
- 서비스 서버 `/move_to_point` 활성화

### 서비스 호출

다른 터미널에서:

```bash
ros2 service call /move_to_point system_interfaces/srv/MoveToPoint \
  "{target_position: {x: 0.0, y: 0.0, z: 0.8}}"
```

## 서비스 인터페이스

### MoveToPoint.srv

**Request:**
```
geometry_msgs/Point target_position
  float64 x
  float64 y
  float64 z
```

**Response:**
```
bool success
string message
```

## 설정 파라미터

코드 내부에 하드코딩된 MoveIt 설정:

- `setPlanningTime(10.0)`: 최대 계획 시간 10초
- `setNumPlanningAttempts(10)`: 계획 시도 횟수 10회
- `setGoalPositionTolerance(0.01)`: 위치 허용 오차 1cm
- `setGoalOrientationTolerance(0.1)`: 방향 허용 오차 0.1 라디안

## 노드 정보

### target_move_node

**노드 이름:** `target_move_server`

**제공 서비스:**
- `/move_to_point` (system_interfaces/srv/MoveToPoint)

**로그 메시지:**
- 서비스 준비 완료: `Service server ready: /move_to_point`
- 요청 수신: `Received request: x=X.XX, y=Y.YY, z=Z.ZZ`
- 계획 프레임: `Planning frame: base_link`
- 엔드 이펙터: `End effector link: link_6`
- 계획 성공: `Planning succeeded! Executing trajectory...`
- 계획 실패: `Planning failed!`

## 파일 구조

```
target_move/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── target_move_dsr.launch.py
├── src/
│   └── target_move.cpp
└── include/
    └── target_move/
```

## Launch 파일

### target_move_dsr.launch.py

**Arguments:**
- `model` (default: e0509): 로봇 모델
- `gripper` (default: none): 그리퍼 타입

**기능:**
- MoveIt 설정 로드
- target_move_node 실행

## 예제

### Python 클라이언트 예제

```python
import rclpy
from rclpy.node import Node
from system_interfaces.srv import MoveToPoint
from geometry_msgs.msg import Point

class MoveClient(Node):
    def __init__(self):
        super().__init__('move_client')
        self.client = self.create_client(MoveToPoint, 'move_to_point')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
    
    def send_goal(self, x, y, z):
        request = MoveToPoint.Request()
        request.target_position = Point(x=x, y=y, z=z)
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

def main():
    rclpy.init()
    client = MoveClient()
    
    # 목표 위치로 이동
    result = client.send_goal(0.0, 0.0, 0.8)
    
    if result.success:
        print(f"Success: {result.message}")
    else:
        print(f"Failed: {result.message}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ 클라이언트 예제

```cpp
#include <rclcpp/rclcpp.hpp>
#include "system_interfaces/srv/move_to_point.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_client");
  
  auto client = node->create_client<system_interfaces::srv::MoveToPoint>("move_to_point");
  
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(node->get_logger(), "Waiting for service...");
  }
  
  auto request = std::make_shared<system_interfaces::srv::MoveToPoint::Request>();
  request->target_position.x = 0.0;
  request->target_position.y = 0.0;
  request->target_position.z = 0.8;
  
  auto result = client->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(node, result) == 
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result.get();
    RCLCPP_INFO(node->get_logger(), "Result: %s", response->message.c_str());
  }
  
  rclcpp::shutdown();
  return 0;
}
```

## 문제 해결

### 서비스가 보이지 않음

```bash
# 서비스 확인
ros2 service list | grep move_to_point

# 노드 확인
ros2 node list
```

### MoveIt 초기화 대기

서버 실행 후 다음 메시지를 확인:
```
[INFO] [target_move_server]: Planning frame: base_link
[INFO] [target_move_server]: End effector link: link_6
```

이 메시지가 나타난 후 서비스를 호출하세요.

### Planning 실패

- 목표 위치가 작업 공간 내에 있는지 확인
- 충돌이 발생하지 않는지 확인
- RViz에서 로봇 상태 시각화

## 라이선스

TODO: License declaration

## 작성자

frezest
