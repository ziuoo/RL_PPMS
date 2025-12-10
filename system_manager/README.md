# System Manager

시스템 매니저와 GUI 컨트롤러 패키지입니다.

## 개요

이 패키지는 두 개의 노드로 구성됩니다:

1. **system_manager**: 실제 시스템 제어 로직을 담당하는 백엔드 노드
2. **gui_controller**: PyQt5 기반 GUI로 system_manager를 제어하는 프론트엔드 노드

## 구조

```
GUI (gui_controller)  →  Services  →  System Manager
     ↓                                      ↓
  버튼 클릭                            main() 함수에서 
  (GO/STOP/                           b_go 변수 확인하여
   Scenario)                           동작 수행
```

### 통신 구조

#### Services (GUI → System Manager)
- `/srv_go_stop` (SetBool): GO/STOP 명령
- `/scenario_select_1` (Trigger): 시나리오 1 선택
- `/scenario_select_2` (Trigger): 시나리오 2 선택

#### Topics (System Manager → GUI)
- `/system/status` (String): 시스템 상태 메시지

## 사용 방법

### 1. 빌드

```bash
cd ~/colcon_ws
colcon build --packages-select system_manager
source install/setup.bash
```

### 2. 실행

#### 터미널 1: System Manager 실행
```bash
ros2 run system_manager system_manager
```

#### 터미널 2: GUI Controller 실행
```bash
ros2 run system_manager gui_controller
```

### 3. GUI 사용

GUI에서 다음 버튼들을 사용할 수 있습니다:

- **▶️ GO**: 시스템 동작 시작 (`b_go = True`)
- **⏸️ STOP**: 시스템 일시정지 (`b_go = False`)
- **🚨 EMERGENCY STOP**: 비상 정지 (토픽 발행)
- **📋 Scenario 1**: 시나리오 1 선택 (`i_trigger_index = 7`)
- **📋 Scenario 2**: 시나리오 2 선택

## 동작 원리

### System Manager의 main() 함수

```python
def main(self):
    # 타이머(10Hz)에 의해 자동 호출됨
    
    if self.b_go == True:  # GUI에서 GO 버튼을 누르면
        # 실제 제어 로직 수행
        _des_vx, _des_psi = self.fnc_leader(...)
        self.pub_mini_0_cmd.publish(_cmd)
    
    elif self.b_go == False:  # GUI에서 STOP 버튼을 누르면
        # 정지 명령
        _cmd.linear.x = 0.0
        _cmd.angular.z = 0.0
        self.pub_mini_0_cmd.publish(_cmd)
```

### GUI 버튼 동작

1. **GO 버튼 클릭**
   - GUI가 `/srv_go_stop` 서비스 호출 (data=True)
   - System Manager가 `self.b_go = True`로 설정
   - `main()` 함수에서 `if self.b_go == True:` 조건 만족
   - 실제 제어 로직 수행

2. **STOP 버튼 클릭**
   - GUI가 `/srv_go_stop` 서비스 호출 (data=False)
   - System Manager가 `self.b_go = False`로 설정
   - `main()` 함수에서 `elif self.b_go == False:` 조건 만족
   - 정지 명령 전송

3. **Scenario 버튼 클릭**
   - GUI가 `/scenario_select_1` 또는 `/scenario_select_2` 서비스 호출
   - System Manager가 `self.i_trigger_index` 값 변경
   - `main()` 함수에서 시나리오에 따라 다른 동작 수행

## 코드 구조

### system_manager.py

```python
class System_Manager(Node):
    def __init__(self):
        # 타이머 생성 - main() 함수를 10Hz로 자동 호출
        self.timer = self.create_timer(0.1, self.main)
    
    def init_variable(self):
        self.b_go = False  # GO/STOP 상태
        self.i_trigger_index = np.inf  # 시나리오 인덱스
    
    def srvcb_go_stop(self, request, response):
        # GUI로부터 GO/STOP 명령 수신
        self.b_go = request.data
        return response
    
    def main(self):
        # 타이머에 의해 자동 호출되는 메인 로직
        if self.b_go:
            # 제어 수행
        else:
            # 정지
```

### gui_controller.py

```python
class GUINode(Node):
    def call_go_service(self, go_command):
        # GO/STOP 서비스 호출
        request = SetBool.Request()
        request.data = go_command
        self.go_stop_client.call_async(request)

class SystemControlGUI(QMainWindow):
    def on_go_clicked(self):
        # GO 버튼 클릭 시
        self.ros_node.call_go_service(True)
    
    def on_stop_clicked(self):
        # STOP 버튼 클릭 시
        self.ros_node.call_go_service(False)
```

## 의존성

- ROS2 (Jazzy)
- Python 3
- PyQt5
- rclpy
- std_msgs
- std_srvs
- geometry_msgs
- nav_msgs

## 참고사항

- `main()` 함수는 타이머에 의해 자동으로 10Hz(0.1초마다) 호출됩니다
- GUI 버튼의 신호는 서비스를 통해 `b_go` 변수에 전달됩니다
- `main()` 함수는 `b_go` 값을 확인하여 동작 여부를 결정합니다
- 시스템 상태는 `/system/status` 토픽을 통해 GUI로 전송됩니다
