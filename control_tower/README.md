# Control Tower - Manipulator Pick&Place GUI (PyQt5 + LLM)

Manipulator의 Pick&Place 시스템을 관리하기 위한 GUI 기반 Control Tower입니다.
LLM 키워드 추출 기능이 통합되어 있습니다.

## 주요 기능

### 1. 상태 모니터 (왼쪽 패널)
- 실시간 시스템 상태 메시지 표시
- 타임스탬프와 함께 모든 이벤트 로깅
- 자동 스크롤 기능
- LLM 키워드 추출 로그 통합

### 2. YOLO 객체 인식 (오른쪽 상단)
- 실시간 YOLO 인식 결과 이미지 표시 (정사각형)
- 자동 크기 조절
- ROS2 Image 토픽 구독 (`/yolo/detection_image`)

### 3. 시스템 제어 (오른쪽 중단)
- **LMM 시작/중지 버튼**: LMM 제어
- **비상 정지 버튼**: 즉시 시스템 정지
- 현재 LMM 상태 및 시스템 상태 표시

### 4. LLM 키워드 추출 (오른쪽 하단) 🆕
- **음성 명령**: 🎤 버튼으로 음성 녹음 → Whisper로 텍스트 변환 → LLM으로 키워드 추출
- **텍스트 명령**: 텍스트 입력창에서 직접 명령 입력
- **지능형 매칭**: Python 내장 SequenceMatcher + LLM (Ollama) 기반 키워드 추출
- **키워드 사전**: tissue, syringe, medicine, bottle, sanitizer
- **현재 키워드 표시**: 추출된 키워드를 실시간으로 표시

## ROS2 토픽

### Publishers
- `/lmm/control` (std_msgs/Bool): LMM 시작/중지 제어
- `/emergency_stop` (std_msgs/Bool): 비상 정지 신호
- `/robot_keyword` (std_msgs/String): 추출된 키워드 발행

### Subscribers
- `/system/status` (std_msgs/String): 시스템 상태 메시지
- `/yolo/detection_image` (sensor_msgs/Image): YOLO 인식 결과 이미지
- `/robot_keyword` (std_msgs/String): 키워드 수신 (자체 발행 및 외부 발행 모두 구독)

## 설치

### 의존성 설치
```bash
# PyQt5 및 기본 패키지
sudo apt-get install python3-pyqt5 python3-opencv python3-pyaudio

# Whisper 설치 (선택 사항 - 음성 인식 기능 사용 시)
# venv 사용 (권장)
python3 -m venv ~/whisper_env
source ~/whisper_env/bin/activate
pip install openai-whisper
deactivate

# 또는 시스템에 직접 설치 (Ubuntu 23.04 이상)
pip install --break-system-packages openai-whisper

# Ollama 설치 (LLM)
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3.2
```

### 패키지 빌드
```bash
cd ~/colcon_ws
colcon build --packages-select control_tower
source install/setup.bash
```

## 실행

```bash
ros2 run control_tower control_tower_gui
```

## 사용 방법

### 1. GUI 실행
위 명령어로 Control Tower GUI를 실행합니다.

### 2. LMM 제어
- "LMM 시작" 버튼을 클릭하여 LMM을 시작/중지합니다.
- 버튼 색상으로 현재 상태를 확인할 수 있습니다.

### 3. 키워드 추출 (음성)
1. 🎤 "음성으로 명령하기" 버튼 클릭
2. 음성으로 명령 (예: "티슈 가져다 줘", "bottle please")
3. ⏹️ "녹음 중지" 버튼 클릭
4. 자동으로 음성 인식 → 키워드 추출 → ROS2 토픽 발행

### 4. 키워드 추출 (텍스트)
1. 텍스트 입력창에 명령 입력 (예: "medicine을 집어줘")
2. Enter 키 또는 📤 버튼 클릭
3. 자동으로 키워드 추출 → ROS2 토픽 발행

### 5. 상태 모니터링
- 왼쪽 패널에서 모든 시스템 활동을 실시간으로 확인합니다.
- 키워드 추출 과정도 상세히 로깅됩니다.

### 6. YOLO 이미지 확인
- 오른쪽 상단 패널에서 YOLO가 인식한 객체 이미지를 확인합니다.

### 7. 비상 정지
- 문제 발생 시 빨간색 "⚠️ 비상 정지 ⚠️" 버튼을 클릭합니다.

## 키워드 추출 프로세스

```
입력 (음성/텍스트)
    ↓
[음성인식] Whisper (음성인 경우)
    ↓
[1단계] 직접 매칭
    ├─ 완전 일치/포함 검사
    └─ SequenceMatcher 유사도 매칭 (80% 이상)
    ↓
[2단계] LLM 추출 (1단계 실패 시)
    ├─ Ollama (llama3.2) 사용
    └─ 프롬프트 기반 키워드 추출
    ↓
[발행] /robot_keyword 토픽으로 발행
    ↓
[표시] GUI에 키워드 표시
```

## 키워드 사전 커스터마이징

`control_tower_gui.py` 파일에서 다음 부분을 수정하세요:

```python
self.dictionary = ["tissue", "syringe", "medicine", "bottle", "sanitizer"]
```

원하는 키워드로 변경 후 다시 빌드하면 됩니다.

## 테스트용 퍼블리셔 예제

```bash
# 상태 메시지 발행
ros2 topic pub /system/status std_msgs/String "data: '로봇 암 초기화 완료'" --once
ros2 topic pub /system/status std_msgs/String "data: '객체 감지됨: bottle'" --once

# 키워드 직접 발행
ros2 topic pub /robot_keyword std_msgs/String "data: 'tissue'" --once
```

## 화면 구성

```
┌────────────────────────────────────────────────────────────────┐
│  Manipulator Pick&Place Control Tower                         │
├─────────────────────────────────┬──────────────────────────────┤
│  상태 모니터 (왼쪽 전체)        │  YOLO 객체 인식             │
│  ┌───────────────────────────┐ │  ┌────────────────────────┐ │
│  │[12:34:56] 시스템 초기화   │ │  │                        │ │
│  │[12:34:57] 🎤 녹음 시작    │ │  │   YOLO Detection       │ │
│  │[12:34:58] [인식] "티슈..."│ │  │      (정사각형)        │ │
│  │[12:35:00] [LLM] 키워드... │ │  │                        │ │
│  │[12:35:01] ✅ 발행: tissue │ │  └────────────────────────┘ │
│  │...                        │ │                              │
│  └───────────────────────────┘ │  시스템 제어                 │
│                                 │  ┌────────────────────────┐ │
│                                 │  │     LMM 시작           │ │
│                                 │  └────────────────────────┘ │
│                                 │  ┌────────────────────────┐ │
│                                 │  │  ⚠️ 비상 정지 ⚠️      │ │
│                                 │  └────────────────────────┘ │
│                                 │  LMM 상태: 대기 중          │
│                                 │  시스템 상태: 정상          │
│                                 │                              │
│                                 │  🤖 LLM 키워드 추출         │
│                                 │  ┌────────────────────────┐ │
│                                 │  │   🎯 tissue           │ │
│                                 │  └────────────────────────┘ │
│                                 │  인식 키워드: tissue, ...   │
│                                 │  ┌────────────────────────┐ │
│                                 │  │ 🎤 음성으로 명령하기   │ │
│                                 │  └────────────────────────┘ │
│                                 │  [텍스트 입력창]  📤        │
└─────────────────────────────────┴──────────────────────────────┘
```

## 문제 해결

### GUI가 실행되지 않는 경우
```bash
python3 -c "import PyQt5"
source ~/colcon_ws/install/setup.bash
```

### 음성 인식이 작동하지 않는 경우
```bash
# venv 사용 (권장)
python3 -m venv ~/whisper_env
source ~/whisper_env/bin/activate
pip install openai-whisper

# 또는
pip install --break-system-packages openai-whisper
```

### LLM 키워드 추출이 작동하지 않는 경우
```bash
# Ollama 설치 확인
ollama --version

# 모델 다운로드
ollama pull llama3.2

# 테스트
ollama run llama3.2 "Hello"
```

### 오디오 녹음 오류
```bash
sudo apt-get install python3-pyaudio portaudio19-dev
pip install pyaudio
```

## 라이선스

TODO: License declaration
