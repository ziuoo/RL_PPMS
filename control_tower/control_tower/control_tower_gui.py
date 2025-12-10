#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime
import threading
import subprocess
# import pyaudio
import wave
from difflib import SequenceMatcher

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTextEdit, QLabel, 
                             QGroupBox, QScrollArea, QGridLayout, QLineEdit)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QPixmap, QImage, QFont

# SetTarget 서비스 import
try:
    from system_interfaces.srv import SetTarget
except ImportError:
    # 서비스를 찾을 수 없는 경우 대체 처리
    SetTarget = None

# Whisper는 사용 시점에 import (지연 import)


class ROSSignals(QObject):
    """ROS 콜백에서 Qt 시그널을 발생시키기 위한 클래스"""
    status_update = pyqtSignal(str)
    image_update = pyqtSignal(np.ndarray)
    keyword_update = pyqtSignal(str)
    log_message = pyqtSignal(str, str)  # (tag, message)


class ControlTowerNode(Node):
    """ROS2 노드 클래스"""
    
    def __init__(self, signals):
        super().__init__('control_tower_node')
        self.signals = signals
        self.bridge = CvBridge()
        
        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Service Client
        if SetTarget is not None:
            self.keyword_client = self.create_client(SetTarget, '/set_target')
        else:
            self.get_logger().error('SetTarget 서비스를 import할 수 없습니다')
            self.keyword_client = None
        
        # Publishers
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', qos_profile)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            '/system/status',
            self.status_callback,
            qos_profile
        )
        
        self.yolo_image_sub = self.create_subscription(
            Image,
            '/yolo/detection_image',
            self.yolo_image_callback,
            qos_profile
        )
        
        self.keyword_sub = self.create_subscription(
            String,
            '/robot_keyword',
            self.keyword_callback,
            qos_profile
        )
        
        self.get_logger().info('Control Tower Node initialized')
    
    def status_callback(self, msg):
        """시스템 상태 메시지 콜백"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        status_msg = f"[{timestamp}] {msg.data}"
        self.signals.status_update.emit(status_msg)
    
    def yolo_image_callback(self, msg):
        """YOLO 인식 이미지 콜백"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.signals.image_update.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {str(e)}')
    
    def publish_emergency_stop(self):
        """비상 정지 명령 발행"""
        msg = Bool()
        msg.data = True
        self.emergency_stop_pub.publish(msg)
        self.get_logger().warn('Emergency Stop Activated!')
    
    def call_keyword_service(self, keyword):
        """키워드 서비스 호출 (SetTarget 사용)"""
        if self.keyword_client is None:
            self.get_logger().error('키워드 서비스 클라이언트가 초기화되지 않았습니다')
            self.signals.log_message.emit("오류", "서비스 클라이언트 없음")
            return
        
        if not self.keyword_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('키워드 서비스를 사용할 수 없습니다')
            self.signals.log_message.emit("경고", "키워드 서비스 연결 실패")
            return
        
        request = SetTarget.Request()
        request.target = keyword  # 키워드를 target 필드에 설정
        
        # 비동기 호출
        future = self.keyword_client.call_async(request)
        future.add_done_callback(lambda f: self._service_response_callback(f, keyword))
        
        self.get_logger().info(f'키워드 서비스 호출: {keyword}')
    
    def _service_response_callback(self, future, keyword):
        """서비스 응답 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'서비스 응답 성공: {response.message}')
                self.signals.log_message.emit("서비스", f'키워드 "{keyword}" 전송 성공: {response.message}')
            else:
                self.get_logger().warn(f'서비스 응답 실패: {response.message}')
                self.signals.log_message.emit("경고", f'키워드 전송 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'서비스 호출 오류: {e}')
            self.signals.log_message.emit("오류", f'서비스 호출 실패: {str(e)}')
    
    def keyword_callback(self, msg):
        """키워드 수신 콜백"""
        keyword = msg.data
        self.get_logger().info('')
        self.get_logger().info('🎯' * 20)
        self.get_logger().info(f'  ✅ 키워드 수신: "{keyword}"')
        self.get_logger().info(f'  📦 이 키워드로 Pick & Place 수행 예정')
        self.get_logger().info('🎯' * 20)
        self.get_logger().info('')
        self.signals.keyword_update.emit(keyword)


class ControlTowerGUI(QMainWindow):
    """Control Tower GUI 메인 윈도우"""
    
    def __init__(self):
        super().__init__()
        
        # ROS2 초기화
        rclpy.init()
        self.signals = ROSSignals()
        self.ros_node = ControlTowerNode(self.signals)
        
        # 시그널 연결
        self.signals.status_update.connect(self.update_status_monitor)
        self.signals.image_update.connect(self.update_yolo_image)
        self.signals.keyword_update.connect(self.update_keyword_display)
        self.signals.log_message.connect(self.add_log_message)
        
        # ROS2 스핀 타이머
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)  # 10ms
        
        # LLM 설정
        self.model_name = "llama3.2"
        self.dictionary = ["tissue", "syringe", "medicine", "bottle", "sanitizer"]
        self.is_recording = False
        self.recording_time = 0
        
        # Audio 설정
        # self.CHUNK = 1024
        # self.FORMAT = pyaudio.paInt16
        # self.CHANNELS = 1
        # self.RATE = 16000
        # self.audio = pyaudio.PyAudio()
        # self.frames = []
        
        # UI 초기화
        self.init_ui()
        
    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle('Manipulator Pick&Place Control Tower')
        self.setGeometry(100, 100, 1400, 800)
        
        # 중앙 위젯
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 메인 레이아웃
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # 왼쪽 패널 (상태 모니터)
        left_panel = self.create_left_panel()
        main_layout.addWidget(left_panel, 2)
        
        # 오른쪽 패널 (YOLO 이미지 & 컨트롤)
        right_panel = self.create_right_panel()
        main_layout.addWidget(right_panel, 1)
        
        # 초기 메시지
        self.add_status_message("Control Tower 시스템 초기화 완료")
        
    def create_left_panel(self):
        """왼쪽 패널 생성 (상태 모니터)"""
        left_widget = QWidget()
        left_layout = QVBoxLayout()
        left_widget.setLayout(left_layout)
        
        # 상태 모니터 그룹
        status_group = QGroupBox("상태 모니터")
        status_layout = QVBoxLayout()
        status_group.setLayout(status_layout)
        
        # 상태 텍스트 영역
        self.status_monitor = QTextEdit()
        self.status_monitor.setReadOnly(True)
        font = QFont("Monospace", 15)
        self.status_monitor.setFont(font)
        status_layout.addWidget(self.status_monitor)
        
        left_layout.addWidget(status_group)
        
        return left_widget
    
    def create_right_panel(self):
        """오른쪽 패널 생성 (YOLO 이미지 & 컨트롤 버튼)"""
        right_widget = QWidget()
        right_layout = QVBoxLayout()
        right_widget.setLayout(right_layout)
        
        # YOLO 감지 이미지 그룹
        yolo_group = QGroupBox("YOLO 객체 인식")
        yolo_layout = QVBoxLayout()
        yolo_group.setLayout(yolo_layout)
        
        # 이미지 레이블 (정사각형)
        self.yolo_image_label = QLabel()
        self.yolo_image_label.setAlignment(Qt.AlignCenter)
        self.yolo_image_label.setMinimumSize(400, 400)
        self.yolo_image_label.setMaximumSize(600, 600)
        self.yolo_image_label.setText("YOLO 이미지 대기 중...")
        self.yolo_image_label.setStyleSheet("""
            QLabel {
                background-color: #2b2b2b;
                color: #888;
                border: 2px solid #555;
                border-radius: 5px;
                font-size: 16px;
            }
        """)
        yolo_layout.addWidget(self.yolo_image_label, alignment=Qt.AlignCenter)
        
        right_layout.addWidget(yolo_group)
        
        # 컨트롤 패널 그룹
        control_group = QGroupBox("시스템 제어")
        control_layout = QVBoxLayout()
        control_group.setLayout(control_layout)
        
        # 비상 정지 버튼
        emergency_button = QPushButton("⚠️ 비상 정지 ⚠️")
        emergency_button.setMinimumHeight(100)
        emergency_button.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                font-size: 20px;
                font-weight: bold;
                border-radius: 5px;
                border: 3px solid #d32f2f;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
            QPushButton:pressed {
                background-color: #c62828;
            }
        """)
        emergency_button.clicked.connect(self.emergency_stop)
        control_layout.addWidget(emergency_button)
        
        # 상태 표시 레이블
        self.system_status_label = QLabel("시스템 상태: 정상")
        self.system_status_label.setAlignment(Qt.AlignCenter)
        self.system_status_label.setStyleSheet("font-size: 16px; padding: 10px; font-weight: bold;")
        control_layout.addWidget(self.system_status_label)
        
        right_layout.addWidget(control_group)
        
        # LLM 키워드 추출 그룹
        llm_group = QGroupBox("🤖 LLM 키워드 추출")
        llm_layout = QVBoxLayout()
        llm_group.setLayout(llm_layout)
        
        # 현재 키워드 표시
        self.keyword_display = QLabel("대기 중...")
        self.keyword_display.setAlignment(Qt.AlignCenter)
        self.keyword_display.setStyleSheet("""
            QLabel {
                background-color: #0f172a;
                color: #10b981;
                font-size: 18px;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
                border: 2px solid #10b981;
            }
        """)
        llm_layout.addWidget(self.keyword_display)
        
        # 키워드 사전 표시
        dict_text = ", ".join(self.dictionary)
        dict_label = QLabel(f"인식 키워드: {dict_text}")
        dict_label.setWordWrap(True)
        dict_label.setStyleSheet("font-size: 10px; color: #888; padding: 5px;")
        llm_layout.addWidget(dict_label)
        
        # 음성 녹음 버튼
        self.record_button = QPushButton("🎤 음성으로 명령하기")
        self.record_button.setMinimumHeight(45)
        self.record_button.setStyleSheet("""
            QPushButton {
                background-color: #10b981;
                color: white;
                font-size: 14px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #059669;
            }
            QPushButton:pressed {
                background-color: #047857;
            }
        """)
        self.record_button.clicked.connect(self.toggle_recording)
        llm_layout.addWidget(self.record_button)
        
        # 녹음 타이머 레이블
        self.timer_label = QLabel("")
        self.timer_label.setAlignment(Qt.AlignCenter)
        self.timer_label.setStyleSheet("color: #ef4444; font-weight: bold;")
        llm_layout.addWidget(self.timer_label)
        
        # 텍스트 입력 영역
        text_input_layout = QHBoxLayout()
        
        self.text_input = QLineEdit()
        self.text_input.setPlaceholderText("텍스트로 명령 입력...")
        self.text_input.setStyleSheet("""
            QLineEdit {
                background-color: #1e293b;
                color: #e2e8f0;
                border: 1px solid #555;
                border-radius: 5px;
                padding: 8px;
                font-size: 11px;
            }
        """)
        self.text_input.returnPressed.connect(self.send_text_command)
        text_input_layout.addWidget(self.text_input)
        
        send_button = QPushButton("📤")
        send_button.setFixedWidth(40)
        send_button.setStyleSheet("""
            QPushButton {
                background-color: #3b82f6;
                color: white;
                font-size: 14px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #2563eb;
            }
        """)
        send_button.clicked.connect(self.send_text_command)
        text_input_layout.addWidget(send_button)
        
        llm_layout.addLayout(text_input_layout)
        
        right_layout.addWidget(llm_group)
        
        return right_widget
    
    def emergency_stop(self):
        """비상 정지"""
        self.add_status_message("⚠️⚠️⚠️ 비상 정지 활성화! ⚠️⚠️⚠️")
        self.system_status_label.setText("시스템 상태: 비상 정지")
        self.system_status_label.setStyleSheet("font-size: 16px; padding: 10px; color: #f44336; font-weight: bold;")
        
        self.ros_node.publish_emergency_stop()
    
    def add_status_message(self, message):
        """상태 모니터에 메시지 추가"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        formatted_msg = f"[{timestamp}] {message}"
        self.status_monitor.append(formatted_msg)
        
        # 자동 스크롤
        scrollbar = self.status_monitor.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def update_status_monitor(self, message):
        """ROS 메시지로부터 상태 모니터 업데이트"""
        self.status_monitor.append(message)
        
        # 자동 스크롤
        scrollbar = self.status_monitor.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def update_yolo_image(self, cv_image):
        """YOLO 이미지 업데이트"""
        # OpenCV BGR을 RGB로 변환
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        
        # QImage로 변환
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        
        # 레이블 크기에 맞게 스케일
        pixmap = QPixmap.fromImage(qt_image)
        scaled_pixmap = pixmap.scaled(self.yolo_image_label.size(), 
                                      Qt.KeepAspectRatio, 
                                      Qt.SmoothTransformation)
        
        self.yolo_image_label.setPixmap(scaled_pixmap)
        self.add_status_message("📷 YOLO 이미지 업데이트됨")
    
    def update_keyword_display(self, keyword):
        """키워드 디스플레이 업데이트"""
        self.keyword_display.setText(f"🎯 {keyword}")
        self.add_status_message(f"📦 수신 키워드: {keyword} → Pick & Place 수행 예정")
    
    def add_log_message(self, tag, message):
        """로그 메시지 추가"""
        self.add_status_message(f"[{tag}] {message}")
    
    # ========== LLM 키워드 추출 기능 ==========
    
    def toggle_recording(self):
        """녹음 토글"""
        if self.is_recording:
            self.stop_recording()
        else:
            self.start_recording()
    
    def start_recording(self):
        """녹음 시작"""
        self.is_recording = True
        self.recording_time = 0
        self.frames = []
        
        self.record_button.setStyleSheet("""
            QPushButton {
                background-color: #ef4444;
                color: white;
                font-size: 14px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #dc2626;
            }
        """)
        self.record_button.setText("⏹️ 녹음 중지")
        self.text_input.setEnabled(False)
        
        self.add_status_message("🎤 녹음 시작")
        
        # 별도 스레드에서 녹음
        threading.Thread(target=self._record_audio, daemon=True).start()
        self.update_recording_timer()
    
    def _record_audio(self):
        """오디오 녹음"""
        try:
            stream = self.audio.open(
                format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.CHUNK
            )
            
            while self.is_recording:
                try:
                    data = stream.read(self.CHUNK, exception_on_overflow=False)
                    self.frames.append(data)
                except Exception as e:
                    self.ros_node.get_logger().error(f"녹음 오류: {e}")
                    break
            
            stream.stop_stream()
            stream.close()
        except Exception as e:
            self.signals.log_message.emit("오류", f"오디오 스트림 오류: {e}")
    
    def update_recording_timer(self):
        """녹음 타이머 업데이트"""
        if self.is_recording:
            self.recording_time += 1
            self.timer_label.setText(f"⏺️ 녹음 중: {self.recording_time}초")
            QTimer.singleShot(1000, self.update_recording_timer)
        else:
            self.timer_label.setText("")
    
    def stop_recording(self):
        """녹음 중지 및 처리"""
        self.is_recording = False
        self.record_button.setStyleSheet("""
            QPushButton {
                background-color: #10b981;
                color: white;
                font-size: 14px;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #059669;
            }
        """)
        self.record_button.setText("🎤 음성으로 명령하기")
        
        self.add_status_message(f"🎤 녹음 완료 ({self.recording_time}초)")
        
        # 오디오 파일 저장
        output_file = "/tmp/control_tower_recording.wav"
        try:
            wf = wave.open(output_file, 'wb')
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(self.audio.get_sample_size(self.FORMAT))
            wf.setframerate(self.RATE)
            wf.writeframes(b''.join(self.frames))
            wf.close()
            
            # 별도 스레드에서 음성 인식 및 키워드 추출
            threading.Thread(target=self._process_voice, args=(output_file,), daemon=True).start()
        except Exception as e:
            self.signals.log_message.emit("오류", f"파일 저장 오류: {e}")
            self.text_input.setEnabled(True)
    
    def _process_voice(self, audio_file):
        """음성 파일 처리 - keyword_gui.py와 동일한 방식으로 직접 import"""
        try:
            self.signals.log_message.emit("Whisper", "음성을 텍스트로 변환 중...")
            
            # Whisper로 텍스트 변환 (직접 import)
            import whisper
            model = whisper.load_model("base")
            result = model.transcribe(audio_file, language="ko")
            text = result["text"].strip()
            
            if text:
                self.signals.log_message.emit("인식", f'"{text}"')
                self._extract_and_publish_keyword(text)
            else:
                self.signals.log_message.emit("오류", "음성 인식 실패")
                self.text_input.setEnabled(True)
                
        except Exception as e:
            self.signals.log_message.emit("오류", str(e))
            self.text_input.setEnabled(True)
    
    def send_text_command(self):
        """텍스트 명령 전송"""
        text = self.text_input.text().strip()
        if not text:
            return
        
        self.text_input.clear()
        self.text_input.setEnabled(False)
        
        self.add_status_message(f"📝 입력: \"{text}\"")
        
        threading.Thread(target=self._extract_and_publish_keyword, args=(text,), daemon=True).start()
    
    def _extract_and_publish_keyword(self, text):
        """LLM으로 키워드 추출 및 서비스 호출 - 첫 번째 키워드만"""
        self.signals.log_message.emit("LLM", "키워드 분석 시작")
        
        try:
            # 1. 직접 매칭 시도
            keywords = self._direct_match(text)
            
            # 2. 직접 매칭 실패 시 LLM 사용
            if not keywords:
                keywords = self._llm_extract(text)
            
            # 3. 키워드 서비스 호출 (첫 번째만)
            if keywords:
                keyword = keywords[0]  # 첫 번째 키워드
                
                # ROS2 서비스로 호출
                self.ros_node.call_keyword_service(keyword)
                
                self.signals.log_message.emit("✅ 전송", f'키워드: "{keyword}"')
                self.keyword_display.setText(f"🎯 {keyword}")
            else:
                self.signals.log_message.emit("경고", "키워드를 찾을 수 없습니다")
                
        except Exception as e:
            self.signals.log_message.emit("오류", str(e))
        finally:
            self.text_input.setEnabled(True)
    
    def _direct_match(self, text):
        """직접 키워드 매칭 (내장 라이브러리 사용) - 첫 번째 키워드만 반환"""
        text_lower = text.lower()
        
        # 1. 완전 일치/포함 - 첫 번째 발견 시 즉시 반환
        for keyword in self.dictionary:
            if keyword.lower() in text_lower:
                self.signals.log_message.emit("매칭", f"매칭 성공: {keyword}")
                return [keyword]  # 첫 번째만 반환
        
        # 2. 유사도 매칭 (SequenceMatcher 사용, 0.6 이상)
        for keyword in self.dictionary:
            # 부분 문자열로 매칭
            ratio = SequenceMatcher(None, keyword.lower(), text_lower).ratio()
            if ratio >= 0.6:
                self.signals.log_message.emit("매칭", f"유사도 매칭 성공: {keyword}")
                return [keyword]  # 첫 번째만 반환
            else:
                # 단어 단위로 분할하여 매칭
                words = text_lower.split()
                for word in words:
                    word_ratio = SequenceMatcher(None, keyword.lower(), word).ratio()
                    if word_ratio >= 0.8:
                        self.signals.log_message.emit("매칭", f"단어 매칭 성공: {keyword}")
                        return [keyword]  # 첫 번째만 반환
        
        return []
    
    def _llm_extract(self, text):
        """LLM으로 키워드 추출 - 첫 번째 키워드만 반환"""
        prompt = f"""
다음 텍스트에서 아래 목록에 있는 물체 이름(영어) 중 가장 먼저 언급된 것 하나만 답하세요.
목록: {', '.join(self.dictionary)}

텍스트: "{text}"

목록에 없는 물체는 무시하고, 반드시 영어 단어 하나만 답하세요. 아무것도 없으면 'none'이라고 답하세요.
예시 답변: tissue
"""
        try:
            result = subprocess.run(
                ['ollama', 'run', self.model_name, prompt],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                response = result.stdout.strip().lower()
                # 첫 단어만 추출
                first_word = response.split()[0] if response.split() else ""
                
                # dictionary에 있는지 확인
                for keyword in self.dictionary:
                    ratio = SequenceMatcher(None, keyword.lower(), first_word).ratio()
                    if ratio >= 0.8:
                        self.signals.log_message.emit("LLM", f"추출 성공: {keyword}")
                        return [keyword]  # 첫 번째만 반환
            
            return []
        except Exception as e:
            self.ros_node.get_logger().error(f"LLM 오류: {e}")
            return []
    
    def spin_ros(self):
        """ROS2 노드 스핀"""
        rclpy.spin_once(self.ros_node, timeout_sec=0)
    
    def closeEvent(self, event):
        """윈도우 종료 시 처리"""
        self.add_status_message("시스템 종료 중...")
        self.ros_timer.stop()
        self.audio.terminate()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    gui = ControlTowerGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
