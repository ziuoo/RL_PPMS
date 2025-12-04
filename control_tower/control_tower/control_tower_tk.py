#!/usr/bin/env python3
"""
컨트롤 타워 - 통합 관제 시스템
모든 로봇 시스템 관제 + LLM 키워드 추출 통합
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import time
import subprocess
import pyaudio
import wave

from rapidfuzz import fuzz, process
import json


class ControlTowerGUI(Node):
    def __init__(self, root):
        super().__init__('control_tower')
        
        self.root = root
        self.root.title("🏢 Control Tower - 로봇 시스템 통합 관제")
        self.root.geometry("1200x800")
        self.root.configure(bg='#1e293b')
        
        # 모니터링 상태
        self.current_keyword = "대기 중..."
        self.object_position = "위치 정보 없음"
        self.robot_status = "IDLE"
        
        # ROS2 구독 - 키워드 모니터링
        self.keyword_sub = self.create_subscription(
            String,
            'robot_keyword',
            self.keyword_callback,
            10
        )
        
        # ROS2 발행 - 긴급 정지 등
        self.emergency_pub = self.create_publisher(
            String,
            'emergency_stop',
            10
        )
        
        # ROS2 발행 - 키워드 발행
        self.keyword_pub = self.create_publisher(
            String,
            'robot_keyword',
            10
        )
        
        # LLM 키워드 추출 상태
        self.model_name = "llama3.2"
        self.dictionary = ["tissue", "syringe", "medicine", "bottle", "sanitizer"]
        self.is_recording = False
        self.recording_time = 0
        
        # Audio settings
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.audio = pyaudio.PyAudio()
        self.frames = []
        
        self.setup_ui()
        self.get_logger().info('🏢 Control Tower 시작 (LLM 통합)')
        self.get_logger().info(f'키워드 사전: {self.dictionary}')
        
    def setup_ui(self):
        """UI 구성"""
        # 헤더
        header = tk.Frame(self.root, bg='#0f172a', height=80)
        header.pack(fill=tk.X)
        header.pack_propagate(False)
        
        title = tk.Label(
            header,
            text="🏢 CONTROL TOWER",
            font=('Arial', 24, 'bold'),
            bg='#0f172a',
            fg='#3b82f6'
        )
        title.pack(side=tk.LEFT, padx=30, pady=20)
        
        # 시간 표시
        self.time_var = tk.StringVar()
        time_label = tk.Label(
            header,
            textvariable=self.time_var,
            font=('Arial', 14),
            bg='#0f172a',
            fg='#94a3b8'
        )
        time_label.pack(side=tk.RIGHT, padx=30)
        self.update_time()
        
        # 메인 컨테이너
        main_container = tk.Frame(self.root, bg='#1e293b')
        main_container.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        # 왼쪽: 상태 모니터링
        left_panel = tk.Frame(main_container, bg='#1e293b')
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # 1. 키워드 모니터
        keyword_frame = tk.LabelFrame(
            left_panel,
            text="🎯 현재 키워드",
            font=('Arial', 14, 'bold'),
            bg='#0f172a',
            fg='#e2e8f0',
            relief=tk.RIDGE,
            borderwidth=2
        )
        keyword_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.keyword_display = tk.Label(
            keyword_frame,
            text="대기 중...",
            font=('Arial', 24, 'bold'),
            bg='#0f172a',
            fg='#10b981',
            height=2
        )
        self.keyword_display.pack(padx=15, pady=15, fill=tk.BOTH)
        
        # 2. 객체 위치 모니터
        position_frame = tk.LabelFrame(
            left_panel,
            text="📍 객체 위치 (YOLO)",
            font=('Arial', 14, 'bold'),
            bg='#0f172a',
            fg='#e2e8f0',
            relief=tk.RIDGE,
            borderwidth=2
        )
        position_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.position_display = tk.Label(
            position_frame,
            text="위치 정보 없음",
            font=('Arial', 16),
            bg='#0f172a',
            fg='#3b82f6',
            height=3,
            justify=tk.LEFT,
            anchor='w'
        )
        self.position_display.pack(padx=15, pady=15, fill=tk.BOTH)
        
        # 3. 로봇 상태 모니터
        robot_frame = tk.LabelFrame(
            left_panel,
            text="🤖 로봇 상태",
            font=('Arial', 14, 'bold'),
            bg='#0f172a',
            fg='#e2e8f0',
            relief=tk.RIDGE,
            borderwidth=2
        )
        robot_frame.pack(fill=tk.BOTH, expand=True)
        
        self.robot_display = tk.Label(
            robot_frame,
            text="IDLE",
            font=('Arial', 20, 'bold'),
            bg='#0f172a',
            fg='#f59e0b',
            height=2
        )
        self.robot_display.pack(padx=15, pady=15, fill=tk.BOTH, expand=True)
        
        # 오른쪽: LLM 키워드 추출 + 활동 로그
        right_panel = tk.Frame(main_container, bg='#1e293b')
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))
        
        # LLM 키워드 추출 패널
        llm_frame = tk.LabelFrame(
            right_panel,
            text="🤖 LLM 키워드 추출",
            font=('Arial', 14, 'bold'),
            bg='#0f172a',
            fg='#e2e8f0',
            relief=tk.RIDGE,
            borderwidth=2
        )
        llm_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 키워드 사전 표시
        dict_text = ", ".join(self.dictionary)
        tk.Label(
            llm_frame,
            text=f"인식 키워드: {dict_text}",
            bg='#0f172a',
            fg='#10b981',
            font=('Arial', 10),
            wraplength=400
        ).pack(padx=10, pady=(10, 5))
        
        # 음성 녹음 버튼
        self.record_btn = tk.Button(
            llm_frame,
            text="🎤 음성으로 명령하기",
            command=self.toggle_recording,
            bg='#10b981',
            fg='white',
            font=('Arial', 12, 'bold'),
            relief=tk.FLAT,
            cursor='hand2',
            height=2
        )
        self.record_btn.pack(fill=tk.X, padx=10, pady=5)
        
        # 텍스트 입력
        text_frame = tk.Frame(llm_frame, bg='#0f172a')
        text_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.text_input = tk.Entry(
            text_frame,
            font=('Arial', 11),
            bg='#1e293b',
            fg='#e2e8f0',
            insertbackground='#e2e8f0',
            relief=tk.FLAT
        )
        self.text_input.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.text_input.bind('<Return>', lambda e: self.send_text_command())
        
        send_btn = tk.Button(
            text_frame,
            text="📤",
            command=self.send_text_command,
            bg='#3b82f6',
            fg='white',
            font=('Arial', 11, 'bold'),
            relief=tk.FLAT,
            cursor='hand2',
            width=3
        )
        send_btn.pack(side=tk.LEFT)
        
        # 녹음 타이머
        self.timer_var = tk.StringVar(value="")
        timer_label = tk.Label(
            llm_frame,
            textvariable=self.timer_var,
            bg='#0f172a',
            fg='#ef4444',
            font=('Arial', 10, 'bold')
        )
        timer_label.pack(pady=(0, 10))
        
        # 활동 로그
        log_frame = tk.LabelFrame(
            right_panel,
            text="📋 활동 로그",
            font=('Arial', 14, 'bold'),
            bg='#0f172a',
            fg='#e2e8f0',
            relief=tk.RIDGE,
            borderwidth=2
        )
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_display = scrolledtext.ScrolledText(
            log_frame,
            wrap=tk.WORD,
            font=('Consolas', 10),
            bg='#0f172a',
            fg='#e2e8f0',
            relief=tk.FLAT
        )
        self.log_display.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.log_display.config(state=tk.DISABLED)
        
        # 태그 설정
        self.log_display.tag_config('info', foreground='#3b82f6')
        self.log_display.tag_config('success', foreground='#10b981')
        self.log_display.tag_config('warning', foreground='#f59e0b')
        self.log_display.tag_config('error', foreground='#ef4444')
        
        self.add_log("시스템", "Control Tower 초기화 완료", "success")
    
    def update_time(self):
        """시간 업데이트"""
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        self.time_var.set(current_time)
        self.root.after(1000, self.update_time)
    
    def add_log(self, tag, message, color='info'):
        """로그 추가"""
        self.log_display.config(state=tk.NORMAL)
        timestamp = time.strftime("%H:%M:%S")
        self.log_display.insert(tk.END, f"[{timestamp}] ", color)
        self.log_display.insert(tk.END, f"[{tag}] ", 'warning')
        self.log_display.insert(tk.END, f"{message}\n", color)
        self.log_display.see(tk.END)
        self.log_display.config(state=tk.DISABLED)
    
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
        
        self.record_btn.config(bg='#ef4444', text='⏹️ 녹음 중지')
        self.text_input.config(state=tk.DISABLED)
        
        self.add_log("음성", "녹음 시작", "info")
        
        # 별도 스레드에서 녹음
        threading.Thread(target=self._record_audio, daemon=True).start()
        self.update_timer()
    
    def _record_audio(self):
        """오디오 녹음"""
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
                self.get_logger().error(f"녹음 오류: {e}")
                break
        
        stream.stop_stream()
        stream.close()
    
    def update_timer(self):
        """녹음 타이머 업데이트"""
        if self.is_recording:
            self.recording_time += 1
            self.timer_var.set(f"⏺️ 녹음 중: {self.recording_time}초")
            self.root.after(1000, self.update_timer)
        else:
            self.timer_var.set("")
    
    def stop_recording(self):
        """녹음 중지 및 처리"""
        self.is_recording = False
        self.record_btn.config(bg='#10b981', text='🎤 음성으로 명령하기')
        
        self.add_log("음성", f"녹음 완료 ({self.recording_time}초)", "success")
        
        # 오디오 파일 저장
        output_file = "/tmp/control_tower_recording.wav"
        wf = wave.open(output_file, 'wb')
        wf.setnchannels(self.CHANNELS)
        wf.setsampwidth(self.audio.get_sample_size(self.FORMAT))
        wf.setframerate(self.RATE)
        wf.writeframes(b''.join(self.frames))
        wf.close()
        
        # 별도 스레드에서 음성 인식 및 키워드 추출
        threading.Thread(target=self._process_voice, args=(output_file,), daemon=True).start()
    
    def _process_voice(self, audio_file):
        """음성 파일 처리"""
        try:
            self.add_log("Whisper", "음성을 텍스트로 변환 중...", "info")
            
            import whisper
            model = whisper.load_model("base")
            result = model.transcribe(audio_file, language="ko")
            text = result["text"].strip()
            
            if text:
                self.root.after(0, lambda t=text: self.add_log("인식", f'"{t}"', "success"))
                self.root.after(0, lambda t=text: self._extract_and_publish_keyword(t))
            else:
                self.root.after(0, lambda: self.add_log("오류", "음성 인식 실패", "error"))
                
        except Exception as e:
            error_msg = str(e)
            self.root.after(0, lambda msg=error_msg: self.add_log("오류", msg, "error"))
        finally:
            self.root.after(0, lambda: self.text_input.config(state=tk.NORMAL))
    
    def send_text_command(self):
        """텍스트 명령 전송"""
        text = self.text_input.get().strip()
        if not text:
            return
        
        self.text_input.delete(0, tk.END)
        self.text_input.config(state=tk.DISABLED)
        
        self.add_log("입력", f'"{text}"', "info")
        
        threading.Thread(target=self._extract_and_publish_keyword, args=(text,), daemon=True).start()
    
    def _extract_and_publish_keyword(self, text):
        """LLM으로 키워드 추출 및 발행"""
        self.add_log("LLM", "키워드 분석 시작", "info")
        
        try:
            # 1. 직접 매칭 시도
            keywords = self._direct_match(text)
            
            # 2. 직접 매칭 실패 시 LLM 사용
            if not keywords:
                keywords = self._llm_extract(text)
            
            # 3. 키워드 발행
            if keywords:
                keyword = keywords[0]  # 첫 번째 키워드만 사용
                
                # ROS2 토픽으로 발행
                msg = String()
                msg.data = keyword
                self.keyword_pub.publish(msg)
                
                self.get_logger().info(f'키워드 발행: {keyword}')
                
                self.root.after(0, lambda k=keyword: self.add_log(
                    "✅ 발행", 
                    f'키워드: "{k}"', 
                    "success"
                ))
                self.root.after(0, lambda k=keyword: self.keyword_display.config(
                    text=f"🎯 {k}",
                    fg='#10b981'
                ))
            else:
                self.root.after(0, lambda: self.add_log(
                    "경고", 
                    "키워드를 찾을 수 없습니다", 
                    "warning"
                ))
                
        except Exception as e:
            error_msg = str(e)
            self.root.after(0, lambda msg=error_msg: self.add_log("오류", msg, "error"))
        finally:
            self.root.after(0, lambda: self.text_input.config(state=tk.NORMAL))
    
    def _direct_match(self, text):
        """직접 키워드 매칭 (fuzzy + 한글 변환)"""
        text_lower = text.lower()
        found = []
        # 1. 완전 일치/포함
        for keyword in self.dictionary:
            if keyword.lower() in text_lower:
                found.append(keyword)
        # 2. fuzzy matching (80점 이상)
        for keyword in self.dictionary:
            score = fuzz.partial_ratio(keyword.lower(), text_lower)
            if score >= 80 and keyword not in found:
                found.append(keyword)
        if found:
            self.add_log("매칭", f"fuzzy 매칭 성공: {found}", "success")
        return found
    
    def _llm_extract(self, text):
        """LLM으로 키워드 추출 (프롬프트 개선, 다중 키워드, 후처리)"""
        prompt = f"""
다음 텍스트에서 아래 목록에 있는 물체 이름(영어)만 모두 추출해서 콤마(,)로 구분해 답하세요.
예시 답변: tissue, bottle
목록: {', '.join(self.dictionary)}

텍스트: "{text}"

목록에 없는 물체는 무시하고, 반드시 영어로만 답하세요. 아무것도 없으면 'none'이라고 답하세요.
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
                # 후처리: 콤마/줄바꿈/공백 기준 분리, 사전 키워드만 추출
                candidates = [w.strip() for w in response.replace('\n', ',').split(',') if w.strip()]
                found = []
                for keyword in self.dictionary:
                    for cand in candidates:
                        if fuzz.ratio(keyword.lower(), cand) >= 80 and keyword not in found:
                            found.append(keyword)
                if found:
                    self.add_log("LLM", f"추출 성공: {found}", "success")
                    return found
            return []
        except Exception as e:
            self.get_logger().error(f"LLM 오류: {e}")
            return []
    
    def keyword_callback(self, msg):
        """키워드 수신 콜백 - 키워드 리시버 기능 통합"""
        keyword = msg.data
        self.get_logger().info('')
        self.get_logger().info('🎯' * 20)
        self.get_logger().info(f'  ✅ 키워드 수신: "{keyword}"')
        self.get_logger().info(f'  📦 이 키워드로 Pick & Place 수행 예정')
        self.get_logger().info('🎯' * 20)
        self.get_logger().info('')
        
        # GUI 업데이트
        self.root.after(0, lambda k=keyword: self.keyword_display.config(
            text=f"🎯 {k}",
            fg='#10b981'
        ))
        self.root.after(0, lambda k=keyword: self.add_log(
            "📦 수신", 
            f'키워드: "{k}" → Pick & Place 수행 예정', 
            "success"
        ))
    
    def cleanup(self):
        """종료 시 정리"""
        self.add_log("시스템", "Control Tower 종료 중...", "warning")
        self.audio.terminate()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    root = tk.Tk()
    app = ControlTowerGUI(root)
    
    # ROS2 spin을 별도 스레드에서 실행
    def spin_ros():
        rclpy.spin(app)
    
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    
    # 종료 처리
    def on_closing():
        app.cleanup()
        rclpy.shutdown()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
