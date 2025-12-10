#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
import numpy as np
import cv2

from std_msgs.msg import Int8, String
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
from system_interfaces.srv import SetTarget, TargetPosition, TargetPose, MoveDone, GripperValue

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTextEdit, QLabel, 
                             QGroupBox, QGridLayout, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPixmap, QImage


class System_Manager(Node):
    def __init__(self, gui_callback=None):
        super().__init__('system_manager')
        
        self.gui_callback = gui_callback  # GUI 로그 콜백
        self.bridge = CvBridge()
        
        self.init_variable()
        self.init_pubsub()
        
        # 타이머 생성 (10Hz = 0.1초마다 main 함수 호출)
        self.timer = self.create_timer(0.1, self.main)

    def init_variable(self):
        self.b_go = True
        self.b_state = False
        self.b_pick = False
        
        self.selected_object = None  # 선택된 객체
        
        # TargetPosition 서비스 응답 데이터
        self.target_x = 0.0
        self.target_y = 0.0
        self.b_target_received = False  # 타겟 위치 수신 여부
        
        self.b_move = False  # 이동 완료 여부
        self.b_gripper = False  # 그리퍼 완료 여부
        
        self.f_filtered_psi = 0
        
        self.joint_state = JointState()
        self.yolo_image = None
        
        self.object_poses = {
                            'bottle': {'z': 0.19, 'qx': 0.0, 'qy': -0.7071, 'qz': 0.0, 'qw': 0.7071},
                            'medicine': {'z': 0.2, 'qx': 1.0, 'qy': 0.0, 'qz': 0.0, 'qw': 0.0},
                            'sanitizer': {'z': 0.16, 'qx': 0.0, 'qy': -0.7071, 'qz': 0.0, 'qw': 0.7071},
                            'tissue': {'z': 0.27, 'qx': 1.0, 'qy': 0.0, 'qz': 0.0, 'qw': 0.0},
                            'syringe': {'z': 0.12, 'qx': 1.0, 'qy': 0.0, 'qz': 0.0, 'qw': 0.0}
                            }
        
        self.object_gripper_values = {
                            'bottle': 370,
                            'medicine': 500,
                            'sanitizer': 350,
                            'tissue': 750,
                            'syringe': 670,
                            }
        
        # Place 위치 (고정 좌표)
        self.place_pose = {'x': 0.161, 'y': 0.503, 'z': 0.391, 'qx': 1.0, 'qy': 0.0, 'qz': 0.0, 'qw': 0.0}
        
        # Home 위치 (초기 위치)
        self.home_pose = {'x': 0.374, 'y': 0.0, 'z': 0.406, 'qx': 1.0, 'qy': 0.0, 'qz': 0.0, 'qw': 0.0}

    def init_pubsub(self):
        # QoS 설정
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.cbfnc_joint_state, qos)
        self.create_subscription(Image, '/yolo/detection_image', self.cbfnc_yolo_image, qos)

        # Publishers
        # self.pub_mini_0_cmd = self.create_publisher(Twist, '/mini_0_cmd', qos)
        
        # Service Clients
        self.cli_set_target = self.create_client(SetTarget, '/e0509/set_target')
        self.cli_target_pose = self.create_client(TargetPose, '/e0509/target_pose')
        self.cli_gripper_value = self.create_client(GripperValue, '/e0509/gripper_value')
        
        # Service Servers
        self.srv_target_position = self.create_service(TargetPosition, '/e0509/target_position', self.srvcb_target_position)
        # self.srv_move_done = self.create_service(MoveDone, '/e0509/move_done', self.srvcb_move_done)
        # self.srv_gripper_done = self.create_service(MoveDone, '/e0509/gripper_done', self.srvcb_gripper_done)
        
        self.get_logger().info('System Manager 초기화 완료')

    # GUI에서 직접 호출할 수 있는 메서드들
    def select_object(self, object_name):
        """GUI에서 직접 호출: 객체 선택"""
        self.selected_object = object_name
        self.get_logger().info(f'객체 선택: {object_name}')
        if self.gui_callback:
            self.gui_callback(f'선택된 객체: {object_name}')
    
    def call_set_target_service(self, target_name):
        """SetTarget 서비스 호출"""
        if not self.cli_set_target.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('SetTarget 서비스를 사용할 수 없습니다.')
            if self.gui_callback:
                self.gui_callback('⚠️ SetTarget 서비스 연결 실패')
            return
        
        request = SetTarget.Request()
        request.target = target_name
        
        future = self.cli_set_target.call_async(request)
        future.add_done_callback(self.set_target_callback)
        
        self.get_logger().info(f'SetTarget 서비스 호출: {target_name}')
        if self.gui_callback:
            self.gui_callback(f'🎯 타겟 설정 요청: {target_name}')
    
    def set_target_callback(self, future):
        """SetTarget 서비스 응답 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'SetTarget 서비스 성공: {response.message}')
                if self.gui_callback:
                    self.gui_callback(f'✅ 타겟 설정 완료: {response.message}')
            else:
                self.get_logger().warn(f'SetTarget 서비스 실패: {response.message}')
                if self.gui_callback:
                    self.gui_callback(f'❌ 타겟 설정 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'SetTarget 서비스 오류: {str(e)}')
            if self.gui_callback:
                self.gui_callback(f'❌ 서비스 오류: {str(e)}')
    
    def call_target_pose_service(self, x, y, z, qx, qy, qz, qw):
        """TargetPose 서비스 호출"""
        if not self.cli_target_pose.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('TargetPose 서비스를 사용할 수 없습니다.')
            if self.gui_callback:
                self.gui_callback('TargetPose 서비스 연결 실패')
            return
        
        request = TargetPose.Request()
        request.target_pose.position.x = x
        request.target_pose.position.y = y
        request.target_pose.position.z = z
        request.target_pose.orientation.x = qx
        request.target_pose.orientation.y = qy
        request.target_pose.orientation.z = qz
        request.target_pose.orientation.w = qw
        
        future = self.cli_target_pose.call_async(request)
        future.add_done_callback(self.target_pose_callback)
        
        self.get_logger().info(f'TargetPose 서비스 호출: pos=({x:.2f}, {y:.2f}, {z:.2f})')
        if self.gui_callback:
            self.gui_callback(f'목표 위치 전송: ({x:.2f}, {y:.2f}, {z:.2f}, {qx:.2f}, {qy:.2f}, {qz:.2f}, {qw:.2f})')
            self.gui_callback('로봇이 이동을 시작합니다. 안전에 주의해주세요.')
    
    def target_pose_callback(self, future):
        """TargetPose 서비스 응답 콜백"""
        try:
            response = future.result()
            if response.success:
                self.b_move = True
                self.get_logger().info(f'Moveit 이동 성공: {response.message}')
                if self.gui_callback:
                    self.gui_callback(f'목표 위치 이동 완료: {response.message}')
            else:
                self.b_move = False
                self.get_logger().warn(f'Moveit 이동 실패: {response.message}')
                if self.gui_callback:
                    self.gui_callback(f'목표 위치 이동 실패: {response.message}')
                    self.gui_callback('로봇 이동에 실패했습니다. 위치를 다시 확인해주세요.')
        except Exception as e:
            self.get_logger().error(f'Moveit 이동 오류: {str(e)}')
            if self.gui_callback:
                self.gui_callback(f'서비스 오류: {str(e)}')
    
    def call_gripper_value_service(self, value):
        """GripperValue 서비스 호출"""
        if not self.cli_gripper_value.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('GripperValue 서비스를 사용할 수 없습니다.')
            if self.gui_callback:
                self.gui_callback('GripperValue 서비스 연결 실패')
            return
        
        request = GripperValue.Request()
        request.value = value
        
        future = self.cli_gripper_value.call_async(request)
        future.add_done_callback(self.gripper_value_callback)
        
        self.get_logger().info(f'GripperValue 서비스 호출: value={value}')
        if self.gui_callback:
            self.gui_callback(f'그리퍼 값 전송: {value}')
    
    def gripper_value_callback(self, future):
        """GripperValue 서비스 응답 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Gripper Control 성공: {response.message}')
                self.b_gripper = True
            else:
                self.get_logger().warn(f'Gripper Control 실패: {response.message}')
                self.b_gripper = False
                if self.gui_callback:
                    self.gui_callback(f'Gripper 값 전송 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'GripperValue 서비스 오류: {str(e)}')
            self.b_gripper = False
            if self.gui_callback:
                self.gui_callback(f'Gripper 서비스 오류: {str(e)}')
    
    def start_voice_recognition(self):
        """GUI에서 직접 호출: 음성인식 시작 (TODO: 기능 구현)"""
        self.get_logger().info('음성인식 시작')
        if self.gui_callback:
            self.gui_callback('음성인식 시작 (구현 예정)')
    
    def emergency_stop(self):
        """GUI에서 직접 호출: 비상 정지"""
        self.b_go = False
        self.get_logger().warn('비상 정지 활성화!')
        if self.gui_callback:
            self.gui_callback('비상 정지 활성화!')

    # Subscribe callback functions
    def cbfnc_joint_state(self, _data:JointState):
        self.joint_state = _data
        self.b_state = True
    
    def cbfnc_yolo_image(self, msg:Image):
        """YOLO 이미지 수신"""
        try:
            self.yolo_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'이미지 변환 오류: {str(e)}')

    # Service callback functions (외부 서비스 호출용)
    def srvcb_target_position(self, request, response):
        """TargetPosition 서비스 콜백 - 외부 노드로부터 타겟 중심점 수신"""
        self.target_x = request.target_position.x
        self.target_y = request.target_position.y
        self.b_target_received = True
        
        self.get_logger().info(f'타겟 중심점 수신: x={self.target_x}, y={self.target_y}')
        if self.gui_callback:
            self.gui_callback(f'📍 타겟 중심점 수신: ({self.target_x:.2f}, {self.target_y:.2f})')
        
        response.success = True
        response.message = "Target center received"
        return response
    
    # def srvcb_move_done(self, request, response):
    #     """MoveDone 서비스 콜백 - 외부 노드로부터 이동 완료 수신"""
    #     self.b_move = request.done
        
    #     if self.b_move:
    #         if self.gui_callback:
    #             self.gui_callback('✅ 로봇이 목표 지점에 도달했습니다.')
        
    #     response.success = True
    #     response.message = "Move done received"
    #     return response
    
    # def srvcb_gripper_done(self, request, response):
    #     """GripperDone 서비스 콜백 - 외부 노드로부터 그리퍼 완료 수신"""
    #     self.b_gripper = request.done
        
    #     if self.b_gripper:
    #         if self.gui_callback:
    #             self.gui_callback('✅ 그리퍼 동작 완료')
        
    #     response.success = True
    #     response.message = "Gripper done received"
    #     return response

    # Main functions
    def main(self):
        """타이머에 의해 10Hz로 자동 호출되는 메인 로직"""
        if not hasattr(self, '_count'):
            self._count = 0
            self._flag_emergen = 0
            self._b_done = False
            self._alpha = 0.7
            self._chosen_index = 5
            self._dist_threshold = 3

        if self.b_state and self.b_go:
            if self.selected_object is not None and self.b_pick == False:
                # Target 선택 후 한 번만 서비스 호출
                if not hasattr(self, '_target_sent') or not self._target_sent:
                    self.call_set_target_service(self.selected_object)
                    self._target_sent = True
                
                # Yolo로 도출된 Target Position정보를 객체별 Pose정보로 변환
                if self.b_target_received:
                    # Target Pose를 한 번만 전송
                    if not hasattr(self, '_pose_sent') or not self._pose_sent:
                        pose = self.object_poses.get(self.selected_object, 
                                            {'z': 0.1, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})
                        
                        # TargetPose 서비스 호출
                        self.call_target_pose_service(
                            self.target_x, self.target_y, pose['z'],
                            pose['qx'], pose['qy'], pose['qz'], pose['qw']
                        )
                        self._pose_sent = True
                    
                    # 로봇이 목표 지점에 도착했으면 그리퍼 제어
                    if self.b_move:
                        # 그리퍼 값을 한 번만 전송
                        if not hasattr(self, '_gripper_sent') or not self._gripper_sent:
                            # gripper_value = self.object_gripper_values.get(self.selected_object, 30)
                            # self.call_gripper_value_service(gripper_value)
                            self.b_gripper = True
                            self._gripper_sent = True
                        
                        if self.b_gripper:
                            self.b_pick = True
                            self.gui_callback('🎉 물체 픽업 완료!')
                            self.b_move = False
                            self.b_gripper = False
            
            # Pick 완료 후 Place 동작
            if self.b_pick:
                # 1단계: 픽업 위치에서 0.1m 위로 이동
                if not hasattr(self, '_lift_sent') or not self._lift_sent:
                    pose = self.object_poses.get(self.selected_object, 
                                        {'z': 0.1, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})
                    
                    # z값 0.1 증가
                    lift_z = pose['z'] + 0.1
                    
                    self.call_target_pose_service(
                        self.target_x, self.target_y, lift_z,
                        pose['qx'], pose['qy'], pose['qz'], pose['qw']
                    )
                    self._lift_sent = True
                    if self.gui_callback:
                        self.gui_callback('⬆️ 물체를 들어올립니다.')
                
                # 2단계: Place 위치로 이동
                if self.b_move:
                    if not hasattr(self, '_place_sent') or not self._place_sent:
                        self.call_target_pose_service(
                            self.place_pose['x'], self.place_pose['y'], self.place_pose['z'],
                            self.place_pose['qx'], self.place_pose['qy'], 
                            self.place_pose['qz'], self.place_pose['qw']
                        )
                        self._place_sent = True
                        self.b_move = False  # 다음 이동 대기
                        if self.gui_callback:
                            self.gui_callback('📍 배치 위치로 이동합니다.')
                    
                    # 3단계: Place 위치 도착 후 그리퍼 열기
                    elif self.b_move:
                        if not hasattr(self, '_release_sent') or not self._release_sent:
                            # self.call_gripper_value_service(200)  # Open
                            self.b_gripper = True
                            self._release_sent = True
                            if self.gui_callback:
                                self.gui_callback('🤲 물체를 내려놓습니다.')
                        
                        # 그리퍼 열림 완료
                        if self.b_gripper:
                            if not hasattr(self, '_home_sent') or not self._home_sent:
                                # 홈 위치로 이동
                                self.call_target_pose_service(
                                    self.home_pose['x'], self.home_pose['y'], self.home_pose['z'],
                                    self.home_pose['qx'], self.home_pose['qy'], 
                                    self.home_pose['qz'], self.home_pose['qw']
                                )
                                self._home_sent = True
                                self.b_gripper = False  # 다음 이동 대기
                                if self.gui_callback:
                                    self.gui_callback('🏠 홈 위치로 복귀합니다.')
                            
                            # 홈 도착 완료
                            elif self.b_move:
                                if self.gui_callback:
                                    self.gui_callback('✅ 모든 작업 완료!')
                                # 모든 플래그 리셋
                                self.b_pick = False
                                self.b_move = False
                                self.b_gripper = False
                                self.b_target_received = False
                                self.selected_object = None               
        else:
            # 정지 상태
            if hasattr(self, '_target_sent'):
                self._target_sent = False
            if hasattr(self, '_pose_sent'):
                self._pose_sent = False
            if hasattr(self, '_gripper_sent'):
                self._gripper_sent = False
            if hasattr(self, '_lift_sent'):
                self._lift_sent = False
            if hasattr(self, '_place_sent'):
                self._place_sent = False
            if hasattr(self, '_release_sent'):
                self._release_sent = False
            if hasattr(self, '_home_sent'):
                self._home_sent = False
            pass


class SystemControlGUI(QMainWindow):
    """시스템 제어 GUI (System Manager와 통합)"""
    
    def __init__(self, system_manager):
        super().__init__()
        
        self.system_manager = system_manager  # System Manager 인스턴스 참조
        
        # YOLO 이미지 업데이트 타이머
        self.image_timer = QTimer()
        self.image_timer.timeout.connect(self.update_yolo_image)
        self.image_timer.start(100)  # 100ms
        
        # GUI 초기화
        self.init_ui()
        
        # 상태 변수
        self.is_running = False
    
    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle('System Manager Control')
        self.setGeometry(100, 100, 1200, 800)
        
        # 메인 위젯
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # 왼쪽 레이아웃 (상태 메시지 창 + 객체 선택)
        left_layout = QVBoxLayout()
        
        # 상태 메시지 창
        status_group = QGroupBox('📊 시스템 상태')
        status_group.setFont(QFont('Arial', 12, QFont.Bold))
        status_layout = QVBoxLayout()
        
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setFont(QFont('Courier', 10))
        status_layout.addWidget(self.status_text)
        
        status_group.setLayout(status_layout)
        left_layout.addWidget(status_group)
        
        # 객체 선택 그룹 (왼쪽 하단)
        object_group = QGroupBox('📦 객체 선택')
        object_group.setFont(QFont('Arial', 11, QFont.Bold))
        object_layout = QGridLayout()
        
        # 객체 버튼들
        objects = [
            ('물병', 'bottle'),
            ('약통', 'medicine'),
            ('손소독제', 'sanitizer'),
            ('휴지', 'tissue'),
            ('주사기', 'syringe')
        ]
        
        for idx, (name, obj_id) in enumerate(objects):
            btn = QPushButton(name)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #3498db;
                    color: white;
                    font-size: 16px;
                    font-weight: bold;
                    padding: 30px;
                    border-radius: 8px;
                }
                QPushButton:hover {
                    background-color: #5dade2;
                }
                QPushButton:pressed {
                    background-color: #2874a6;
                }
                QPushButton:disabled {
                    background-color: #95a5a6;
                    color: #bdc3c7;
                }
            """)
            btn.clicked.connect(lambda checked, o=obj_id, n=name: self.on_object_selected(o, n))
            object_layout.addWidget(btn, idx // 3, idx % 3)
            
            # 버튼을 리스트에 저장 (나중에 활성화/비활성화 제어용)
            if not hasattr(self, 'object_buttons'):
                self.object_buttons = []
            self.object_buttons.append(btn)
        
        object_group.setLayout(object_layout)
        left_layout.addWidget(object_group)
        
        main_layout.addLayout(left_layout, 2)
        
        # 오른쪽 레이아웃 (제어 패널)
        right_layout = QVBoxLayout()
        
        # YOLO 이미지 그룹 (우측 상단)
        yolo_group = QGroupBox('YOLO 객체 인식')
        yolo_group.setFont(QFont('Arial', 11, QFont.Bold))
        yolo_layout = QVBoxLayout()
        
        self.yolo_label = QLabel()
        self.yolo_label.setAlignment(Qt.AlignCenter)
        self.yolo_label.setMinimumHeight(300)
        self.yolo_label.setStyleSheet('background-color: #2c3e50; color: white; border-radius: 8px; font-size: 14px;')
        self.yolo_label.setText('YOLO 이미지 대기 중...')
        yolo_layout.addWidget(self.yolo_label)
        
        yolo_group.setLayout(yolo_layout)
        right_layout.addWidget(yolo_group)
        
        # 로봇 상태 체크 그룹
        robot_status_group = QGroupBox('로봇 연결 상태')
        robot_status_group.setFont(QFont('Arial', 11, QFont.Bold))
        robot_status_group.setMaximumHeight(80)  # 최대 높이 제한
        robot_status_layout = QHBoxLayout()
        
        self.robot_status_label = QLabel('●')
        self.robot_status_label.setFont(QFont('Arial', 20, QFont.Bold))
        self.robot_status_label.setAlignment(Qt.AlignCenter)
        self.robot_status_label.setStyleSheet('color: #e74c3c;')  # 초기: 빨간불
        robot_status_layout.addWidget(self.robot_status_label)
        
        self.robot_status_text = QLabel('연결 대기 중...')
        self.robot_status_text.setFont(QFont('Arial', 15))
        self.robot_status_text.setAlignment(Qt.AlignCenter)
        robot_status_layout.addWidget(self.robot_status_text)
        
        robot_status_group.setLayout(robot_status_layout)
        right_layout.addWidget(robot_status_group)
        
        # 로봇 상태 업데이트 타이머
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_robot_status)
        self.status_timer.start(500)  # 500ms마다 체크
        
        # 음성인식 버튼
        voice_group = QGroupBox('음성 제어')
        voice_group.setFont(QFont('Arial', 11, QFont.Bold))
        voice_layout = QVBoxLayout()
        
        self.voice_btn = QPushButton('음성인식 시작')
        self.voice_btn.setStyleSheet("""
            QPushButton {
                background-color: #9b59b6;
                color: white;
                font-size: 14px;
                font-weight: bold;
                padding: 15px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #bb8fce;
            }
            QPushButton:pressed {
                background-color: #7d3c98;
            }
            QPushButton:disabled {
                background-color: #95a5a6;
                color: #bdc3c7;
            }
        """)
        self.voice_btn.clicked.connect(self.on_voice_clicked)
        voice_layout.addWidget(self.voice_btn)
        
        voice_group.setLayout(voice_layout)
        right_layout.addWidget(voice_group)
        
        # 비상 정지 버튼
        emergency_group = QGroupBox('⚠️ 비상 제어')
        emergency_group.setFont(QFont('Arial', 11, QFont.Bold))
        emergency_layout = QVBoxLayout()
        
        self.emergency_btn = QPushButton('🚨 비상 정지')
        self.emergency_btn.setStyleSheet("""
            QPushButton {
                background-color: #c0392b;
                color: white;
                font-size: 20px;
                font-weight: bold;
                padding: 35px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #e74c3c;
            }
            QPushButton:pressed {
                background-color: #a93226;
            }
        """)
        self.emergency_btn.clicked.connect(self.on_emergency_clicked)
        emergency_layout.addWidget(self.emergency_btn)
        
        emergency_group.setLayout(emergency_layout)
        right_layout.addWidget(emergency_group)
        
        main_layout.addLayout(right_layout, 1)
        
        # 초기 로그
        self.add_status('🚀 System Manager GUI 시작')
        self.add_status('📡 System Manager 연결 완료')
    
    def update_yolo_image(self):
        """YOLO 이미지 업데이트"""
        if self.system_manager.yolo_image is not None:
            try:
                image = self.system_manager.yolo_image
                h, w, ch = image.shape
                bytes_per_line = ch * w
                qt_image = QImage(image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qt_image.rgbSwapped())
                
                # 라벨 크기에 맞게 스케일
                scaled_pixmap = pixmap.scaled(self.yolo_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.yolo_label.setPixmap(scaled_pixmap)
            except Exception as e:
                pass
    
    def update_robot_status(self):
        """로봇 상태 업데이트 (b_state 기반)"""
        if self.system_manager.b_state:
            # 연결됨 - 초록불
            self.robot_status_label.setStyleSheet('color: #27ae60;')
            self.robot_status_text.setText('✅ 로봇 연결됨')
            
            # 객체 선택 버튼 활성화
            if hasattr(self, 'object_buttons'):
                for btn in self.object_buttons:
                    btn.setEnabled(True)
            
            # 음성인식 버튼 활성화
            self.voice_btn.setEnabled(True)
        else:
            # 연결 안됨 - 빨간불
            self.robot_status_label.setStyleSheet('color: #e74c3c;')
            self.robot_status_text.setText('❌ 연결 대기 중...')
            
            # 객체 선택 버튼 비활성화
            if hasattr(self, 'object_buttons'):
                for btn in self.object_buttons:
                    btn.setEnabled(False)
            
            # 음성인식 버튼 비활성화
            self.voice_btn.setEnabled(False)
    
    def on_object_selected(self, object_id, object_name):
        """객체 선택 버튼 클릭"""
        self.system_manager.select_object(object_id)
        self.add_status(f'🎯 객체 선택됨: {object_name}')
        
        # 다른 객체 버튼과 음성인식 버튼 비활성화
        if hasattr(self, 'object_buttons'):
            for btn in self.object_buttons:
                btn.setEnabled(False)
        self.voice_btn.setEnabled(False)
    
    def on_voice_clicked(self):
        """음성인식 버튼 클릭"""
        self.system_manager.start_voice_recognition()
        self.add_status('🎤 음성인식 시작 (기능 구현 예정)')
        
        # 객체 선택 버튼 비활성화
        if hasattr(self, 'object_buttons'):
            for btn in self.object_buttons:
                btn.setEnabled(False)
    
    def on_emergency_clicked(self):
        """비상 정지 버튼 클릭"""
        self.system_manager.emergency_stop()
        self.is_running = False
        self.add_status('🚨 비상 정지 활성화!')
    
    def add_status(self, message):
        """상태 메시지 추가"""
        from datetime import datetime
        timestamp = datetime.now().strftime('%H:%M:%S')
        log_entry = f'[{timestamp}] {message}'
        self.status_text.append(log_entry)
        
        # 자동 스크롤
        scrollbar = self.status_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


def main():
    # ROS2 초기화
    rclpy.init()
    
    # Qt 애플리케이션 생성
    app = QApplication(sys.argv)
    
    # System Manager 생성 (GUI 로그 콜백 연결)
    def gui_log_callback(message):
        if hasattr(main, 'gui') and main.gui:
            main.gui.add_status(message)
    
    system_manager = System_Manager(gui_callback=gui_log_callback)
    
    # GUI 생성 (System Manager 인스턴스 전달)
    gui = SystemControlGUI(system_manager)
    main.gui = gui  # 콜백에서 접근할 수 있도록 저장
    gui.show()
    
    # ROS2 스핀을 위한 타이머
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(system_manager, timeout_sec=0))
    ros_timer.start(10)  # 10ms
    
    # Qt 이벤트 루프 시작
    exit_code = app.exec_()
    
    # 종료 시 정리
    ros_timer.stop()
    system_manager.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
