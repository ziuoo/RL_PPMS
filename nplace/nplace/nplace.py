#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from system_interfaces.srv import TargetPose, GripperValue, MoveDone
from dsr_msgs2.srv import MoveLine, MoveJoint, DrlStart
import math
import textwrap

# Doosan Robotics 상수 정의
DR_MV_MOD_ABS = 0  # 절대 이동
DR_MV_MOD_REL = 1  # 상대 이동
DR_BASE = 0  # Base coordinate
DR_WORLD = 1  # World coordinate
DR_TOOL = 2  # Tool coordinate
DR_MV_RA_DUPLICATE = 0
DR_MV_RA_OVERRIDE = 1
DR_MV_APP_NONE = 0

class NPlaceController(Node):
    def __init__(self):
        super().__init__('nplace_controller')
        
        self.init_variable()
        self.init_services()
        self.init_doosan_robot()
        
        self.get_logger().info('NPlace Controller 초기화 완료')
    
    def init_variable(self):
        """변수 초기화"""
        self.current_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [x, y, z, rx, ry, rz]
        self.velocity = [150.0, 150.0]  # 속도 [mm/s, deg/s]
        self.acceleration = [300.0, 300.0]  # 가속도 [mm/s^2, deg/s^2]
        self.robot_connected = False
        self.is_safe_position = False  # 안전한 자세 여부
        self.safe_joint_pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]  # 안전한 중간 자세
        self.gripper_initialized = False  # 그리퍼 초기화 여부
    
    def init_services(self):
        """서비스 및 클라이언트 초기화"""
        # System Manager로부터 받을 서비스 서버
        self.srv_target_pose = self.create_service(
            TargetPose, 
            '/e0509/target_pose', 
            self.srvcb_target_pose
        )
        
        self.srv_gripper_value = self.create_service(
            GripperValue, 
            '/e0509/gripper_value', 
            self.srvcb_gripper_value
        )
        
        # Doosan Robot에게 보낼 서비스 클라이언트
        self.cli_movel = self.create_client(MoveLine, '/dsr01/motion/move_line')
        self.cli_movej = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.cli_drl = self.create_client(DrlStart, '/dsr01/drl/drl_start')
        
        # System Manager에게 보낼 서비스 클라이언트
        self.cli_move_done = self.create_client(MoveDone, '/e0509/move_done')
        
        self.get_logger().info('서비스 서버 및 클라이언트 등록 완료')
    
    def init_doosan_robot(self):
        """Doosan Robot 초기화"""
        # 주기적으로 서비스 연결 확인하는 타이머 생성
        self.check_service_timer = self.create_timer(1.0, self.check_service_connection)
        self.service_check_count = 0
        self.get_logger().info('Doosan Robot 서비스 연결 확인 중...')
    
    def check_service_connection(self):
        """주기적으로 서비스 연결 상태 확인"""
        if not self.robot_connected:
            if self.cli_movel.service_is_ready():
                self.robot_connected = True
                self.get_logger().info('✅ Doosan Robot MoveLine 서비스 연결 완료')
                self.check_service_timer.cancel()
                
                # 서비스 연결 후 안전 자세 이동 시작 (비동기)
                self.get_logger().info('🏠 초기화: 안전 자세로 이동 시작')
                self.start_safe_position_move()
            else:
                self.service_check_count += 1
                if self.service_check_count % 5 == 0:  # 5초마다 로그 출력
                    self.get_logger().info(f'⏳ MoveLine 서비스 대기 중... ({self.service_check_count}초)')
                if self.service_check_count >= 10:  # 10초 후 시뮬레이션 모드
                    self.get_logger().warn('⚠️ Doosan Robot MoveLine 서비스 없음 (시뮬레이션 모드)')
                    self.check_service_timer.cancel()
    
    def start_safe_position_move(self):
        """안전 자세 이동을 비동기로 시작"""
        if not self.cli_movej.service_is_ready():
            self.get_logger().warn('⚠️ MoveJoint 서비스가 준비되지 않음')
            return
        
        self.get_logger().info(f'🏠 안전 자세로 이동: {self.safe_joint_pos}')
        
        movej_request = MoveJoint.Request()
        movej_request.pos = self.safe_joint_pos
        movej_request.vel = 30.0
        movej_request.acc = 10.0
        movej_request.time = 0.0
        movej_request.mode = 0
        movej_request.blend_type = 0
        movej_request.sync_type = 0
        
        # 비동기 호출 후 콜백 등록
        future = self.cli_movej.call_async(movej_request)
        future.add_done_callback(self.safe_position_callback)
    
    def safe_position_callback(self, future):
        """안전 자세 이동 완료 콜백"""
        try:
            result = future.result()
            if result and result.success:
                self.is_safe_position = True
                self.get_logger().info('✅ 초기 안전 자세 이동 완료')
                
                # 안전 자세 이동 후 그리퍼 초기화 시작 (비동기)
                self.get_logger().info('🔧 그리퍼 초기화 시작')
                self.initialize_gripper()
            else:
                self.get_logger().error('❌ 초기 안전 자세 이동 실패')
        except Exception as e:
            self.get_logger().error(f'❌ 안전 자세 이동 오류: {str(e)}')
    
    def move_to_safe_position(self):
        """안전한 중간 자세로 이동 (MoveJoint 사용)"""
        try:
            self.get_logger().info(f'🏠 안전 자세로 이동: {self.safe_joint_pos}')
            
            # MoveJoint 서비스 준비 확인
            if not self.cli_movej.service_is_ready():
                self.get_logger().warn('⚠️ MoveJoint 서비스가 준비되지 않음')
                import time
                time.sleep(1.0)
                if not self.cli_movej.service_is_ready():
                    self.get_logger().error('❌ MoveJoint 서비스 사용 불가')
                    return False
            
            movej_request = MoveJoint.Request()
            movej_request.pos = self.safe_joint_pos
            movej_request.vel = 30.0
            movej_request.acc = 10.0
            movej_request.time = 0.0
            movej_request.mode = 0
            movej_request.blend_type = 0
            movej_request.sync_type = 0
            
            self.get_logger().info('📞 MoveJoint 서비스 호출 중...')
            future = self.cli_movej.call_async(movej_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.done() and future.result() is not None:
                result = future.result()
                if result.success:
                    self.is_safe_position = True
                    self.get_logger().info('✅ 안전 자세 이동 완료')
                    return True
                else:
                    self.get_logger().error('❌ MoveJoint 서비스가 False 반환')
                    return False
            else:
                self.get_logger().error('❌ MoveJoint 서비스 타임아웃 또는 응답 없음')
                return False
            
        except Exception as e:
            self.get_logger().error(f'❌ 안전 자세 이동 오류: {str(e)}')
            return False
    
    def movel_callback(self, future, x, y, z):
        """MoveLine 완료 콜백"""
        try:
            result = future.result()
            if result and result.success:
                self.get_logger().info(f'✅ 로봇 이동 완료: ({x:.2f}, {y:.2f}, {z:.2f})')
                
                # System Manager에게 이동 완료 알림
                self.send_move_done(True)
            else:
                self.get_logger().error(f'❌ 로봇 이동 실패: ({x:.2f}, {y:.2f}, {z:.2f})')
                self.send_move_done(False)
                self.get_logger().error('   DSR 컨트롤러에서 알람을 확인하세요')
        except Exception as e:
            self.get_logger().error(f'❌ MoveLine 결과 처리 오류: {str(e)}')
            self.send_move_done(False)
    
    def send_move_done(self, done: bool):
        """System Manager에게 이동 완료 알림"""
        if not self.cli_move_done.service_is_ready():
            self.get_logger().warn('⚠️ MoveDone 서비스가 준비되지 않음')
            return
        
        request = MoveDone.Request()
        request.done = done
        
        future = self.cli_move_done.call_async(request)
        future.add_done_callback(lambda f: self.move_done_callback(f, done))
    
    def move_done_callback(self, future, done: bool):
        """MoveDone 서비스 응답 콜백"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ MoveDone 전송 완료: done={done}')
            else:
                self.get_logger().warn(f'⚠️ MoveDone 전송 실패')
        except Exception as e:
            self.get_logger().error(f'❌ MoveDone 콜백 오류: {str(e)}')
    
    # def quaternion_to_euler(self, x, y, z, w):
    #     """쿼터니언을 오일러 각도(roll, pitch, yaw)로 변환 (degrees)"""
    #     # Roll (x-axis rotation)
    #     sinr_cosp = 2 * (w * x + y * z)
    #     cosr_cosp = 1 - 2 * (x * x + y * y)
    #     roll = math.atan2(sinr_cosp, cosr_cosp)
        
    #     # Pitch (y-axis rotation)
    #     sinp = 2 * (w * y - z * x)
    #     if abs(sinp) >= 1:
    #         pitch = math.copysign(math.pi / 2, sinp)
    #     else:
    #         pitch = math.asin(sinp)
        
    #     # Yaw (z-axis rotation)
    #     siny_cosp = 2 * (w * z + x * y)
    #     cosy_cosp = 1 - 2 * (y * y + z * z)
    #     yaw = math.atan2(siny_cosp, cosy_cosp)
        
    #     # Convert to degrees
    #     return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def srvcb_target_pose(self, request, response):
        """TargetPose 서비스 콜백 - 로봇을 목표 위치로 이동"""
        try:
            # Pose 데이터 추출
            x = request.target_pose.position.x * 1000  # m -> mm 변환
            y = request.target_pose.position.y * 1000
            z = request.target_pose.position.z * 1000
            
            # 회전값은 고정된 안전한 자세 사용 (그리퍼가 아래를 향함)
            # rx, ry, rz = 180.0, 0.0, 0.0  # 원래 변환값
            rx, ry, rz = 0.0, 180.0, 0.0  # Z축 아래 향하는 안전한 자세
            
            self.get_logger().info(f'목표 위치 수신: x={x:.2f}mm, y={y:.2f}mm, z={z:.2f}mm')
            self.get_logger().info(f'고정 회전 사용: rx={rx:.2f}°, ry={ry:.2f}°, rz={rz:.2f}°')
            
            # 목표 포즈 설정 [x, y, z, rx, ry, rz]
            target_pose = [x, y, z, rx, ry, rz]
            
            if self.robot_connected:
                # Doosan Robotics MoveLine 서비스 호출 (비동기)
                self.get_logger().info('🤖 Doosan Robot MoveLine 서비스 호출')
                
                try:
                    # MoveLine 요청 생성
                    movel_request = MoveLine.Request()
                    movel_request.pos = target_pose
                    movel_request.vel = self.velocity
                    movel_request.acc = self.acceleration
                    movel_request.time = 0.0
                    movel_request.radius = 0.0
                    movel_request.ref = DR_BASE
                    movel_request.mode = DR_MV_MOD_ABS
                    movel_request.blend_type = 0
                    movel_request.sync_type = 0
                    
                    self.get_logger().info(f'MoveLine: pos={target_pose}')
                    
                    # 비동기 서비스 호출
                    future = self.cli_movel.call_async(movel_request)
                    future.add_done_callback(lambda f: self.movel_callback(f, x, y, z))
                    
                    response.success = True
                    response.message = f"MoveLine requested: ({x:.2f}, {y:.2f}, {z:.2f})"
                    
                except Exception as e:
                    self.get_logger().error(f'❌ MoveLine 서비스 호출 오류: {str(e)}')
                    response.success = False
                    response.message = f"MoveLine service error: {str(e)}"
            else:
                self.get_logger().warn('⚠️ 로봇이 연결되지 않음 (시뮬레이션 모드)')
                self.get_logger().info(f'[SIM] 목표 위치: {target_pose}')
                
                response.success = True
                response.message = "Simulation mode - robot not connected"
            
        except Exception as e:
            self.get_logger().error(f'TargetPose 서비스 오류: {str(e)}')
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response
    
    def initialize_gripper(self):
        """그리퍼 초기화 - gripper_controller.py의 DRL_BASE_CODE 사용"""
        if self.gripper_initialized:
            return
        
        # gripper_controller.py와 동일한 DRL_BASE_CODE
        init_script = textwrap.dedent("""
            g_slaveid = 0
            flag = 0
            def modbus_set_slaveid(slaveid):
                global g_slaveid
                g_slaveid = slaveid
                
            def modbus_fc06(address, value):
                global g_slaveid
                data = (g_slaveid).to_bytes(1, byteorder='big')
                data += (6).to_bytes(1, byteorder='big')
                data += (address).to_bytes(2, byteorder='big')
                data += (value).to_bytes(2, byteorder='big')
                return modbus_send_make(data)
                
            def modbus_fc16(startaddress, cnt, valuelist):
                global g_slaveid
                data = (g_slaveid).to_bytes(1, byteorder='big')
                data += (16).to_bytes(1, byteorder='big')
                data += (startaddress).to_bytes(2, byteorder='big')
                data += (cnt).to_bytes(2, byteorder='big')
                data += (2 * cnt).to_bytes(1, byteorder='big')
                for i in range(0, cnt):
                    data += (valuelist[i]).to_bytes(2, byteorder='big')
                return modbus_send_make(data)
                
            def recv_check():
                size, val = flange_serial_read(0.1)
                if size > 0:
                    return True, val
                else:
                    tp_log("CRC Check Fail")
                    return False, val
                    
            def gripper_move(stroke):
                flange_serial_write(modbus_fc16(282, 2, [stroke, 0]))
                wait(1.0)
            
            # ---- init serial & torque/current ----
            while True:
                flange_serial_open(
                    baudrate=57600,
                    bytesize=DR_EIGHTBITS,
                    parity=DR_PARITY_NONE,
                    stopbits=DR_STOPBITS_ONE,
                )
                
                modbus_set_slaveid(1)
                
                # 256(40257) Torque enable
                # 275(40276) Goal Current
                # 282(40283) Goal Position
                
                flange_serial_write(modbus_fc06(256, 1))   # torque enable
                flag, val = recv_check()
                
                flange_serial_write(modbus_fc06(275, 400)) # goal current
                flag, val = recv_check()
                
                if flag is True:
                    break
                
                flange_serial_close()
        """)
        
        self.get_logger().info('📝 그리퍼 초기화 DRL 전송 중...')
        self._send_drl_script_async(init_script, is_init=True)
    
    def _send_drl_script_async(self, code: str, is_init: bool = False):
        """DRL 스크립트 비동기 전송"""
        if not self.cli_drl.service_is_ready():
            self.get_logger().error('❌ DRL 서비스가 준비되지 않음')
            return
        
        request = DrlStart.Request()
        request.robot_system = 0
        request.code = code
        
        future = self.cli_drl.call_async(request)
        if is_init:
            future.add_done_callback(self._gripper_init_callback)
        else:
            future.add_done_callback(self._gripper_move_callback)
    
    def _gripper_init_callback(self, future):
        """그리퍼 초기화 결과 콜백"""
        try:
            result = future.result()
            if result and result.success:
                # 초기화 스크립트가 전송되었지만, 실제 실행은 로봇에서 진행 중
                # while True 루프가 성공할 때까지 시간이 필요함
                self.get_logger().info('📡 그리퍼 초기화 DRL 전송 성공 - 실행 대기 중...')
                
                # 초기화 실행 완료를 위한 타이머 (3초 후 완료로 간주)
                self.gripper_init_timer = self.create_timer(3.0, self._gripper_init_complete)
            else:
                self.get_logger().error('❌ 그리퍼 초기화 DRL 전송 실패')
        except Exception as e:
            self.get_logger().error(f'❌ 그리퍼 초기화 오류: {str(e)}')
    
    def _gripper_init_complete(self):
        """그리퍼 초기화 완료 처리"""
        self.gripper_initialized = True
        self.get_logger().info('✅ 그리퍼 초기화 완료 (시리얼 통신 활성화)')
        # 타이머 취소 (한 번만 실행)
        if hasattr(self, 'gripper_init_timer'):
            self.gripper_init_timer.cancel()
    
    def _gripper_move_callback(self, future):
        """그리퍼 이동 결과 콜백"""
        try:
            result = future.result()
            if result and result.success:
                self.get_logger().info('✅ 그리퍼 이동 명령 완료')
            else:
                self.get_logger().error('❌ 그리퍼 이동 명령 실패')
        except Exception as e:
            self.get_logger().error(f'❌ 그리퍼 이동 오류: {str(e)}')
    
    def srvcb_gripper_value(self, request, response):
        """GripperValue 서비스 콜백 - 그리퍼 제어"""
        try:
            gripper_value = request.value
            
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'🤏 GRIPPER CONTROL REQUEST')
            self.get_logger().info(f'   Gripper Value: {gripper_value}')
            self.get_logger().info('=' * 60)
            
            # 그리퍼 초기화 확인
            if not self.gripper_initialized:
                self.get_logger().warn('⚠️  그리퍼가 아직 초기화되지 않음')
                response.success = False
                response.message = "Gripper not initialized yet"
                return response
            
            # GUI로부터 받은 gripper_value를 그대로 stroke 값으로 사용
            stroke = gripper_value
            self.get_logger().info(f'   ➡️ 그리퍼 이동 - stroke: {stroke}')
            
            # 그리퍼 이동 - gripper_controller.py와 동일하게 전체 DRL_BASE_CODE 포함
            # 매번 초기화 루프를 실행하여 시리얼 연결 보장
            drl_base_code = textwrap.dedent("""
                g_slaveid = 0
                flag = 0
                def modbus_set_slaveid(slaveid):
                    global g_slaveid
                    g_slaveid = slaveid
                    
                def modbus_fc06(address, value):
                    global g_slaveid
                    data = (g_slaveid).to_bytes(1, byteorder='big')
                    data += (6).to_bytes(1, byteorder='big')
                    data += (address).to_bytes(2, byteorder='big')
                    data += (value).to_bytes(2, byteorder='big')
                    return modbus_send_make(data)
                    
                def modbus_fc16(startaddress, cnt, valuelist):
                    global g_slaveid
                    data = (g_slaveid).to_bytes(1, byteorder='big')
                    data += (16).to_bytes(1, byteorder='big')
                    data += (startaddress).to_bytes(2, byteorder='big')
                    data += (cnt).to_bytes(2, byteorder='big')
                    data += (2 * cnt).to_bytes(1, byteorder='big')
                    for i in range(0, cnt):
                        data += (valuelist[i]).to_bytes(2, byteorder='big')
                    return modbus_send_make(data)
                    
                def recv_check():
                    size, val = flange_serial_read(0.1)
                    if size > 0:
                        return True, val
                    else:
                        tp_log("CRC Check Fail")
                        return False, val
                        
                def gripper_move(stroke):
                    flange_serial_write(modbus_fc16(282, 2, [stroke, 0]))
                    wait(1.0)
                
                # ---- init serial & torque/current ----
                while True:
                    flange_serial_open(
                        baudrate=57600,
                        bytesize=DR_EIGHTBITS,
                        parity=DR_PARITY_NONE,
                        stopbits=DR_STOPBITS_ONE,
                    )
                    
                    modbus_set_slaveid(1)
                    
                    # 256(40257) Torque enable
                    # 275(40276) Goal Current
                    # 282(40283) Goal Position
                    
                    flange_serial_write(modbus_fc06(256, 1))   # torque enable
                    flag, val = recv_check()
                    
                    flange_serial_write(modbus_fc06(275, 400)) # goal current
                    flag, val = recv_check()
                    
                    if flag is True:
                        break
                    
                    flange_serial_close()
            """)
            
            move_script = drl_base_code + f"\ngripper_move({stroke})\n"
            
            self._send_drl_script_async(move_script, is_init=False)
            
            response.success = True
            response.message = f"Gripper command sent: stroke={stroke}"
            
        except Exception as e:
            self.get_logger().error(f'GripperValue 서비스 오류: {str(e)}')
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    nplace_controller = NPlaceController()
    
    try:
        rclpy.spin(nplace_controller)
    except KeyboardInterrupt:
        nplace_controller.get_logger().info('사용자에 의해 종료됨')
    except Exception as e:
        nplace_controller.get_logger().error(f'예외 발생: {str(e)}')
    finally:
        try:
            nplace_controller.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
