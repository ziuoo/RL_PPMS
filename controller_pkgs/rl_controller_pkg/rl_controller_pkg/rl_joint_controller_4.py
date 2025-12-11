#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Doosan E0509 + IsaacLab PPO 정책 Position Controller 제어 노드 (절대 position 버전)

- IsaacLab에서 JointPositionActionCfg(use_default_offset=True, scale=0.03)로 학습된 정책을
  실제 로봇에서 그대로 쓰는 것을 목표로 함.

기능 요약:
- 관측 생성/정책 추론: 20 Hz
- 실제 로봇 명령 publish: 10 Hz (/dsr01/dsr_position_controller/commands)
- IsaacLab LiftEnvCfg 관측 구조 재현:
    [ joint_pos_rel(6),
      joint_vel_rel(6),
      target_cmd(7: x,y,z,qw,qx,qy,qz),
      last_action(6) ]
  → obs dim = 25
- 정책 출력(action)을 "절대 position 정책"으로 해석:
    joint_target_raw = default_joint_pos + action_scale * action
  (IsaacLab JointPositionActionCfg 수식과 동일)
- 추가 안전장치(선택):
    - per-step joint delta 제한 (max_delta_deg_per_step)
    - low-pass filter (EMA) 로 target 완화
    - joint limit clipping
- TF2를 통해 EE(엔드 이펙터) 좌표 얻어서 타겟과 거리 계산
- EE와 Target XYZ의 거리가 10cm 이하이면 제어 종료(stop_motion = True)
"""

from __future__ import annotations

import math
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Duration

from tf2_ros import Buffer, TransformListener
from system_interfaces.srv import TargetPose, MoveDone


# =============================================================================
#  OBS BUILDER
# =============================================================================
class SpoonBussingObsBuilder:
    def __init__(self, device: str = "cpu"):
        self.device = torch.device(device)

        # ✅ IsaacLab에서 사용한 기본 joint 자세와 동일하게 맞출 것
        self.default_joint_pos = np.array(
            [0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0], dtype=np.float32
        )
        self.default_joint_vel = np.zeros(6, dtype=np.float32)
        self.last_action = np.zeros(6, dtype=np.float32)

    @staticmethod
    def _to_np(arr, expected_dim: int, name: str):
        if isinstance(arr, torch.Tensor):
            arr = arr.detach().cpu().numpy()
        arr = np.asarray(arr, dtype=np.float32).reshape(-1)
        if arr.shape[0] != expected_dim:
            raise ValueError(f"{name} 길이 {expected_dim}이어야 하는데 {arr.shape[0]} 입니다.")
        return arr

    def set_last_action(self, action):
        self.last_action[:] = self._to_np(action, 6, "action")

    def build_obs(self, joint_pos, joint_vel, target_cmd):
        # q, dq → Isaac과 동일하게 상대값으로
        q = self._to_np(joint_pos, 6, "joint_pos")
        dq = self._to_np(joint_vel, 6, "joint_vel")

        q_rel = q - self.default_joint_pos
        dq_rel = dq - self.default_joint_vel

        # target_cmd: [x,y,z,qw,qx,qy,qz] (또는 [x,y,z])
        tgt = np.asarray(target_cmd, dtype=np.float32)
        if tgt.shape[0] == 3:
            x, y, z = tgt
            tgt = np.array([x, y, z, 1, 0, 0, 0], dtype=np.float32)

        obs_np = np.concatenate([q_rel, dq_rel, tgt, self.last_action], axis=0)
        return torch.from_numpy(obs_np).unsqueeze(0).to(self.device)


# =============================================================================
#  PPO + CONTROL NODE
# =============================================================================
class PPOPositionControlNode(Node):
    def __init__(self):
        super().__init__("ppo_position_control_node")

        # ---------------------------------------------------------------------
        # 설정 값들
        # ---------------------------------------------------------------------
        self.policy_path = "/home/user/colcon_ws/src/rl_controller_pkg/model/policy_2.pt"

        # 추론 주기 / 명령 publish 비율
        self.compute_dt = 0.05  # 20 Hz
        self.publish_every_n_steps = 2  # → 10 Hz

        # ✅ IsaacLab JointPositionActionCfg.scale 과 동일해야 함!
        #   Isaac cfg에서 scale=0.03 로 썼다면 여기서도 반드시 0.03
        self.action_scale = 0.03

        # 🔹 step당 허용 delta (추가 안전장치, Isaac엔 없던 것)
        #   Isaac과 완전히 동일한 동작을 원하면 값을 아주 크게 두거나,
        #   아래 delta_clamp 부분을 주석 처리해도 됨.
        self.max_delta_deg_per_step = 15.0
        self.max_delta_rad = math.radians(self.max_delta_deg_per_step)

        # smoothing factor (EMA)
        self.filter_alpha = 0.8

        # joint limit (간단히 ±180°로 설정, 필요시 Doosan spec에 맞게 수정)
        limit = math.radians(180)
        self.joint_min = np.array([-limit] * 6, dtype=np.float32)
        self.joint_max = np.array([limit] * 6, dtype=np.float32)

        # EE-Target 거리 10cm 이하에서 정지
        self.stop_distance_threshold = 0.05  # [m]

        # STOP flag
        self.stop_motion = False

        # ---------------------------------------------------------------------
        # Policy 로드
        # ---------------------------------------------------------------------
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = device
        try:
            self.policy = torch.jit.load(self.policy_path, map_location=device)
            self.policy.eval()
            self.get_logger().info(f"[OK] Loaded policy from {self.policy_path} on {device}")
        except Exception as e:
            self.get_logger().error(f"Policy load failed: {e}")
            self.policy = None

        # OBS 빌더
        self.obs_builder = SpoonBussingObsBuilder(device=device)

        # 로봇 상태
        self.joint_pos = np.zeros(6, dtype=np.float32)
        self.joint_vel = np.zeros(6, dtype=np.float32)
        self.state_ready = False

        # 초기 target (dummy)
        self.target_cmd = None
        self.filtered_target = None
        self.step_count = 0

        # Init state 관련
        self.is_initialized = False
        self.init_duration = 10.0
        self.init_start_time = None
        self.init_start_q = None
        self.init_traj_sent = False
        
        # 로그용 시간
        self.next_ctrl_log_time = 0.0
        self.next_init_log_time = 0.0
        
        # joint 이름
        self.joint_names = [f"joint_{i+1}" for i in range(6)]
        
        # joint limits (rl_joint_controller_2.py에서 가져옴)
        self.joint_lower = np.array(
            [-6.196, -6.196, -2.618, -6.196, -math.radians(155), -6.196],
            dtype=np.float32
        )
        self.joint_upper = -self.joint_lower.copy()
        
        # default joint pos (obs_builder와 동일)
        self.default_joint_pos = self.obs_builder.default_joint_pos

        # TF listener (EE pose 계산용)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------------------------------------------------------------
        # ROS I/O
        # ---------------------------------------------------------------------
        self.create_subscription(JointState, "/dsr01/joint_states", self.joint_state_cb, 10)
        
        # TargetPose 서비스 서버 생성
        self.target_pose_srv = self.create_service(
            TargetPose,
            "/e0509/pick_pose",
            self.target_pose_service_cb
        )
        
        # MoveDone 서비스 클라이언트 생성
        self.move_done_client = self.create_client(MoveDone, "/e0509/move_done")

        # Doosan trajectory controller 명령 토픽
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/dsr01/dsr_moveit_controller/joint_trajectory",
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.target_frame_id = "target_pose"   # RViz에서 이 frame 선택해서 Axes 띄우면 됨
        self.target_parent_frame = "base_link" # 기본값, 실제로는 /target_pose header에서 갱신

        # 20Hz 메인 타이머 (제어 가능하게 변수로 저장)
        self.compute_timer = self.create_timer(self.compute_dt, self.compute_loop)

        self.get_logger().info(
            f"PPOPositionControlNode started.\n"
            f"  action_scale = {self.action_scale}\n"
            f"  max_delta_deg_per_step = {self.max_delta_deg_per_step}\n"
            f"  stop_distance_threshold = {self.stop_distance_threshold} m"
        )

    # =====================================================================
    # CALLBACKS
    # =====================================================================
    def joint_state_cb(self, msg: JointState):
        # 조인트 순서가 joint_1~6와 동일하다고 가정
        self.joint_pos = np.array(msg.position[:6], dtype=np.float32)
        self.joint_vel = np.array(msg.velocity[:6], dtype=np.float32)

        if not self.state_ready:
            # 첫 상태 들어왔을 때 filtered_target 초기화
            self.filtered_target = self.joint_pos.copy()
            self.state_ready = True

    def target_pose_service_cb(self, request: TargetPose.Request, response: TargetPose.Response):
        """
        TargetPose 서비스 콜백: 목표 위치를 설정하고 응답 반환
        """
        try:
            p = request.target_pose.position
            q = request.target_pose.orientation
            
            self.target_cmd = np.array(
                [p.x, p.y, p.z, q.w, q.x, q.y, q.z],
                dtype=np.float32,
            )

            # 부모 프레임은 base_link로 고정
            self.target_parent_frame = "base_link"

            # 🔹 TF 브로드캐스트
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.target_parent_frame
            t.child_frame_id = self.target_frame_id

            t.transform.translation.x = float(p.x)
            t.transform.translation.y = float(p.y)
            t.transform.translation.z = float(p.z)

            t.transform.rotation.x = float(q.x)
            t.transform.rotation.y = float(q.y)
            t.transform.rotation.z = float(q.z)
            t.transform.rotation.w = float(q.w)

            self.tf_broadcaster.sendTransform(t)
            
            # 성공 응답
            response.success = True
            response.message = f"Target pose set: [{p.x:.3f}, {p.y:.3f}, {p.z:.3f}, {q.w:.3f}, {q.x:.3f}, {q.y:.3f}, {q.z:.3f}]"
            self.get_logger().info(f"[Service] {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to set target pose: {str(e)}"
            self.get_logger().error(f"[Service] {response.message}")
        
        return response
    
    def _call_move_done_service(self, done: bool, message: str):
        """
        MoveDone 서비스를 호출하여 완료 상태 전송
        """
        if not self.move_done_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("[MoveDone] Service not available")
            return
        
        request = MoveDone.Request()
        request.done = done
        
        future = self.move_done_client.call_async(request)
        future.add_done_callback(
            lambda f: self._move_done_response_cb(f, message)
        )
    
    def _move_done_response_cb(self, future, message: str):
        """
        MoveDone 서비스 응답 콜백
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"[MoveDone] {message} - Server response: {response.message}")
            else:
                self.get_logger().warn(f"[MoveDone] Service call failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"[MoveDone] Service call exception: {e}")

    # =====================================================================
    # MAIN LOOP (20 Hz)
    # =====================================================================
    def compute_loop(self):
        if not self.state_ready or self.policy is None:
            return
        
        if not self.is_initialized:
            self._move_to_init_state()
            return
        
        now = self.get_clock().now().nanoseconds / 1e9
        if self.target_cmd is None:
            if now >= self.next_ctrl_log_time:
                self.get_logger().warn(
                    "[CTRL] Waiting for /target_pose (target_pose is None)"
                )
                self.next_ctrl_log_time = now + 2.0
            return
        
        if self.target_cmd is not None:
            p_x, p_y, p_z, q_w, q_x, q_y, q_z = self.target_cmd

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.target_parent_frame   # 예: base_link
            t.child_frame_id = self.target_frame_id        # 예: target_pose

            t.transform.translation.x = float(p_x)
            t.transform.translation.y = float(p_y)
            t.transform.translation.z = float(p_z)
            t.transform.rotation.w = float(q_w)
            t.transform.rotation.x = float(q_x)
            t.transform.rotation.y = float(q_y)
            t.transform.rotation.z = float(q_z)

            self.tf_broadcaster.sendTransform(t)

        # STOP 상태면 로봇 명령 중단
        if self.stop_motion:
            return

        self.step_count += 1

        # ----------------------------------------------------------
        # 1) PPO POLICY INFERENCE
        # ----------------------------------------------------------
        obs = self.obs_builder.build_obs(self.joint_pos, self.joint_vel, self.target_cmd)

        with torch.no_grad():
            action = self.policy(obs)[0].cpu().numpy()

        # last_action 업데이트 (다음 step obs에 사용)
        self.obs_builder.set_last_action(action)

        # ----------------------------------------------------------
        # 2) IsaacLab JointPositionActionCfg 와 동일한 해석
        #    q_target = default_joint_pos + scale * action
        # ----------------------------------------------------------
        default_q = self.obs_builder.default_joint_pos
        joint_target_raw = default_q + self.action_scale * action

        # ----------------------------------------------------------
        # 3) per-step delta 제한 (추가 안전장치)
        #    Isaac과 완전히 같게 하려면 이 부분 제거 가능.
        # ----------------------------------------------------------
        delta = joint_target_raw - self.joint_pos
        delta_clamped = np.clip(delta, -self.max_delta_rad, self.max_delta_rad)
        joint_target_safe = self.joint_pos + delta_clamped

        # ----------------------------------------------------------
        # 4) TF → EE 좌표 얻기 & 타겟까지 거리 계산
        # ----------------------------------------------------------
        try:
            tf_trans = self.tf_buffer.lookup_transform(
                "base_link", "link_6", Time()
            )
            ee_x = tf_trans.transform.translation.x
            ee_y = tf_trans.transform.translation.y
            ee_z = tf_trans.transform.translation.z

            tgt_x, tgt_y, tgt_z = self.target_cmd[0], self.target_cmd[1], self.target_cmd[2]
            dx = ee_x - tgt_x
            dy = ee_y - tgt_y
            dz = ee_z - tgt_z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            if dist < self.stop_distance_threshold:
                self.get_logger().info(
                    f"🎉 EE reached target within {self.stop_distance_threshold*100:.0f}cm → STOPPING. "
                    f"dist={dist:.3f} m"
                )
                self.stop_motion = True
                
                # 🔹 현재 위치로 짧은 trajectory 전송 (로봇 정지)
                stop_traj = JointTrajectory()
                stop_traj.joint_names = [f"joint_{i+1}" for i in range(6)]
                stop_point = JointTrajectoryPoint()
                stop_point.positions = self.joint_pos.tolist()  # 현재 위치 유지
                stop_point.time_from_start.sec = 0
                stop_point.time_from_start.nanosec = int(0.1 * 1e9)  # 0.1초
                stop_traj.points.append(stop_point)
                self.traj_pub.publish(stop_traj)
                self.get_logger().info("🛑 Stop command sent (holding current position)")
                
                # 타이머 정지 - 더 이상 명령 발행 중단
                if self.compute_timer is not None:
                    self.compute_timer.cancel()
                    self.get_logger().info("⏸️ Control timer stopped - no more commands will be sent")
                
                # MoveDone 서비스 호출
                self._call_move_done_service(True, f"Target reached. Distance: {dist:.4f}m")
                
                return
        except Exception:
            # TF가 아직 안 뜬 경우 등은 그냥 무시
            pass

        # ----------------------------------------------------------
        # 5) smoothing + joint limit
        # ----------------------------------------------------------
        if self.filtered_target is None:
            self.filtered_target = joint_target_safe.copy()
        else:
            self.filtered_target = (
                self.filter_alpha * self.filtered_target
                + (1.0 - self.filter_alpha) * joint_target_safe
            )

        joint_target_final = np.clip(self.filtered_target, self.joint_min, self.joint_max)

        # ----------------------------------------------------------
        # 6) 10Hz로 명령 publish (JointTrajectory 형식)
        # ----------------------------------------------------------
        if self.step_count % self.publish_every_n_steps == 0:
            traj = JointTrajectory()
            traj.joint_names = [f"joint_{i+1}" for i in range(6)]
            
            point = JointTrajectoryPoint()
            point.positions = joint_target_final.tolist()
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int(0.4 * 1e9)  # 0.4초
            
            traj.points.append(point)
            self.traj_pub.publish(traj)

            jp = [round(float(v), 3) for v in self.joint_pos]
            jt = [round(float(v), 3) for v in joint_target_final]

            self.get_logger().info(
                f"\n===== CONTROL (10Hz) STEP {self.step_count} =====\n"
                f"Current Joints: {jp}\n"
                f"Target  Joints: {jt}\n"
                f"===============================================\n"
            )

    def _move_to_init_state(self):
        now = self.get_clock().now()

        # 첫 호출 시 셋업
        if self.init_start_time is None:
            if self.joint_pos is None:
                return
            self.init_start_time = now
            self.init_start_q = self.joint_pos.copy()
            self.init_traj_sent = False
            self.next_init_log_time = 0.0
            self.get_logger().info(
                f"[Init] Starting initialization: total {self.init_duration:.1f}s"
            )
            return

        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        remaining = max(0.0, self.init_duration - elapsed)
        progress = min(1.0, elapsed / self.init_duration) * 100.0

        if elapsed >= self.next_init_log_time:
            self.get_logger().info(
                f"[Init] Moving to default pose... "
                f"{elapsed:.1f}s elapsed, {remaining:.1f}s remaining "
                f"({progress:.1f}%)"
            )
            self.next_init_log_time = elapsed + 0.5

        # 첫 trajectory 한번만 보냄
        if not self.init_traj_sent:
            if self.joint_pos is None:
                return

            traj = JointTrajectory()
            traj.joint_names = self.joint_names

            num_points = 40   # 10초 동안 40포인트 → 0.25s 간격
            for i in range(num_points + 1):
                t = self.init_duration * i / num_points
                alpha = t / self.init_duration

                q = self.init_start_q + (self.default_joint_pos - self.init_start_q) * alpha
                q = np.clip(q, self.joint_lower, self.joint_upper)

                pt = JointTrajectoryPoint()
                pt.positions = q.tolist()
                pt.time_from_start = Duration(
                    sec=int(t),
                    nanosec=int((t % 1.0) * 1e9)
                )
                traj.points.append(pt)

            self.traj_pub.publish(traj)
            self.init_traj_sent = True
            self.get_logger().info("[Init] Initialization trajectory published.")
            return

        # init 완료 판정
        if elapsed >= self.init_duration + 1.0:
            self.is_initialized = True
            self.get_logger().info("[Init] Initialization complete. Switching to PPO control.")


# =============================================================================
# MAIN
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = PPOPositionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
