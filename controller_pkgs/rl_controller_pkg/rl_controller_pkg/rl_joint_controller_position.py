#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

import numpy as np
import torch
import math


class PPOController(Node):
    """
    Isaac Lab에서 학습한 PPO 정책(policy.pt, TorchScript)을
    실제 Doosan 로봇(/dsr01/dsr_position_controller/commands)에 적용하는 ROS2 노드.
    """

    def __init__(self):
        super().__init__('ppo_controller')

        # ======================
        # 1) 파라미터 설정
        # ======================
        self.declare_parameter(
            'policy_path',
            '/home/user/colcon_ws/src/rl_controller_pkg/model/policy.pt'
        )
        self.declare_parameter('target_pose_topic', '/target_pose')
        self.declare_parameter('joint_states_topic', '/dsr01/joint_states')
        self.declare_parameter(
            'command_topic',
            '/dsr01/dsr_position_controller/commands'
        )

        policy_path = self.get_parameter('policy_path').get_parameter_value().string_value
        joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        target_pose_topic = self.get_parameter('target_pose_topic').get_parameter_value().string_value
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value

        # joint 이름 (Isaac Lab, dsr 패키지에서 사용하는 이름과 동일해야 함)
        self.joint_names = ['joint_1', 'joint_2', 'joint_3',
                            'joint_4', 'joint_5', 'joint_6']

        # Isaac Lab에서 사용한 기본 자세 (default_joint_pos)
        self.default_joint_pos = np.array([
            0.0,      # joint_1
            0.0,      # joint_2
            1.5708,   # joint_3 (90deg)
            0.0,      # joint_4
            1.5708,   # joint_5 (90deg)
            0.0       # joint_6
        ], dtype=np.float32)

        # joint limits (5도 margin 반영, rad)
        # TODO: 나중에는 e0509 실제 joint limit로 전부 교체하는 게 베스트
        self.joint_lower = np.array(
            [-6.196, -6.196, -2.618, -6.196, -math.radians(155.0), -6.196],
            dtype=np.float32
        )
        self.joint_upper = -self.joint_lower.copy()

        # PPO 학습 시 사용한 action scale (실로봇에서는 보수적으로)
        self.action_scale = 0.005  # rad

        # 제어 주기 및 속도 제한 (rad/step)
        self.control_dt = 0.1          # 10 Hz
        self.max_vel_deg = 150.0        # 관절 속도 목표 상한 (225보다 약간 낮게)
        self.max_vel = math.radians(self.max_vel_deg)   # rad/s
        # self.max_joint_delta = self.max_vel * self.control_dt  # rad/step
        self.max_joint_delta = 0.005

        self.get_logger().info(
            f"Velocity limit: {self.max_vel_deg:.1f} deg/s -> "
            f"max_delta = {self.max_joint_delta:.4f} rad/step"
        )

        # ======================
        # 2) JIT policy 로드
        # ======================
        try:
            self.policy = torch.jit.load(policy_path, map_location='cpu')
            self.policy.eval()
            self.get_logger().info(f'Loaded TorchScript policy from: {policy_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load policy: {e}')
            raise e

        # ======================
        # 3) 상태 변수
        # ======================
        self.current_q = None          # joint position (6,)
        self.current_dq = None         # joint velocity (6,)
        self.target_pose = None        # [x, y, z, qw, qx, qy, qz]
        self.prev_action = np.zeros(6, dtype=np.float32)  # 지난 step raw_action (-1~1)
        self.prev_cmd = None           # 이전에 보낸 joint 명령 (속도 제한용)

        # 저역 필터링용 (정책 출력 smoothing)
        self.filtered_action = np.zeros(6, dtype=np.float32)
        self.smoothing_alpha = 0.3  # 0~1, 작을수록 더 부드럽게 (0.2~0.5 사이 추천)

        # 목표 근처 hold 조건 (joint 기준)
        self.hold_pos_threshold = 0.002   # rad, 약 0.1도
        self.hold_vel_threshold = 0.05    # rad/s, 아주 느리게 움직일 때만 hold

        # Init state로 이동하기 위한 상태 관리
        self.is_initialized = False
        self.init_duration = 20.0      # init state로 이동하는 데 걸리는 시간 (s)
        self.init_start_time = None
        self.init_start_q = None

        # ======================
        # 4) ROS 인터페이스
        # ======================
        self.create_subscription(JointState, joint_states_topic,
                                 self.joint_state_cb, 10)
        self.create_subscription(PoseStamped, target_pose_topic,
                                 self.target_pose_cb, 10)

        self.cmd_pub = self.create_publisher(Float64MultiArray,
                                             command_topic, 10)

        # 25 Hz 제어 루프
        self.timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info(
            f"PPOController started.\n"
            f"  policy_path       : {policy_path}\n"
            f"  joint_states_topic: {joint_states_topic}\n"
            f"  target_pose_topic : {target_pose_topic}\n"
            f"  command_topic     : {command_topic}"
        )

    # ======================
    # 콜백들
    # ======================
    def joint_state_cb(self, msg: JointState):
        if not msg.name:
            self.get_logger().warn('JointState message has empty name list.')
            return

        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        q = []
        dq = []
        try:
            for jn in self.joint_names:
                idx = name_to_idx[jn]
                q.append(msg.position[idx])
                if len(msg.velocity) > idx:
                    dq.append(msg.velocity[idx])
                else:
                    dq.append(0.0)
        except KeyError as e:
            self.get_logger().error(
                f'Joint name {e} not found in JointState. '
                f'Available: {msg.name}'
            )
            return

        self.current_q = np.array(q, dtype=np.float32)
        self.current_dq = np.array(dq, dtype=np.float32)

    def target_pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        self.target_pose = np.array(
            [p.x, p.y, p.z, o.w, o.x, o.y, o.z],
            dtype=np.float32
        )

    # ======================
    # 메인 제어 루프
    # ======================
    def control_loop(self):
        if self.current_q is None or self.current_dq is None:
            return

        # 1) 먼저 init pose로 천천히 이동
        if not self.is_initialized:
            self._move_to_init_state()
            return

        if self.target_pose is None:
            # RL 제어는 target_pose가 들어온 뒤 시작
            self.get_logger().warn(
                'Waiting for /target_pose to start RL control...',
                throttle_duration_sec=5.0
            )
            return

        # 2) Observation 25차원 구성
        joint_pos_rel = self.current_q - self.default_joint_pos   # (6,)
        joint_vel = self.current_dq                               # (6,)
        target = self.target_pose                                 # (7,)
        prev_action = self.prev_action                            # (6,)

        obs_np = np.concatenate(
            [joint_pos_rel, joint_vel, target, prev_action],
            axis=0
        )
        if obs_np.shape[0] != 25:
            self.get_logger().error(
                f'Obs dim mismatch: got {obs_np.shape[0]}, expected 25.'
            )
            return

        obs_tensor = torch.from_numpy(obs_np).float().unsqueeze(0)

        # 3) 정책 호출 → raw_action (-1~1)
        with torch.inference_mode():
            try:
                raw_action_tensor = self.policy(obs_tensor)
            except Exception as e:
                self.get_logger().error(f'Policy forward failed: {e}')
                return

        raw_action = raw_action_tensor.cpu().numpy().flatten()  # (6,)

        # 저역 필터링: 정책 출력을 부드럽게 (고주파 진동 감소)
        # alpha = self.smoothing_alpha
        # self.filtered_action = (
        #     alpha * raw_action + (1.0 - alpha) * self.filtered_action
        # )
        # used_action = self.filtered_action  # 이후 연산에 이 값을 사용

        used_action = raw_action  # 이후 연산에 이 값을 사용

        # 4) default + scale + joint limit
        processed = self.default_joint_pos + used_action * self.action_scale
        processed = np.clip(processed, self.joint_lower, self.joint_upper)

        if not np.all(np.isfinite(processed)):
            self.get_logger().warn(
                f"Non-finite command detected: {processed}, skipping publish."
            )
            return

        # 5) 속도 제한: previous command와의 차이를 제한
        if self.prev_cmd is None:
            # RL 시작 직후에는 현재 joint 상태를 기준으로 시작
            self.prev_cmd = self.current_q.copy()

        delta = processed - self.prev_cmd
        delta_clipped = np.clip(delta,
                                -self.max_joint_delta,
                                self.max_joint_delta)
        safe_cmd = self.prev_cmd + delta_clipped
        safe_cmd = np.clip(safe_cmd, self.joint_lower, self.joint_upper)

        if not np.all(np.isfinite(safe_cmd)):
            self.get_logger().warn(
                f"Non-finite safe_cmd detected: {safe_cmd}, skipping publish."
            )
            return
        
        # 목표 근처 hold: 변화가 아주 작고, 관절 속도도 거의 없으면 명령 고정
        if self.prev_cmd is not None:
            pos_diff = np.linalg.norm(safe_cmd - self.prev_cmd)
            vel_norm = np.linalg.norm(self.current_dq)

            if (pos_diff < self.hold_pos_threshold) and (vel_norm < self.hold_vel_threshold):
                # 거의 멈춰 있는 상태 → 이전 명령 유지 (추가 흔들림 방지)
                safe_cmd = self.prev_cmd.copy()
                # 원하면 한번만 로그 찍고 싶으면 이 정도:
                # self.get_logger().info("Near target - holding position.", throttle_duration_sec=2.0)

        # 명령 publish
        cmd_msg = Float64MultiArray()
        cmd_msg.data = safe_cmd.tolist()
        self.cmd_pub.publish(cmd_msg)

        # 상태 업데이트
        self.prev_action = used_action.copy()
        self.prev_cmd = safe_cmd.copy()

    def _move_to_init_state(self):
        """
        로봇을 default_joint_pos로 부드럽게 이동시킴 (linear interpolation).
        """
        current_time = self.get_clock().now()

        # 첫 호출 시 시작 시간과 시작 위치 기록
        if self.init_start_time is None:
            self.init_start_time = current_time
            self.init_start_q = self.current_q.copy()
            self.prev_cmd = self.current_q.copy()
            self.get_logger().info(
                f'Starting initialization: moving to default pose in '
                f'{self.init_duration}s'
            )

        elapsed = (current_time - self.init_start_time).nanoseconds / 1e9

        if elapsed >= self.init_duration:
            # 초기화 완료
            self.is_initialized = True
            self.get_logger().info('Initialization complete. RL control starting...')
            # 마지막으로 default position을 한 번 더 보내되, 속도 제한 적용
            target_q = self.default_joint_pos.copy()
            delta = target_q - self.prev_cmd
            delta_clipped = np.clip(delta,
                                    -self.max_joint_delta,
                                    self.max_joint_delta)
            target_q = self.prev_cmd + delta_clipped
            target_q = np.clip(target_q, self.joint_lower, self.joint_upper)

            self.prev_cmd = target_q.copy()
            cmd_msg = Float64MultiArray()
            cmd_msg.data = target_q.tolist()
            self.cmd_pub.publish(cmd_msg)
            return

        # Linear interpolation: q(t) = q_start + (q_target - q_start) * (t / T)
        alpha = elapsed / self.init_duration
        target_q = self.init_start_q + (self.default_joint_pos - self.init_start_q) * alpha
        target_q = np.clip(target_q, self.joint_lower, self.joint_upper)

        # 속도 제한
        if self.prev_cmd is None:
            self.prev_cmd = self.current_q.copy()

        delta = target_q - self.prev_cmd
        delta_clipped = np.clip(delta,
                                -self.max_joint_delta,
                                self.max_joint_delta)
        target_q = self.prev_cmd + delta_clipped
        target_q = np.clip(target_q, self.joint_lower, self.joint_upper)

        self.prev_cmd = target_q.copy()

        cmd_msg = Float64MultiArray()
        cmd_msg.data = target_q.tolist()
        self.cmd_pub.publish(cmd_msg)

    def reset_policy_state(self):
        self.prev_action[:] = 0.0
        self.prev_cmd = None


def main(args=None):
    rclpy.init(args=args)
    node = PPOController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
