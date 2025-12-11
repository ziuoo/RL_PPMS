#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Duration

import numpy as np
import torch
import math


class PPOController(Node):
    def __init__(self):
        super().__init__("ppo_traj_controller")

        # Parameters
        self.declare_parameter("policy_path", "/home/user/colcon_ws/src/rl_controller_pkg/model/policy.pt")
        self.declare_parameter("joint_states_topic", "/dsr01/joint_states")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("traj_command_topic", "/dsr01/dsr_moveit_controller/joint_trajectory")

        policy_path = self.get_parameter("policy_path").get_parameter_value().string_value
        joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        target_pose_topic = self.get_parameter("target_pose_topic").get_parameter_value().string_value
        traj_topic = self.get_parameter("traj_command_topic").get_parameter_value().string_value

        # Joint names
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

        # Isaac Lab에서 사용한 default joint pose (관절 기준점)
        self.default_joint_pos = np.array(
            [0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0], dtype=np.float32
        )

        # Joint limits (Doosan + Isaac 설정에 맞게)
        self.joint_lower = np.array(
            [-6.196, -6.196, -2.618, -6.196, -math.radians(155), -6.196],
            dtype=np.float32,
        )
        self.joint_upper = -self.joint_lower.copy()

        # === Δ-joint action scale (Isaac Lab JointPositionActionCfg.scale 과 동일하게 맞추기) ===
        #   Isaac 쪽에서 scale = 0.07 로 학습했다면 여기서도 0.07 로 설정
        self.action_scale = 0.03  # Δq = action * action_scale

        # Control rate
        self.control_dt = 0.1  # 10 Hz

        # Trajectory duration (한 번에 보낼 joint_trajectory 길이)
        self.traj_duration = 0.2  # 0.2초짜리 트젝을 계속 덮어씀

        # ---------------- Init 상태 이동 관련 (예전 코드 그대로) ----------------
        self.is_initialized = False          # init 완료 여부
        self.init_duration = 20.0           # default pose까지 20초에 걸쳐 이동
        self.init_start_time = None         # init 시작 시각
        self.init_start_q = None            # init 시작 시의 관절 상태
        self.init_traj_sent = False         # init용 trajectory를 보냈는지 여부
        # -------------------------------------------------------------------

        # Load policy
        try:
            self.policy = torch.jit.load(policy_path, map_location="cpu")
            self.policy.eval()
            self.get_logger().info(f"[OK] Loaded policy {policy_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load TorchScript policy: {e}")
            raise e

        # State buffers
        self.current_q = None
        self.current_dq = None
        self.target_pose = None
        self.prev_action = np.zeros(6, dtype=np.float32)

        # ROS interfaces
        self.create_subscription(JointState, joint_states_topic, self.joint_state_cb, 10)
        self.create_subscription(PoseStamped, target_pose_topic, self.target_pose_cb, 10)

        self.traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # target pose TF 이름 및 부모 프레임 저장용
        self.target_frame_id = "target_pose"   # RViz에서 이 frame 선택해서 Axes 띄우면 됨
        self.target_parent_frame = "base_link" # 기본값, 실제로는 /target_pose header에서 갱신

        # Timer
        self.timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info(
            f"PPO Trajectory Controller (Δjoint version) started.\n"
            f"action_scale = {self.action_scale}, traj_duration = {self.traj_duration}s\n"
            f"Init duration = {self.init_duration}s → default pose 먼저 이동 후 PPO 제어 시작"
        )

    # --------------------------------------------------
    # Joint State Callback
    # --------------------------------------------------
    def joint_state_cb(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        try:
            q = [msg.position[name_to_idx[jn]] for jn in self.joint_names]
            dq = [msg.velocity[name_to_idx[jn]] for jn in self.joint_names]
        except KeyError as e:
            self.get_logger().error(f"JointState missing joint: {e}")
            return

        self.current_q = np.array(q, dtype=np.float32)
        self.current_dq = np.array(dq, dtype=np.float32)

    # --------------------------------------------------
    # Target Pose Callback
    # --------------------------------------------------
    def target_pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation

        # PPO용 numpy 상태는 그대로 유지
        self.target_pose = np.array(
            [p.x, p.y, p.z, o.w, o.x, o.y, o.z],
            dtype=np.float32
        )

        # 부모 프레임 (header.frame_id가 비어있으면 base_link 사용)
        self.target_parent_frame = msg.header.frame_id or "base_link"

        # 🔹 TF 브로드캐스트 (여기서는 numpy 안씀!)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.target_parent_frame      # 예: "base_link"
        t.child_frame_id = self.target_frame_id           # "target_pose"

        t.transform.translation.x = float(p.x)
        t.transform.translation.y = float(p.y)
        t.transform.translation.z = float(p.z)

        t.transform.rotation.x = float(o.x)
        t.transform.rotation.y = float(o.y)
        t.transform.rotation.z = float(o.z)
        t.transform.rotation.w = float(o.w)

        self.tf_broadcaster.sendTransform(t)

    # --------------------------------------------------
    # Main Control Loop
    # --------------------------------------------------
    def control_loop(self):
        if self.current_q is None or self.current_dq is None:
            return

        # 1) 아직 init pose로 안 갔으면 init trajectory 먼저 수행
        if not self.is_initialized:
            self._move_to_init_state()
            return

        # 2) 타겟 포즈 없으면 대기
        if self.target_pose is None:
            self.get_logger().warn(
                'Waiting for /target_pose to start RL control...',
                throttle_duration_sec=5.0,
            )
            return
        
        if self.target_pose is not None:
            p_x, p_y, p_z, q_w, q_x, q_y, q_z = self.target_pose

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

        # 3) Observation 구성 (Isaac Lab ObservationsCfg.Policy 와 동일 구조)
        #    [ (q - q_default), dq, target_pose(7), last_action(6) ]
        obs = np.concatenate(
            [
                self.current_q - self.default_joint_pos,
                self.current_dq,
                self.target_pose,
                self.prev_action,
            ]
        ).astype(np.float32)
        obs_tensor = torch.from_numpy(obs).unsqueeze(0)

        # 4) PPO Forward → Δjoint action
        with torch.no_grad():
            raw_action = self.policy(obs_tensor).cpu().numpy().flatten()

        # 정책 출력이 tanh라 가정하고 안전하게 클립
        raw_action = np.clip(raw_action, -1.0, 1.0)
        self.prev_action = raw_action.copy()

        # 5) Δq 계산 후 target joint 생성
        delta_q = raw_action * self.action_scale  # [rad]
        target_q = self.current_q + delta_q
        target_q = np.clip(target_q, self.joint_lower, self.joint_upper)

        # 6) Trajectory 메시지 생성 및 발행
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = target_q.tolist()
        point.time_from_start.sec = int(self.traj_duration)
        point.time_from_start.nanosec = int((self.traj_duration % 1.0) * 1e9)
        traj.points.append(point)

        self.traj_pub.publish(traj)

        # 7) 디버그 로그
        now = self.get_clock().now().nanoseconds / 1e9
        if not hasattr(self, "next_ctrl_log_time"):
            self.next_ctrl_log_time = 0.0

        if now >= self.next_ctrl_log_time:
            action_norm = float(np.linalg.norm(raw_action))
            delta_norm = float(np.linalg.norm(delta_q))
            tracking_error = float(np.linalg.norm(target_q - self.current_q))

            self.get_logger().info(
                f"[CTRL Δq] |a|={action_norm:.3f}, |Δq|={delta_norm:.4f} rad, "
                f"|q_target - q_curr|={tracking_error:.4f} rad, "
                f"q1={target_q[0]:.3f}, q3={target_q[2]:.3f}, q5={target_q[4]:.3f}"
            )
            self.next_ctrl_log_time = now + 0.5

    # --------------------------------------------------
    # Init pose로 천천히 이동 (이전 코드와 동일한 형태)
    # --------------------------------------------------
    def _move_to_init_state(self):
        """
        joint_trajectory_controller를 사용해서
        로봇을 default_joint_pos로 20초 동안 천천히 이동시키는 init 단계.
        """

        now = self.get_clock().now()

        # 처음 시작할 때 초기화
        if self.init_start_time is None:
            self.init_start_time = now
            self.init_start_q = self.current_q.copy() if self.current_q is not None else None
            self.init_traj_sent = False
            self.next_log_time = 0.0   # 다음 로그 출력 시간
            self.get_logger().info(f"[Init] Starting initialization: total {self.init_duration:.1f}s")
            return

        if self.current_q is None:
            # 아직 joint state를 못 받았으면 대기
            return

        elapsed = (now - self.init_start_time).nanoseconds / 1e9
        remaining = max(0.0, self.init_duration - elapsed)
        progress = min(1.0, elapsed / self.init_duration) * 100.0

        # 진행률 로그 (0.5초마다)
        if elapsed >= self.next_log_time:
            self.get_logger().info(
                f"[Init] Moving to default pose... "
                f"{elapsed:.1f}s elapsed, {remaining:.1f}s remaining "
                f"({progress:.1f}%)"
            )
            self.next_log_time = elapsed + 0.5   # 0.5초마다 출력

        # 첫 번째 trajectory 아직 안 보냈으면 보냄
        if not self.init_traj_sent:
            if self.init_start_q is None:
                return

            traj = JointTrajectory()
            traj.joint_names = self.joint_names

            num_points = 40  # 20초 동안 40포인트 → 0.5초 간격
            for i in range(num_points + 1):
                t = self.init_duration * i / num_points
                alpha = t / self.init_duration

                q = self.init_start_q + (self.default_joint_pos - self.init_start_q) * alpha
                q = np.clip(q, self.joint_lower, self.joint_upper)

                pt = JointTrajectoryPoint()
                pt.positions = q.tolist()
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                traj.points.append(pt)

            self.traj_pub.publish(traj)
            self.init_traj_sent = True

            self.get_logger().info("[Init] Initialization trajectory published.")
            return

        # init 완료 체크 (여유 1초)
        if elapsed >= self.init_duration + 1.0:
            self.is_initialized = True
            self.get_logger().info("[Init] Initialization complete. Switching to PPO control.")


def main(args=None):
    rclpy.init(args=args)
    node = PPOController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
