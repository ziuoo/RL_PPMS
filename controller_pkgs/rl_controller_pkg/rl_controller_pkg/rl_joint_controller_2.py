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

        # -----------------------------
        #  파라미터
        # -----------------------------
        self.declare_parameter("policy_path", "/home/user/colcon_ws/src/rl_controller_pkg/model/policy.pt")
        self.declare_parameter("joint_states_topic", "/dsr01/joint_states")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("traj_command_topic", "/dsr01/dsr_moveit_controller/joint_trajectory")

        policy_path = self.get_parameter("policy_path").get_parameter_value().string_value
        joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        target_pose_topic = self.get_parameter("target_pose_topic").get_parameter_value().string_value
        traj_topic = self.get_parameter("traj_command_topic").get_parameter_value().string_value

        # 조인트 이름
        self.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]

        # Isaac Lab에서 사용한 기본 자세 (초기자세 기준)
        self.default_joint_pos = np.array(
            [0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0], dtype=np.float32
        )

        # 조인트 한계
        self.joint_lower = np.array(
            [-6.196, -6.196, -2.618, -6.196, -math.radians(155), -6.196],
            dtype=np.float32
        )
        self.joint_upper = -self.joint_lower.copy()

        # PPO 액션 스케일
        #  -> 델타 방식이기 때문에 꽤 작게 잡는 게 안전함
        self.action_scale = 0.03  # rad (≈ 1.1 deg)

        # 한 스텝에서 허용하는 최대 델타 (추가 안전장치)ms
        self.max_delta_step = 0.02  # rad

        # 제어 주기 / trajectory duration
        self.control_dt = 0.1       # 10 Hz
        self.traj_duration = 0.4    # 0.4초 정도로 부드럽게

        # Goal 판정 (조인트 기준)
        self.goal_pos_tol = 0.01    # rad
        self.goal_vel_tol = 0.02    # rad/s
        self.goal_hold_time = 1.0   # s

        self.goal_reached = False
        self.goal_reached_since = None

        # 상태 변수
        self.current_q = None
        self.current_dq = None
        self.target_pose = None
        self.prev_action = np.zeros(6, dtype=np.float32)

        # Init state 관련
        self.is_initialized = False
        self.init_duration = 10.0
        self.init_start_time = None
        self.init_start_q = None
        self.init_traj_sent = False

        # 로그용 시간
        self.next_ctrl_log_time = 0.0
        self.next_init_log_time = 0.0

        # PPO policy 로드
        try:
            self.policy = torch.jit.load(policy_path, map_location="cpu")
            self.policy.eval()
            self.get_logger().info(f"[OK] Loaded policy {policy_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load TorchScript policy: {e}")
            raise e

        # ROS 인터페이스
        self.create_subscription(JointState, joint_states_topic, self.joint_state_cb, 10)
        self.create_subscription(PoseStamped, target_pose_topic, self.target_pose_cb, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # target pose TF 이름 및 부모 프레임 저장용
        self.target_frame_id = "target_pose"   # RViz에서 이 frame 선택해서 Axes 띄우면 됨
        self.target_parent_frame = "base_link" # 기본값, 실제로는 /target_pose header에서 갱신



        # 타이머
        self.timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info(
            f"PPO Trajectory Controller started.\n"
            f"  traj_duration = {self.traj_duration}s\n"
            f"  action_scale  = {self.action_scale}\n"
            f"  max_delta_step= {self.max_delta_step}"
        )

    # -----------------------------
    # 콜백들
    # -----------------------------
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




    # -----------------------------
    # 메인 제어 루프
    # -----------------------------
    def control_loop(self):
        if self.current_q is None or self.current_dq is None:
            return

        # 1) 초기 자세로 이동 중이면 그거 먼저
        if not self.is_initialized:
            self._move_to_init_state()
            return

        # 2) 이미 goal에 도달했다면 더 이상 trajectory 안 보냄
        now = self.get_clock().now().nanoseconds / 1e9
        if self.goal_reached:
            if now >= self.next_ctrl_log_time:
                self.get_logger().info("[CTRL] Goal reached — holding.")
                self.next_ctrl_log_time = now + 2.0
            return

        if self.target_pose is None:
            if now >= self.next_ctrl_log_time:
                self.get_logger().warn(
                    "[CTRL] Waiting for /target_pose (target_pose is None)"
                )
                self.next_ctrl_log_time = now + 2.0
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

        # 관찰 구성 (학습 때와 동일 순서 가정)
        obs = np.concatenate([
            self.current_q - self.default_joint_pos,
            self.current_dq,
            self.target_pose,
            self.prev_action
        ])
        obs_tensor = torch.from_numpy(obs).float().unsqueeze(0)

        # 3) PPO forward
        with torch.no_grad():
            raw_action = self.policy(obs_tensor).cpu().numpy().flatten()

        # 클리핑 전/후 크기
        raw_mean = float(np.mean(np.abs(raw_action)))
        raw_action = np.clip(raw_action, -1.0, 1.0)
        clipped_mean = float(np.mean(np.abs(raw_action)))

        self.prev_action = raw_action.copy()

        # 4) 델타 joint 만들기 (현재 q 기준)
        delta_q = raw_action * self.action_scale
        # step당 최대 변화 제한
        delta_q = np.clip(delta_q, -self.max_delta_step, self.max_delta_step)

        # 목표 조인트 = 현재 + delta
        target_q = self.current_q + delta_q
        target_q = np.clip(target_q, self.joint_lower, self.joint_upper)

        # 5) Trajectory message 생성
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = target_q.tolist()
        point.time_from_start.sec = int(self.traj_duration)
        point.time_from_start.nanosec = int((self.traj_duration % 1.0) * 1e9)
        traj.points.append(point)

        self.traj_pub.publish(traj)

        # 6) Goal 판정 (조인트 기준)
        tracking_error = float(np.linalg.norm(target_q - self.current_q))
        vel_norm = float(np.linalg.norm(self.current_dq))

        if tracking_error < self.goal_pos_tol and vel_norm < self.goal_vel_tol:
            if self.goal_reached_since is None:
                self.goal_reached_since = now
            elif now - self.goal_reached_since > self.goal_hold_time:
                self.goal_reached = True
                self.get_logger().info(
                    f"[GOAL] Reached. err={tracking_error:.4f}, vel={vel_norm:.4f}"
                )
        else:
            self.goal_reached_since = None
            self.goal_reached = False

        # 7) 로그 (0.5초마다)
        if now >= self.next_ctrl_log_time:
            self.get_logger().info(
                f"[CTRL] RawMean={raw_mean:.3f} ClipMean={clipped_mean:.3f}  "
                f"Err={tracking_error:.4f}  "
                f"DeltaNorm={float(np.linalg.norm(delta_q)):.4f}  "
                f"TJ1={target_q[0]:.3f}, J3={target_q[2]:.3f}, J5={target_q[4]:.3f}"
            )
            self.next_ctrl_log_time = now + 0.5

    # -----------------------------
    # 초기 자세로 천천히 이동
    # -----------------------------
    def _move_to_init_state(self):
        now = self.get_clock().now()

        # 첫 호출 시 셋업
        if self.init_start_time is None:
            if self.current_q is None:
                return
            self.init_start_time = now
            self.init_start_q = self.current_q.copy()
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
            if self.current_q is None:
                return

            traj = JointTrajectory()
            traj.joint_names = self.joint_names

            num_points = 40   # 20초 동안 40포인트 → 0.5s 간격
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
