#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import torch

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration

from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time


class PPOController(Node):
    def __init__(self):
        super().__init__("ppo_traj_controller")

        # ======================
        # 1) 파라미터 설정
        # ======================
        self.declare_parameter("policy_path", "/home/user/colcon_ws/src/rl_controller_pkg/model/policy.pt")
        self.declare_parameter("joint_states_topic", "/dsr01/joint_states")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("traj_command_topic", "/dsr01/dsr_moveit_controller/joint_trajectory")

        # EE pose 계산용 frame 이름 (필요시 launch에서 override)
        self.declare_parameter("base_frame", "base_link")      # 실제 TF에서 확인해서 수정
        self.declare_parameter("ee_frame", "link_6")       # 예: "link6", "tool0", "tcp" 등

        policy_path = self.get_parameter("policy_path").get_parameter_value().string_value
        joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        target_pose_topic = self.get_parameter("target_pose_topic").get_parameter_value().string_value
        traj_topic = self.get_parameter("traj_command_topic").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value

        # ======================
        # 2) 로봇 / PPO 설정
        # ======================
        self.joint_names = ["joint_1", "joint_2", "joint_3",
                            "joint_4", "joint_5", "joint_6"]

        # Isaac Lab에서 사용한 기본 자세
        self.default_joint_pos = np.array(
            [0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0],
            dtype=np.float32
        )

        # joint limits (간단 버전)
        self.joint_lower = np.array(
            [-6.196, -6.196, -2.618, -6.196, -math.radians(155.0), -6.196],
            dtype=np.float32
        )
        self.joint_upper = -self.joint_lower.copy()

        # PPO scaling (Isaac Lab에서 학습한 값에 맞게)
        self.action_scale = 0.03

        # 제어 주기 / 트젝 duration
        self.control_dt = 0.1     # 10 Hz
        self.traj_duration = 1.0  # 하나의 목표까지 1초에 도달하도록 (가속도 프로파일은 controller가 알아서)

        # ======================
        # 3) PPO policy 로드
        # ======================
        try:
            self.policy = torch.jit.load(policy_path, map_location="cpu")
            self.policy.eval()
            self.get_logger().info(f"[OK] Loaded policy {policy_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load TorchScript policy: {e}")
            raise e

        # ======================
        # 4) 상태 변수
        # ======================
        self.current_q = None
        self.current_dq = None
        self.target_pose = None       # [x, y, z, qw, qx, qy, qz]
        self.prev_action = np.zeros(6, dtype=np.float32)

        # Init state (default pose)로 가는 trajectory 관리
        self.is_initialized = False
        self.init_duration = 20.0
        self.init_start_time = None
        self.init_start_q = None
        self.init_traj_sent = False
        self.next_init_log_time = 0.0

        # Goal (목표 pose 도달) 판정용
        self.goal_reached = False
        self.goal_reached_since = None
        self.goal_pos_tol = 0.01    # [m]
        self.goal_ori_tol = 0.05     # quat dot 기반
        self.goal_vel_tol = 0.05     # [rad/s]
        self.goal_hold_time = 1.0    # [s] 이 시간 이상 유지되면 “도달”

        # ======================
        # 5) ROS 인터페이스
        # ======================
        self.create_subscription(JointState, joint_states_topic, self.joint_state_cb, 10)
        self.create_subscription(PoseStamped, target_pose_topic, self.target_pose_cb, 10)

        self.traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)

        # TF (EE pose 계산용)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer
        self.timer = self.create_timer(self.control_dt, self.control_loop)

        # CTRL 로그용
        self.next_ctrl_log_time = 0.0

        self.get_logger().info(
            f"PPO Trajectory Controller started.\n"
            f"  traj_topic    : {traj_topic}\n"
            f"  base_frame    : {self.base_frame}\n"
            f"  ee_frame      : {self.ee_frame}\n"
            f"  traj_duration : {self.traj_duration}s"
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
        self.target_pose = np.array(
            [p.x, p.y, p.z, o.w, o.x, o.y, o.z],
            dtype=np.float32
        )
        # 새로운 target 들어오면 goal 플래그 리셋
        self.goal_reached = False
        self.goal_reached_since = None

    # --------------------------------------------------
    # Main Control Loop
    # --------------------------------------------------
    def control_loop(self):
        # Joint 상태 없으면 아무 것도 안 함
        if self.current_q is None or self.current_dq is None:
            return

        # 1) 아직 init pose로 안 갔으면 먼저 init
        if not self.is_initialized:
            self._move_to_init_state()
            return

        # 2) 목표 pose 아직 안 들어왔으면 대기
        if self.target_pose is None:
            self.get_logger().warn(
                "[CTRL] Waiting for /target_pose (target_pose is None)",
                throttle_duration_sec=2.0
            )
            return

        # 3) 이미 목표에 도달한 상태면 hold
        if self.goal_reached:
            self.get_logger().info(
                "[CTRL] Goal reached — holding.",
                throttle_duration_sec=5.0
            )
            return

        # target_pose 모양 체크
        if (not isinstance(self.target_pose, np.ndarray)) or (self.target_pose.shape != (7,)):
            self.get_logger().error(
                f"[CTRL] target_pose invalid. type={type(self.target_pose)}, "
                f"shape={getattr(self.target_pose, 'shape', None)}"
            )
            return

        # =============================
        # 4) 관측 벡터 구성
        # =============================
        obs = np.concatenate([
            self.current_q - self.default_joint_pos,  # 6
            self.current_dq,                          # 6
            self.target_pose,                         # 7
            self.prev_action                          # 6
        ])                                            # 총 25

        obs_tensor = torch.from_numpy(obs).float().unsqueeze(0)

        # =============================
        # 5) PPO Forward
        # =============================
        with torch.no_grad():
            raw_action = self.policy(obs_tensor).cpu().numpy().flatten()

        # 안전하게 -1~1로 클리핑
        raw_action = np.clip(raw_action, -1.0, 1.0)
        self.prev_action = raw_action.copy()

        # =============================
        # Target joint 생성 (현재 관절 + delta 방식)
        # =============================
        # 1) 현재 관절 위치에서 delta 적용
        processed = self.current_q + raw_action * self.action_scale

        # 2) joint limit 안으로 클립
        processed = np.clip(processed, self.joint_lower, self.joint_upper)

        # =============================
        # 7) JointTrajectory 메시지 생성 및 publish
        # =============================
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = processed.tolist()
        point.time_from_start.sec = int(self.traj_duration)
        point.time_from_start.nanosec = int((self.traj_duration % 1.0) * 1e9)
        traj.points.append(point)

        self.traj_pub.publish(traj)

        # =============================
        # 8) 상태 로그 (0.5초에 한 번)
        # =============================
        now = self.get_clock().now().nanoseconds / 1e9
        tracking_error = float(np.linalg.norm(processed - self.current_q))
        action_mean = float(np.mean(np.abs(raw_action)))

        if now >= self.next_ctrl_log_time:
            self.get_logger().info(
                f"[CTRL] ActionMean={action_mean:.3f}  "
                f"Err={tracking_error:.4f}  "
                f"TJ1={processed[0]:.3f}"
            )
            self.next_ctrl_log_time = now + 0.5

        # =============================
        # 9) EE pose 기반 목표 도달 판정
        # =============================
        self._check_goal_with_tf(now)

    # --------------------------------------------------
    # EE pose 기반 goal 체크 (TF 사용)
    # --------------------------------------------------
    def _check_goal_with_tf(self, now_sec: float):
        """
        TF에서 base_frame -> ee_frame 변환을 읽어서
        target_pose와 position / orientation / joint velocity를 비교해
        goal 도달 여부를 판정.
        """
        try:
            # 최신 TF 사용
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                Time(),    # latest
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except TransformException as ex:
            # TF가 아직 안 떴으면 그냥 넘어감
            self.get_logger().warn(
                f"[TF] transform {self.base_frame} -> {self.ee_frame} unavailable: {ex}",
                throttle_duration_sec=2.0
            )
            return

        # EE 현재 pose
        ee_pos = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ], dtype=np.float32)

        ee_ori = np.array([
            trans.transform.rotation.w,
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z
        ], dtype=np.float32)

        # 타겟 pose (이미 numpy array)
        target_pos = self.target_pose[0:3]
        target_ori = self.target_pose[3:7]

        # 쿼터니언 정규화
        ee_ori = ee_ori / (np.linalg.norm(ee_ori) + 1e-8)
        target_ori = target_ori / (np.linalg.norm(target_ori) + 1e-8)

        # error 계산
        pos_error = float(np.linalg.norm(ee_pos - target_pos))
        ori_error = float(1.0 - abs(np.dot(ee_ori, target_ori)))
        vel_norm = float(np.linalg.norm(self.current_dq))

        # 조건 만족?
        now = now_sec
        if (pos_error < self.goal_pos_tol) and (ori_error < self.goal_ori_tol) and (vel_norm < self.goal_vel_tol):
            if self.goal_reached_since is None:
                self.goal_reached_since = now
            elif (now - self.goal_reached_since) >= self.goal_hold_time:
                if not self.goal_reached:
                    self.goal_reached = True
                    self.get_logger().info(
                        f"[GOAL] Reached. pos_err={pos_error:.4f}  "
                        f"ori_err={ori_error:.4f}  vel={vel_norm:.4f}"
                    )
        else:
            self.goal_reached_since = None
            self.goal_reached = False

    # --------------------------------------------------
    # Init state로 이동 (trajectory 방식)
    # --------------------------------------------------
    def _move_to_init_state(self):
        """
        joint_trajectory_controller를 사용해서
        로봇을 default_joint_pos로 20초 동안 천천히 이동시키는 init 단계.
        """
        now_msg = self.get_clock().now()
        now = now_msg.nanoseconds / 1e9

        # 처음 호출되면 시작 시각/자세 기록
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

        elapsed = now - self.init_start_time
        remaining = max(0.0, self.init_duration - elapsed)
        progress = min(1.0, elapsed / self.init_duration) * 100.0

        # 1) 진행률 로그 (0.5초마다)
        if elapsed >= self.next_init_log_time:
            self.get_logger().info(
                f"[Init] Moving to default pose... "
                f"{elapsed:.1f}s elapsed, {remaining:.1f}s remaining "
                f"({progress:.1f}%)"
            )
            self.next_init_log_time = elapsed + 0.5

        # 2) 아직 init trajectory 안 보냈으면 한 번만 생성해서 publish
        if not self.init_traj_sent:
            if self.current_q is None or self.init_start_q is None:
                return

            traj = JointTrajectory()
            traj.joint_names = self.joint_names

            num_points = 40  # 20초 동안 40포인트 → 0.5초 간격
            for i in range(num_points + 1):
                t = self.init_duration * i / num_points  # 0 ~ 20
                alpha = t / self.init_duration           # 0 ~ 1

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

        # 3) init duration + 여유시간 지난 후에는 완료 처리
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
