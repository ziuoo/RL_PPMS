#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        self.init_variable()
        self.init_pubsub()
        
        self.get_logger().info('Joint State Publisher 초기화 완료')
    
    def init_variable(self):
        """변수 초기화"""
        self.gripper_position = 0.0  # 그리퍼 위치 (0.0 ~ 1.0)
    
    def init_pubsub(self):
        """Publisher/Subscriber 초기화"""
        # QoS 설정
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber: 원본 6개 조인트 상태 수신
        self.create_subscription(
            JointState, 
            '/joint_states', 
            self.cbfnc_joint_state, 
            qos
        )
        
        # Publisher: 7개 조인트 상태 (6개 + gripper) 발행
        self.pub_joint_states = self.create_publisher(
            JointState, 
            '/joint_states_with_gripper', 
            qos
        )
        
        self.get_logger().info('Subscriber/Publisher 등록 완료')
    
    def cbfnc_joint_state(self, msg: JointState):
        """JointState 콜백 - 6개 조인트에 gripper 추가하여 재발행"""
        # 7개 조인트 상태 메시지 생성
        joint_state_with_gripper = JointState()
        joint_state_with_gripper.header = msg.header
        
        # 기존 6개 조인트 이름 + gripper_joint
        joint_state_with_gripper.name = list(msg.name) + ['gripper_joint']
        
        # 기존 6개 position + gripper position
        joint_state_with_gripper.position = list(msg.position) + [self.gripper_position]
        
        # 기존 6개 velocity + gripper velocity (0.0)
        if len(msg.velocity) > 0:
            joint_state_with_gripper.velocity = list(msg.velocity) + [0.0]
        else:
            joint_state_with_gripper.velocity = []
        
        # 기존 6개 effort + gripper effort (0.0)
        if len(msg.effort) > 0:
            joint_state_with_gripper.effort = list(msg.effort) + [0.0]
        else:
            joint_state_with_gripper.effort = []
        
        # 재발행
        self.pub_joint_states.publish(joint_state_with_gripper)


def main(args=None):
    rclpy.init(args=args)
    
    joint_state_publisher = JointStatePublisher()
    
    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        joint_state_publisher.get_logger().info('사용자에 의해 종료됨')
    finally:
        joint_state_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
