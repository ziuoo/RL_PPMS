#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from system_interfaces.srv import MoveToPoint


class MoveToPointServer(Node):
    def __init__(self):
        super().__init__('move_to_point_server')

        # 서비스 서버 생성: 이름은 PixelToWorld에서 쓰는 'move_to_point'와 동일해야 함
        self.srv = self.create_service(
            MoveToPoint,
            'move_to_point',
            self.handle_move_to_point
        )

        self.get_logger().info('🛰 MoveToPoint 서비스 서버 준비 완료 (/move_to_point)')

    def handle_move_to_point(self, request: MoveToPoint.Request, response: MoveToPoint.Response):
        """
        PixelToWorld 노드에서 보낸 target_position을 받아서
        값 출력 후, 성공 응답을 돌려주는 콜백
        """
        target: Point = request.target_position

        self.get_logger().info(
            f'📥 MoveToPoint 요청 수신:\n'
            f'    x = {target.x:.3f}\n'
            f'    y = {target.y:.3f}\n'
            f'    z = {target.z:.3f}'
        )

        # 여기서 실제 로봇 이동 코드를 넣으면 됨 (MoveIt, dsr, etc.)
        # 지금은 그냥 성공했다고만 응답
        response.success = True
        response.message = (
            f"목표 지점 수신 완료: "
            f"x={target.x:.3f}, y={target.y:.3f}, z={target.z:.3f}"
        )

        self.get_logger().info(f'✅ 응답 전송: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPointServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
