#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
키워드 서비스 서버 (테스트용)
Control Tower에서 전송한 키워드를 받아 처리하는 서비스 서버
SetTarget.srv 사용
"""

import rclpy
from rclpy.node import Node

# SetTarget 서비스 import
try:
    from yolo_depth_interfaces.srv import SetTarget
except ImportError:
    SetTarget = None
    print("ERROR: SetTarget 서비스를 찾을 수 없습니다")
    print("yolo_depth_interfaces 패키지를 확인하세요")


class KeywordServiceServer(Node):
    """키워드 처리 서비스 서버"""
    
    def __init__(self):
        super().__init__('keyword_service_server')
        
        if SetTarget is None:
            self.get_logger().error('SetTarget 서비스를 import할 수 없습니다')
            return
        
        # 서비스 서버 생성
        self.service = self.create_service(
            SetTarget,
            '/set_target',
            self.keyword_callback
        )
        
        self.get_logger().info('🎯 키워드 서비스 서버 시작')
        self.get_logger().info('서비스 이름: /set_target')
        self.get_logger().info('서비스 타입: SetTarget')
        self.get_logger().info('대기 중...')
        self.get_logger().info('')
        
    def keyword_callback(self, request, response):
        """
        서비스 콜백 함수
        
        Args:
            request: SetTarget.Request
                - target (string): 키워드 문자열
            response: SetTarget.Response
                - success (bool): 성공 여부
                - message (string): 응답 메시지
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎯 키워드 서비스 요청 수신!')
        self.get_logger().info(f'키워드: "{request.target}"')
        
        # 실제 키워드 처리 로직을 여기에 구현
        # 예: 로봇 제어, 데이터베이스 저장, 다른 노드로 전달 등
        
        try:
            keyword = request.target
            
            # 키워드 검증
            if not keyword or keyword.strip() == '':
                response.success = False
                response.message = '빈 키워드입니다'
                self.get_logger().warn('⚠️ 빈 키워드 수신')
            else:
                # 성공 응답
                response.success = True
                response.message = f'키워드 "{keyword}" 수신 및 처리 완료'
                
                self.get_logger().info('✅ 처리 성공!')
                self.get_logger().info(f'다음 동작: {keyword} 객체 Pick & Place')
            
        except Exception as e:
            # 실패 응답
            response.success = False
            response.message = f'처리 실패: {str(e)}'
            
            self.get_logger().error('❌ 처리 실패!')
            self.get_logger().error(f'오류: {str(e)}')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    server = KeywordServiceServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.get_logger().info('키워드 서비스 서버 종료')
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
