#!/usr/bin/env python3
import cv2
import rclpy
import tf2_ros
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Image, CameraInfo
from system_interfaces.srv import TargetCenter, TargetPosition

class PixelToWorld(Node):
    def __init__(self):
        super().__init__('pixel_to_world')
        
        self.init_variables()
        self.init_pubsub_and_srv()
        
        self.timer = self.create_timer(0.1, self.main)
        
#%% Initialization functions
    def init_variables(self):
        self.bridge = CvBridge()
        self.camera_info = None
        self.image_msg = None
        
        self.pixel_t = None
        self.pixel_u = None
        self.pixel_v = None
        self.pixel_d = None
        
        self.calculation_done = True
        
        # Camera World Position (m)
        self.camera_position = np.array([0.53, 0.03, 0.98])
        
        self.theta_x = np.pi
        self.theta_y = 0.0
        self.theta_z = np.pi / 2.0
        
    def init_pubsub_and_srv(self):
        # ---------------- Subscription ----------------
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.cbtp_rgb, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.cbtp_info, 10)
        
        # ---------------- Service ----------------
        self.target_center_srv = self.create_service(TargetCenter, '/e0509/target_center', self.cbsrv_target_center)
        self.target_position_client = self.create_client(TargetPosition, '/e0509/target_position')

        # 서비스 준비될 때까지 계속 대기 (비동기)
        while not self.target_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('TargetPosition 서비스(/e0509/target_position) 대기중...')

#%%Callback functions
    # ---------------- Subscription ----------------
    def cbtp_rgb(self, msg: Image):
        self.image_msg = msg
        
    def cbtp_info(self, msg: CameraInfo):
        if self.camera_info is None:
            self.get_logger().info('camera_info 수신됨')
        self.camera_info = msg

    # ---------------- Service ----------------
    def cbsrv_target_center(self, request, response):
        self.pixel_t = request.target
        self.pixel_u = int(request.center_x)
        self.pixel_v = int(request.center_y)
        self.pixel_d = float(request.distance_m)
        
        self.calculation_done = False
        
        self.get_logger().info(
            f'TargetCenter 수신 → target={request.target}, '
            f'pixel_u={self.pixel_u}, pixel_v={self.pixel_v}, '
            f'distance={request.distance_m:.3f}m'
        )
        
        response.success = True
        response.message = f'Received {request.target} center point'
        return response
    
    def send_world_coord(self, world_coord):
        if not self.target_position_client.service_is_ready():
            self.get_logger().warn('Waiting for TargetPosition service to be available...')
            return
        
        req = TargetPosition.Request()
        req.target_position.x = float(world_coord[0])
        req.target_position.y = float(world_coord[1])
        req.target_position.z = float(world_coord[2])
        self.get_logger().info(
        f'TargetPosition 요청 → x={req.target_position.x:.3f}, '
        f'y={req.target_position.y:.3f}, '
        f'z={req.target_position.z:.3f}'
        )

        future = self.target_position_client.call_async(req)
        future.add_done_callback(self.target_position_response_callback)
    
    def target_position_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f'TargetPosition 성공: {res.message}')
            else:
                self.get_logger().warn(f'TargetPosition 실패: {res.message}')
        except Exception as e:
            self.get_logger().error(f'TargetPosition 응답 처리 중 오류: {e}')
       
#%% Calculation Rotation Matrics 
    def compute_rotation_matrices(self, theta_x, theta_y, theta_z):
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(theta_x), -np.sin(theta_x)],
            [0, np.sin(theta_x),  np.cos(theta_x)]
        ])

        R_y = np.array([
            [np.cos(theta_y), 0, np.sin(theta_y)],
            [0, 1, 0],
            [-np.sin(theta_y), 0, np.cos(theta_y)]
        ])

        R_z = np.array([
            [np.cos(theta_z), -np.sin(theta_z), 0],
            [np.sin(theta_z),  np.cos(theta_z), 0],
            [0, 0, 1]
        ])
        
        R = R_z @ R_y @ R_x

        return R

#%% Main processing function
    def main(self):
        if self.calculation_done:
            return
            
        # 필요 데이터 체크
        if self.camera_info is None or self.image_msg is None:    
            return

        # YOLO에서 아직 픽셀 안 들어왔으면 계산 안 함
        if self.pixel_u is None or self.pixel_v is None or self.pixel_d is None:
            return
        
        target = self.pixel_t
        u = self.pixel_u
        v = self.pixel_v
        d = self.pixel_d
        self.get_logger().info(f'픽셀 좌표 수신: target={target}, (u,v)=({u},{v}), depth={d:.3f}m')


        try:
            rgb_img = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'이미지 CvBridge 변환 실패: {e}')
            self.calculation_done = True
            return

        # 범위 체크
        h, w = rgb_img.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().error(f'픽셀 좌표가 이미지 범위를 벗어남: ({u},{v}), 이미지 크기=({w},{h})')
            self.calculation_done = True
            return

        # depth 값 처리
        if d >= self.camera_position[2]:
            depth_val = self.camera_position[2] * 0.95  # 카메라 높이보다 크면 조금 작게
            self.get_logger().warn(f'⚠️ depth({d:.3f}m)가 카메라 높이({self.camera_position[2]:.3f}m)보다 큼 → {depth_val:.3f}m로 조정')
        else:
            depth_val = d

        # 카메라 내부 파라미터
        K = self.camera_info.k  # 3x3 row-major
        fx = K[0]
        fy = K[4]
        cx = K[2]
        cy = K[5]

        # 픽셀 -> 카메라 좌표 (m)
        x_cam = (u - cx) * depth_val / fx
        y_cam = (v - cy) * depth_val / fy
        z_cam = depth_val

        cam_coord = np.array([x_cam, y_cam, z_cam])

        R = self.compute_rotation_matrices(self.theta_x, self.theta_y, self.theta_z)

        # 카메라 → 월드
        world_coord = R @ cam_coord + self.camera_position

        self.get_logger().info(
            f'월드 좌표: x={world_coord[0]:.4f} y={world_coord[1]:.4f} z={world_coord[2]:.4f}'
        )

        # 계산 완료 플래그 설정 (한 번만 계산)
        self.calculation_done = True

        self.send_world_coord(world_coord)
        
        # ----------- 이미지 위에 점과 텍스트 표시 -----------
        # try:
        #     cv_img = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
        # except Exception as e:
        #     self.get_logger().error(f'이미지 CvBridge 변환 실패: {e}')
        #     return
        
        # # 점 그리기
        # cv_img = rgb_img.copy()
        # cv2.circle(cv_img, (u, v), 6, (0, 0, 255), -1)
        # # 텍스트 준비
        # text = f"({world_coord[0]:.2f}, {world_coord[1]:.2f}, {world_coord[2]:.2f})"
        # # 텍스트 위치 (점 오른쪽 위)
        # text_pos = (u + 10, v - 10)
        # cv2.putText(cv_img, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # # 윈도우에 표시
        # cv2.imshow("Pixel to World", cv_img)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PixelToWorld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()