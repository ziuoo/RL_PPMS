import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, TransformStamped 
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf2_ros
from tf2_ros import TransformBroadcaster
from system_interfaces.srv import MoveToPoint, TargetCenter

class PixelToWorld(Node):
    def __init__(self):
        super().__init__('pixel_to_world')

        # self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('target_frame', 'world')

        # self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.pixel_t = None
        self.pixel_u = None
        self.pixel_v = None
        self.pixel_d = None
        
        # 🚩 계산 완료 플래그 (한 번만 계산하도록)
        self.calculation_done = True  # 처음엔 True (대기 상태)

        self.get_logger().info(
            f'PixelToWorld 시작: target_frame={self.target_frame}'
        )

        self.bridge = CvBridge()
        self.camera_info = None
        # self.depth_msg = None
        self.image_msg = None

        # tf buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 카메라 위치: 월드 원점에서 (52, 0, 71) 떨어짐
        # self.camera_position = np.array([0, 0, 0.71])
        self.camera_position = np.array([0.53, 0.03, 0.98])

        theta_x = np.pi
        theta_y = 0.0
        theta_z = np.pi / 2.0

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

        # 카메라 → 월드 회전 행렬
        self.R_world_cam = R_z @ R_y @ R_x

        # ---- TargetCenter 서비스 서버 생성 (YOLO로부터 중심점 받기) ----
        self.target_center_srv = self.create_service(
            TargetCenter,
            'send_target_center',
            self.target_center_callback
        )
        self.get_logger().info('📡 TargetCenter 서비스 서버 생성 (/send_target_center)')

        # ---- MoveToPoint 서비스 클라이언트 생성 ----
        self.move_client = self.create_client(MoveToPoint, 'move_to_point')
        self.get_logger().info('🛰 MoveToPoint 서비스 클라이언트 생성')

        # 서비스 준비될 때까지 계속 대기 (비동기)
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('⏳ MoveToPoint 서비스(/move_to_point) 대기중...')

        # subscribers
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)
        # self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)
        
        self.timer = self.create_timer(0.1, self.try_compute_world_point)

    # --------- TargetCenter 서비스 콜백 (YOLO로부터 중심점 받기) ---------
    def target_center_callback(self, request, response):
        """
        YOLO 노드에서 TargetCenter 서비스로 전달되는 중심점 데이터 수신
        """
        self.pixel_t = request.target
        self.pixel_u = int(request.center_x)
        self.pixel_v = int(request.center_y)
        self.pixel_d = float(request.distance_m)
        
        # 🚩 새로운 데이터가 들어왔으므로 계산 플래그 리셋
        self.calculation_done = False
        
        self.get_logger().info(
            f'📍 TargetCenter 수신 → target={request.target}, '
            f'pixel_u={self.pixel_u}, pixel_v={self.pixel_v}, '
            f'distance={request.distance_m:.3f}m'
        )
        
        response.success = True
        response.message = f'✅ Received {request.target} center point'
        return response

    # --------- 카메라 콜백 ---------
    def camera_info_cb(self, msg: CameraInfo):
        if self.camera_info is None:
            self.get_logger().info('camera_info 수신됨')
        self.camera_info = msg

    def image_cb(self, msg: Image):
        self.image_msg = msg

    # def depth_cb(self, msg: Image):
        # self.depth_msg = msg

    # --------- 월드 좌표 계산 ---------
    def try_compute_world_point(self):
        # 🚩 이미 계산 완료했으면 리턴
        if self.calculation_done:
            return
            
        # 필요 데이터 체크
        if self.camera_info is None or self.image_msg is None:    
            return

        # ✅ YOLO에서 아직 픽셀 안 들어왔으면 계산 안 함
        if self.pixel_u is None or self.pixel_v is None or self.pixel_d is None:
            return
        
        target = self.pixel_t
        u = self.pixel_u
        v = self.pixel_v
        d = self.pixel_d
        
        self.get_logger().info(f'🔍 계산 시작 → target={target}, u={u}, v={v}, d={d:.3f}m')

        # depth 이미지 -> numpy (z값만 사용)
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'이미지 CvBridge 변환 실패: {e}')
            self.calculation_done = True  # 실패 시에도 플래그 설정
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

        self.get_logger().info(f'📏 사용할 depth 값: {depth_val:.3f}m')

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

        self.get_logger().info(f'📐 카메라 프레임 좌표: x={x_cam:.4f} y={y_cam:.4f} z={z_cam:.4f} (m)')

        cam_coord = np.array([x_cam, y_cam, z_cam])

        # 카메라 → 월드
        world_coord = self.R_world_cam @ cam_coord + self.camera_position

        self.get_logger().info(
            f'🌍 월드 좌표 ({self.target_frame}): x={world_coord[0]:.4f} y={world_coord[1]:.4f} z={world_coord[2]:.4f}'
        )

        # 🚩 계산 완료 플래그 설정 (한 번만 계산)
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
        
    def send_world_coord(self, world_coord):
        """
        계산된 world 좌표를 MoveToPoint 서비스로 전송
        world_coord = [x, y, z]
        """
        if not self.move_client.service_is_ready():
            self.get_logger().warn('MoveToPoint 서비스가 아직 준비 안 됨')
            return
        req = MoveToPoint.Request()
        req.target_position.x = float(world_coord[0])
        req.target_position.y = float(world_coord[1])
        req.target_position.z = float(world_coord[2])
        self.get_logger().info(
        f'🚀 MoveToPoint 요청 → x={req.target_position.x:.3f}, '
        f'y={req.target_position.y:.3f}, '
        f'z={req.target_position.z:.3f}'
        )

        future = self.move_client.call_async(req)
        future.add_done_callback(self.move_response_callback)
    
    def move_response_callback(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f'✅ MoveToPoint 성공: {res.message}')
            else:
                self.get_logger().warn(f'⚠️ MoveToPoint 실패: {res.message}')
        except Exception as e:
            self.get_logger().error(f'❌ MoveToPoint 응답 처리 중 오류: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PixelToWorld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()