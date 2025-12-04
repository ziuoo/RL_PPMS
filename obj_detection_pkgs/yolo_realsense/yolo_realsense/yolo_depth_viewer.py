#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import warnings

import cv2
import numpy as np
import torch

from system_interfaces.srv import SetTarget, TargetCenter

warnings.filterwarnings('ignore', category=FutureWarning)


class YOLODepthViewer(Node):
    def __init__(self):
        super().__init__('yolo_depth_viewer')

        self.bridge = CvBridge()
        self.color_frame = None
        self.depth_frame = None
        self.camera_info = None

        # 🎯 ROI 설정
        self.roi_x_min = 415
        self.roi_x_max = 845
        self.roi_y_min = 70
        self.roi_y_max = 500

        # 🎯 내가 관심 있는 클래스들만
        self.allowed_targets = ["tissue", "bottle", "medicine", "sanitizer", "syringe"]

        # 🎯 현재 감지된 객체들 저장 (최신 프레임)
        self.latest_detections = {}
        
        # 🚩 타겟 요청 플래그 (한 번만 전송하도록)
        self.pending_target = None  # None or target name
        self.target_sent = False

        # ✅ SetTarget 서버 (1번 노드로부터 타겟 받기)
        self.target_srv = self.create_service(
            SetTarget,
            'set_target',
            self.set_target_callback
        )
        
        # ✅ TargetCenter 클라이언트 (3번 노드로 중심점 보내기)
        self.center_client = self.create_client(TargetCenter, 'send_target_center')
        
        # self.get_logger().info("🚀 YOLO + Depth Viewer Started")
        # self.get_logger().info("📡 Waiting for 'send_target_center' service...")

        # ---------------- 파라미터 & 모델 로드 ----------------
        self.declare_parameter('model_name', 'yolov5s')
        self.declare_parameter('model_path', '')
        self.declare_parameter('model_type', 'yolov5')

        model_name = self.get_parameter('model_name').value
        model_path = self.get_parameter('model_path').value
        model_type = self.get_parameter('model_type').value

        try:
            if model_path:
                # self.get_logger().info(f"Loading custom model from: {model_path}")
                if model_type == 'yolov8':
                    from ultralytics import YOLO
                    self.model = YOLO(model_path)
                    self.model_type = 'yolov8'
                else:
                    self.model = torch.hub.load(
                        'ultralytics/yolov5', 'custom', path=model_path
                    )
                    self.model_type = 'yolov5'
            else:
                # self.get_logger().info(f"Loading pretrained model: {model_name}")
                if model_type == 'yolov8' or model_name.startswith('yolov8'):
                    from ultralytics import YOLO
                    self.model = YOLO(f'{model_name}.pt')
                    self.model_type = 'yolov8'
                else:
                    self.model = torch.hub.load(
                        'ultralytics/yolov5', model_name, pretrained=True
                    )
                    self.model_type = 'yolov5'

            # self.get_logger().info(
                # f"✅ YOLO Model Loaded: {model_name if not model_path else model_path} "
                # f"(Type: {self.model_type})"
            # )
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

        # ---------------- 카메라 토픽 구독 ----------------
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10
        )
        self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',   # 👈 Depth topic
            self.depth_callback,
            10
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.info_callback,
            10
        )

        # self.get_logger().info("📡 Subscribed to camera topics")
        self.timer = self.create_timer(0.1, self.process)

    # ---------------- 서비스 콜백 ----------------
    def set_target_callback(self, request, response):
        target = request.target

        if target not in self.allowed_targets:
            msg = f"❌ Invalid target '{target}'. Allowed: {self.allowed_targets}"
            response.success = False
            response.message = msg
            return response

        self.get_logger().info(f"🎯 Received target request: {target}")

        # 타겟 설정 및 플래그 리셋
        self.pending_target = target
        self.target_sent = False

        # 즉시 감지된 객체 확인
        if target in self.latest_detections:
            detection = self.latest_detections[target]
            
            # 3번 노드로 중심점 전송
            self.send_target_center(
                target,
                detection['center_x'],
                detection['center_y'],
                detection['distance_m']
            )
            
            # 전송 완료 플래그
            self.target_sent = True
            self.pending_target = None
            
            msg = f"✅ Found {target} at ({detection['center_x']}, {detection['center_y']}) and sent"
            self.get_logger().info(msg)
            response.success = True
            response.message = msg
        else:
            msg = f"⏳ Target '{target}' not detected yet. Will send when detected."
            self.get_logger().info(msg)
            response.success = True
            response.message = msg

        return response

    def send_target_center(self, target, center_x, center_y, distance_m):
        if not self.center_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("⚠️ Service 'send_target_center' not available")
            return

        request = TargetCenter.Request()
        request.target = target
        request.center_x = float(center_x)
        request.center_y = float(center_y)
        request.distance_m = float(distance_m)

        # 비동기 호출
        future = self.center_client.call_async(request)
        future.add_done_callback(
            lambda f: self.handle_center_response(f, target, center_x, center_y, distance_m)
        )

    def handle_center_response(self, future, target, center_x, center_y, distance_m):
        """3번 노드로부터 응답 처리"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"📤 Successfully sent {target} center ({center_x}, {center_y}, {distance_m:.2f}m)"
                )
            else:
                self.get_logger().warn(f"⚠️ Node3 rejected: {response.message}")
        except Exception as e:
            self.get_logger().error(f"❌ Service call failed: {e}")

    # ---------------- 카메라 콜백 ----------------
    def color_callback(self, msg):
        try:
            self.color_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Color conversion failed: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def info_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().info("📐 CameraInfo received")
        self.camera_info = msg

    def is_bbox_in_roi(self, x1, y1, x2, y2):
        """
        바운딩 박스의 중심점이 ROI 내에 있는지 확인
        또는 바운딩 박스가 ROI와 겹치는지 확인
        """
        # 방법 1: 중심점이 ROI 내에 있는지 확인
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        
        # return (self.roi_x_min <= center_x <= self.roi_x_max and 
                # self.roi_y_min <= center_y <= self.roi_y_max)
        
        # 방법 2: 바운딩 박스가 ROI와 겹치는지 확인 (아래 주석 해제)
        # return not (x1 < self.roi_x_min or x2 > self.roi_x_max or 
                    # y2 < self.roi_y_min or y1 > self.roi_y_max)
        
        return (self.roi_x_min <= x1 <= self.roi_x_max and
                    self.roi_y_min <= y1 <= self.roi_y_max and
                    self.roi_x_min <= x2 <= self.roi_x_max and
                    self.roi_y_min <= y2 <= self.roi_y_max)

    # ---------------- 메인 처리 ----------------
    def process(self):
        if self.color_frame is None or self.depth_frame is None:
            return

        try:
            color_img = self.color_frame.copy()
            depth_img = self.depth_frame.copy()

            # ROI 영역 표시
            # cv2.rectangle(
            #     color_img,
            #     (self.roi_x_min, self.roi_y_min),
            #     (self.roi_x_max, self.roi_y_max),
            #     (255, 0, 0),
            #     2
            # )

            # 중앙 픽셀 깊이 정보 (정보용)
            h, w = depth_img.shape[:2]
            cx, cy = w // 2, h // 2
            depth_raw = depth_img[cy, cx]
            center_depth_m = depth_raw * 0.001

            # YOLO inference (verbose=False 추가)
            results = self.model(color_img, verbose=False)
            
            # 최근 감지된 객체 초기화
            self.latest_detections = {}

            # ============ YOLOv8 ============
            if self.model_type == 'yolov8':
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        conf = float(box.conf[0])
                        class_id = int(box.cls[0])
                        class_name = result.names[class_id]

                        # 🎯 내가 관심 있는 물체가 아니면 스킵
                        if class_name not in self.allowed_targets:
                            continue

                        # 🎯 ROI 내에 있는지 확인
                        if not self.is_bbox_in_roi(x1, y1, x2, y2):
                            continue

                        # depth ROI
                        roi = depth_img[y1:y2, x1:x2]
                        if roi.size == 0:
                            continue

                        distance_mm = float(np.median(roi))
                        distance_m = distance_mm * 0.001

                        # 중심 좌표 계산
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)

                        # 최근 감지된 객체로 저장
                        self.latest_detections[class_name] = {
                            'center_x': center_x,
                            'center_y': center_y,
                            'distance_m': distance_m,
                            'confidence': conf
                        }

                        # 1) 바운딩 박스 표시
                        cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = f"{class_name} {conf:.2f} | {distance_m:.2f}m"
                        cv2.putText(
                            color_img,
                            label,
                            (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2
                        )

                        # 2) 중심점 표시 (초록색 원)
                        cv2.circle(color_img, (center_x, center_y), 4, (0, 255, 0), -1)
                        cv2.putText(
                            color_img,
                            f"({center_x},{center_y})",
                            (center_x + 10, center_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.4,
                            (0, 255, 0),
                            1
                        )

            # ROI 정보 표시
            # cv2.putText(
            #     color_img,
            #     f"ROI: x({self.roi_x_min}-{self.roi_x_max}), y({self.roi_y_min}-{self.roi_y_max})",
            #     (10, 90),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     0.5,
            #     (255, 0, 0),
            #     2
            # )
            
            # 감지된 객체 목록 표시
            # detected_list = ', '.join(self.latest_detections.keys()) if self.latest_detections else 'None'
            # cv2.putText(
            #     color_img,
            #     f"Detected: {detected_list}",
            #     (10, 120),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     0.5,
            #     (255, 0, 0),
            #     2
            # )

            cv2.imshow("YOLO", color_img)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YOLODepthViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroy_all_windows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()