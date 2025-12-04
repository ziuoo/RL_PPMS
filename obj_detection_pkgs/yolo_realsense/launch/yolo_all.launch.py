#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """RealSense 카메라 + YOLO Depth Viewer 통합 실행"""
    
    # 패키지 경로
    pkg_share = get_package_share_directory('yolo_realsense')
    default_config = os.path.join(pkg_share, 'config', 'yolo_params.yaml')
    
    # RealSense 패키지 경로
    realsense_pkg = get_package_share_directory('realsense2_camera')
    realsense_launch = os.path.join(realsense_pkg, 'launch', 'rs_launch.py')
    
    # 커스텀 모델 경로
    custom_model_path = os.path.join(pkg_share, 'model', 'best_2.pt')
    
    # Launch Arguments
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='yolov8s',
        description='YOLO 모델 이름 (yolov5s, yolov8n, etc.)'
    )
    
    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='yolov8',
        description='YOLO 모델 타입 (yolov5 or yolov8)'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=custom_model_path,
        description='커스텀 YOLO 모델 경로 (.pt 파일)'
    )
    
    # RealSense 카메라 실행
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
        }.items()
    )
    
    # YOLO Depth Viewer 실행
    yolo_viewer_node = Node(
        package='yolo_realsense',
        executable='yolo_depth_viewer',
        name='yolo_depth_viewer_node',
        output='screen',
        parameters=[{
            'model_name': LaunchConfiguration('model_name'),
            'model_type': LaunchConfiguration('model_type'),
            'model_path': LaunchConfiguration('model_path'),
        }]
    )
    
    # Pixel to World 변환 노드 실행
    pixel_to_world_node = Node(
        package='coordinate_transformation',
        executable='pixel_to_world',
        name='pixel_to_world_node',
        output='screen'
    )
    
    return LaunchDescription([
        model_name_arg,
        model_type_arg,
        model_path_arg,
        realsense_node,
        yolo_viewer_node,
        # pixel_to_world_node,
    ])
