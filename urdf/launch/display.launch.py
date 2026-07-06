#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 현재 launch 파일의 위치를 기준으로 상위 urdf 폴더 내의 finger2.urdf 절대 경로 추적
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_folder = os.path.dirname(current_dir)
    urdf_file = os.path.join(urdf_folder, 'finger2.urdf')

    # URDF 파일 내용 읽기
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 로봇 상태 퍼블리셔 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 조인트 상태 퍼블리셔 GUI 노드
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 노드 실행
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])