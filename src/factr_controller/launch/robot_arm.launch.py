from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 패키지의 share 디렉토리 경로 가져오기
    package_share_directory = get_package_share_directory('factr_controller')
    
    # YAML 파라미터 파일 경로
    config_file = os.path.join(
        package_share_directory,
        'initial_joint_positions.yaml'
    )
    
    # robot_arm_node 실행
    robot_arm_node = Node(
        package='factr_controller',
        executable='robot_arm_node',
        name='robot_arm',
        parameters=[config_file],
        output='screen'
    )
    
    return LaunchDescription([
        robot_arm_node
    ])

