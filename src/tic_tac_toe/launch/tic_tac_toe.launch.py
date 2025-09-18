from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    d435_node = Node(
            package='realsense2_camera',
            namespace='',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[
                {'enable_color': True},
                {'rgb_camera.color_profile':'640x480x60'},
                {'rgb_camera.enable_auto_exposure':False},
                {'rgb_camera.exposure':700},
                {'enable_depth': False},
                {'enable_infra1': False},
                {'enable_infra2': False},
                {'enable_gyro': False},
                {'enable_accel': False},
                {'enable_pointcloud': False},
                {'enable_sync': False},
                {'align_depth': False}
            ]
        )
    
    get_board_state_node = Node(
            package='tic_tac_toe',
            namespace='',
            executable='get_board_state',
            name='get_board_state'
        )
    
    compute_best_move_node = Node(
            package='tic_tac_toe',
            namespace='',
            executable='compute_best_move',
            name='compute_best_move'
        )
    
    piper_node = Node(
        package='piper',
        executable='piper_single_ctrl',
        name='piper_ctrl_single_node',
        output='screen',
        parameters=[{
            'can_port': 'can0',
            'auto_enable': True,
            'gripper_val_mutiple': 1,
            'gripper_exist': False,
        }],
        remappings=[
            ('joint_ctrl_single', '/joint_states'),
        ]
    )

    tic_tac_toe_pkg_share = get_package_share_directory('tic_tac_toe')
    rviz_config_path = os.path.join(tic_tac_toe_pkg_share, 'config', 'tic_tac_toe.rviz')
    rviz_arg = DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config_path,
        )
    
    moveit_config = MoveItConfigsBuilder("piper", package_name="piper_no_gripper_moveit").to_moveit_configs()

    arm_controller_node = Node(
            package='tic_tac_toe',
            namespace='',
            executable='arm_control',
            name='arm_control'
        )
    
    web_controller_node = Node(
            package='tic_tac_toe',
            namespace='',
            executable='web_control',
            name='web_control'
        )

    return LaunchDescription([
        d435_node,
        get_board_state_node,
        compute_best_move_node,
        piper_node,
        rviz_arg,
        generate_demo_launch(moveit_config),
        arm_controller_node,
        web_controller_node
    ])