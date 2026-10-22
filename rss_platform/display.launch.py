"""Launch robot_state_publisher + joint_state_publisher_gui + rviz2 for sim_simple.urdf.

Run with (workspace install must be sourced so package://rss_meshes/... mesh paths resolve):
    source /opt/ros/jazzy/setup.bash
    source /path/to/your/ros2_ws/install/setup.bash
    ros2 launch ./display.launch.py

Arguments:
    urdf:=<path>          override the URDF file (default: sim_simple.urdf next to this launch file)
    gui:=false            use headless joint_state_publisher instead of the GUI (default: true)
    rviz_config:=<path>   load an RViz config file (default: view.rviz next to this launch file if it exists)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ====================================================================== 

def generate_launch_description():

    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    default_rviz = os.path.join(pkg_dir, 'view.rviz')

    target_urdf = open(os.path.join(pkg_dir, 'sim_target.urdf')).read()
    chaser_urdf = open(os.path.join(pkg_dir, 'sim_chaser.urdf')).read()

    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Launch joint_state_publisher_gui if true, joint_state_publisher if false.',
        ),

        # ── Target robot_state_publisher (namespaced) ──────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='target',
            output='screen',
            parameters=[{
                'robot_description': target_urdf,
                'frame_prefix': 'target/',
            }],
        ),

        # ── Chaser robot_state_publisher (namespaced) ──────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='chaser',
            output='screen',
            parameters=[{
                'robot_description': chaser_urdf,
                'frame_prefix': 'chaser/',
            }],
        ),

        # sim_simple.urdf has two `floating` root joints that robot_state_publisher
        # does not broadcast. Publish the simulator "home position" transforms
        # provided by the URDF author so the TF tree is connected in RViz.
        #
        # Target: base_link -> rotary_optical_plate  (with target/ prefix)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_rotary_optical_plate',
            arguments=[
                '--x', '0', '--y', '0.0', '--z', '1.738',
                '--qx', '0.0', '--qy', '0.0', '--qz', '-0.260', '--qw', '0.966',
                '--frame-id', 'target/base_link',
                '--child-frame-id', 'target/rotary_optical_plate',
            ],
        ),
        # Chaser: base_link -> slide_optical_plate  (with chaser/ prefix)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_slide_optical_plate',
            arguments=[
                '--x', '-5.272', '--y', '0.050', '--z', '1.684',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'chaser/base_link',
                '--child-frame-id', 'chaser/slide_optical_plate',
            ],
        ),

        # Connect target and chaser into a common TF tree via a shared "world" frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_world_to_target',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'world',
                '--child-frame-id', 'target/base_link',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_world_to_chaser',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'world',
                '--child-frame-id', 'chaser/base_link',
            ],
        ),

        # Joint state publisher for the chaser (has revolute joints for the UR arm)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='chaser',
            remappings=[('robot_description', '/chaser/robot_description')],
            condition=IfCondition(gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace='chaser',
            remappings=[('robot_description', '/chaser/robot_description')],
            condition=UnlessCondition(gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=(['-d', default_rviz] if os.path.exists(default_rviz) else []),
        ),
    ])
