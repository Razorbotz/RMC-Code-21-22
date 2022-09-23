from launch import LaunchDescription
from launch_ros.actions import Node

## @file
# Launch file that contains all the nodes necessary
# to run the robot

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='logic',
            name='logic',
            executable='logic_node',
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='communication2',
            name='communication2',
            executable='communication2_node',
            parameters=[
                {"robot_name": "Spinner"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='power_distribution_panel',
            name='power_distribution_panel',
            executable='power_distribution_panel_node',
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='talon',
            name='shoulder',
            executable='talon_node',
            parameters=[
                {"motor_number": 14},
                {"diagnostics_port": 56715},
                {"invert_motor": True},
                {"speed_topic": "shoulder_speed"},
                {"info_topic": "talon_14_info"},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='talon',
            name='dump',
            executable='talon_node',
            parameters=[
                {"motor_number": 15},
                {"diagnostics_port": 56714},
                {"invert_motor": True},
                {"speed_topic": "dump_bin_speed"},
                {"info_topic": "talon_15_info"},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='excavation',
            name='excavation',
            executable='excavation_node'
        )
        ,
        Node(
            package='falcon',
            name='right_motors',
            executable='falcon_node',
            parameters=[
                {"motor_number": 10},
                {"motor_number2": 11},
                {"diagnostics_port": 72340},
                {"invert_motor": True},
                {"speed_topic": "drive_right_speed"},
                {"info_topic": "talon_10_info"},
                {"info_topic2": "talon_11_info"},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
            ]
        )
	,
        Node(
            package='falcon',
            name='left_motors',
            executable='falcon_node',
            parameters=[
                {"motor_number": 12},
                {"motor_number2": 13},
                {"diagnostics_port": 72342},
                {"invert_motor": False},
                {"speed_topic": "drive_left_speed"},
                {"info_topic": "talon_13_info"},
                {"info_topic2": "talon_12_info"},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
            ]
        )
        ,
        Node(
            package='servo',
            name='servo',
            executable='servo_node'
        )
#        ,
#        Node(
#            package='zed',
#            name='zed',
#            executable='zed_node'
#        )
    ]
)
