from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def terminal_command(title, command):
    return [
        "gnome-terminal",
        "--title",
        title,
        "--",
        "bash",
        "-lc",
        command,
    ]


def launch_terminals(context):
    px4_dir = LaunchConfiguration("px4_dir").perform(context)
    workspace_dir = LaunchConfiguration("workspace_dir").perform(context)
    ros_distro = LaunchConfiguration("ros_distro").perform(context)
    trajectory_launch_args = LaunchConfiguration("trajectory_launch_args").perform(context)

    px4_cmd = f"cd {px4_dir} && make px4_sitl gz_x500; exec bash"
    agent_cmd = "MicroXRCEAgent udp4 -p 8888; exec bash"
    controller_cmd = (
        f"source /opt/ros/{ros_distro}/setup.bash && "
        f"source {workspace_dir}/install/setup.bash && "
        f"ros2 launch geometric_controller geometric_controller.launch.xml {trajectory_launch_args}; "
        "exec bash"
    )

    return [
        ExecuteProcess(cmd=terminal_command("PX4 SITL", px4_cmd), output="screen"),
        ExecuteProcess(cmd=terminal_command("MicroXRCEAgent", agent_cmd), output="screen"),
        ExecuteProcess(cmd=terminal_command("geometric_controller", controller_cmd), output="screen"),
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("px4_dir", default_value="/home/parallels/PX4-Autopilot"),
            DeclareLaunchArgument("workspace_dir", default_value="/home/parallels/ws_sensor_combined"),
            DeclareLaunchArgument("ros_distro", default_value="jazzy"),
            DeclareLaunchArgument("trajectory_launch_args", default_value=""),
            OpaqueFunction(function=launch_terminals),
        ]
    )
