from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, Command

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():


    declared_Arguments = [
        DeclareLaunchArgument("real", default_value="false", description="real robot OR sim")
    ]
    real = LaunchConfiguration("real")


    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit")
        .robot_description(mappings={"real" : real})
        .to_moveit_configs()
    )



    planning_scene_node = Node(
        package="piper_with_gripper_moveit",
        executable="planning_scene_node",
        name="planning_scene_node",
        output="screen"
    )

    # # generate_demo_launch는 LaunchDescription을 반환하므로,
    # # 여기에 planning_scene_node를 추가해서 LaunchDescription에 포함시켜야 함.
    # demo_launch = generate_demo_launch(moveit_config)

    # # LaunchDescription은 add_action()으로 노드를 추가할 수 있음
    # for arg in declared_Arguments:
    #     demo_launch.add_action(arg)
    # demo_launch.add_action(planning_scene_node)

    # # demo_launch.add_action(DeclareLaunchArgument("real", default_value="false", description="real_hw"))

    # return demo_launch

    ld = LaunchDescription()

    for arg in declared_Arguments:
        ld.add_action(arg)

    demo_launch = generate_demo_launch(moveit_config)
    for action in demo_launch.entities:
        ld.add_action(action)

    ld.add_action(planning_scene_node)

    return ld
