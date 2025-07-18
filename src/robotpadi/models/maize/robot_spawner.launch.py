from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    world = LaunchConfiguration("world")
    file = LaunchConfiguration("file")
    model_string = LaunchConfiguration("model_string")
    topic = LaunchConfiguration("topic")
    entity_name = LaunchConfiguration("entity_name")
    allow_renaming = LaunchConfiguration("allow_renaming")
    x = LaunchConfiguration("x", default="2.8903204995555383")
    y = LaunchConfiguration("y", default="-9.399282181901366")
    z = LaunchConfiguration("z", default="0.6499999999999999")
    roll = LaunchConfiguration("R", default="0.0")
    pitch = LaunchConfiguration("P", default="0.0")
    yaw = LaunchConfiguration("Y", default="1.5949454970799746")

    declare_world_cmd = DeclareLaunchArgument(
        "world", default_value=TextSubstitution(text="virtual_maize_field"), description="World name"
    )

    declare_file_cmd = DeclareLaunchArgument(
        "file", default_value=TextSubstitution(text=""), description="SDF filename"
    )

    declare_model_string_cmd = DeclareLaunchArgument(
        "model_string",
        default_value="",
        description="XML(SDF) string",
    )

    declare_topic_cmd = DeclareLaunchArgument(
        "topic",
        default_value=TextSubstitution(text=""),
        description="Get XML from this topic",
    )

    declare_entity_name_cmd = DeclareLaunchArgument(
        "entity_name",
        default_value=TextSubstitution(text=""),
        description="Name of the entity",
    )

    declare_allow_renaming_cmd = DeclareLaunchArgument(
        "allow_renaming",
        default_value="False",
        description="Whether the entity allows renaming or not",
    )

    spawn_model_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_spawn_model.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("world", world),
            ("file", file),
            ("model_string", model_string),
            ("topic", topic),
            ("entity_name", entity_name),
            ("allow_renaming", allow_renaming),
            ("x", x),
            ("y", y),
            ("z", z),
            ("R", roll),
            ("P", pitch),
            ("Y", yaw),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_file_cmd)
    ld.add_action(declare_model_string_cmd)
    ld.add_action(declare_topic_cmd)
    ld.add_action(declare_entity_name_cmd)
    ld.add_action(declare_allow_renaming_cmd)
    ld.add_action(spawn_model_description)

    return ld
