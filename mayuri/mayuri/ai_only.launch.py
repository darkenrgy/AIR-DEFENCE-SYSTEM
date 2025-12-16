


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def
    model_arg = DeclareLaunchArgument(
        "model_path",
        default_value="ai_core/models/yolo_defense.onnx",
        description="Path to YOLO/ONNX model file",
    )

    conf_arg = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.45",
        description="Minimum detection confidence",
    )

    target_detector = Node(
        package="mayuri",
        executable="target_detection",
        name="target_detector",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("model_path"),
            "confidence_threshold": LaunchConfiguration("confidence_threshold"),
        }],
    )

    object_classifier = Node(
        package="mayuri",
        executable="classification",
        name="object_classifier",
        output="screen",
    )

    motion_tracker = Node(
        package="mayuri",
        executable="motion_tracking",
        name="motion_tracker",
        output="screen",
    )

    fusion_engine = Node(
        package="mayuri",
        executable="fusion_engine",
        name="fusion_engine",
        output="screen",
    )

    return LaunchDescription([
        LogInfo(msg=" Launching MAYURI AI Core (Detection, Classification, Tracking, Fusion)..."),
        model_arg,
        conf_arg,
        target_detector,
        object_classifier,
        motion_tracker,
        fusion_engine,
    ])