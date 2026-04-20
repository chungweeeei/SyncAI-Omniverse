"""Attach an OmniGraph that publishes wheel encoder (/joint_states),
odometry (/odom), and the odom -> base_link transform on /tf.

Requires Isaac Sim runtime: `omni.graph.core` and the `isaacsim.ros2.bridge`
extension must be enabled before calling `attach_odom_publisher`.
Do NOT import this module before `SimulationApp` is instantiated.
"""
from pxr import Sdf


def attach_odom_publisher(
    stage,
    robot_path: str = "/World/SlotCar",
    chassis_link: str = "base_link",
    odom_frame: str = "odom",
    odom_topic: str = "/odom",
    tf_topic: str = "/tf",
    joint_state_topic: str = "/joint_states",
    graph_path: str = "/OdomActionGraph",
    publish_joint_states: bool = True,
) -> str:
    """
        Build (or overwrite) an OmniGraph that publishes:
          * /odom                (nav_msgs/Odometry)            — from IsaacComputeOdometry
          * /tf: odom -> <chassis_link>                         — from the same odometry output
          * /joint_states        (sensor_msgs/JointState)       — all articulation joints,
                                                                   i.e. both drive wheels
                                                                   (= simulated encoders)

        `chassis_link` is the rigid body IsaacComputeOdometry tracks; it is also
        the child frame id reported on /odom and the odom TF.

        Returns the graph prim path.
    """
    import omni.graph.core as og

    robot_prim = stage.GetPrimAtPath(robot_path)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Robot prim not found: {robot_path}")

    chassis_path = f"{robot_path}/{chassis_link}"
    chassis_prim = stage.GetPrimAtPath(chassis_path)
    if not chassis_prim.IsValid():
        raise RuntimeError(f"Chassis link not found: {chassis_path}")

    create_nodes = [
        ("OnTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
    ]
    connect = [
        ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),
        ("OnTick.outputs:tick", "PublishOdom.inputs:execIn"),
        ("OnTick.outputs:tick", "PublishOdomTF.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
        ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
        ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
        ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
        ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
        ("ComputeOdom.outputs:position", "PublishOdomTF.inputs:translation"),
        ("ComputeOdom.outputs:orientation", "PublishOdomTF.inputs:rotation"),
    ]
    set_values = [
        ("ComputeOdom.inputs:chassisPrim", [Sdf.Path(chassis_path)]),
        ("PublishOdom.inputs:topicName", odom_topic),
        ("PublishOdom.inputs:odomFrameId", odom_frame),
        ("PublishOdom.inputs:chassisFrameId", chassis_link),
        ("PublishOdomTF.inputs:topicName", tf_topic),
        ("PublishOdomTF.inputs:parentFrameId", odom_frame),
        ("PublishOdomTF.inputs:childFrameId", chassis_link),
    ]

    if publish_joint_states:
        create_nodes.append(
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState")
        )
        connect.extend([
            ("OnTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
        ])
        set_values.extend([
            ("PublishJointState.inputs:topicName", joint_state_topic),
            ("PublishJointState.inputs:targetPrim", [Sdf.Path(robot_path)]),
        ])

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.CONNECT: connect,
            og.Controller.Keys.SET_VALUES: set_values,
        },
    )
    return graph_path
