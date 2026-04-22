"""Publish Isaac Sim's simulation clock to /clock so ROS2 consumers running
with use_sim_time=true can align their timers, TF buffer, and message filters
to sim time rather than wall time.

Without this, rviz2/nav2 using the default `use_sim_time=false` compare TF
msgs stamped with sim time (starting at 0) against the current wall clock
(~1.7e9) and drop everything with "TF_OLD_DATA ignoring data from past".

Requires Isaac Sim runtime: `omni.graph.core` and the `isaacsim.ros2.bridge`
extension must be enabled before calling `attach_clock_publisher`.
"""


def attach_clock_publisher(
    stage,
    topic: str = "/clock",
    graph_path: str = "/ClockActionGraph",
) -> str:
    """
        Build (or overwrite) an OmniGraph that publishes the current sim time
        to `topic` (rosgraph_msgs/Clock) every playback tick.

        `/clock` is intentionally NOT namespaced -- it is a ROS2-wide singleton
        that all nodes running with use_sim_time=true read from.
    """
    import omni.graph.core as og

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", topic),
            ],
        },
    )
    return graph_path
