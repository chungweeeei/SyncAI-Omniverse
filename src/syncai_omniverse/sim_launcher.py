"""Helpers to set up the Isaac Sim World, load scenes, and spawn the Jetbot."""

import numpy as np
import omni.usd

from isaacsim.core.api import World
from isaacsim.robot.wheeled_robots import WheeledRobot
from isaacsim.storage.native import get_assets_root_path


def create_world(physics_dt: float = 1.0 / 60.0,
                 rendering_dt: float = 1.0 / 60.0) -> World:
    world = World(physics_dt=physics_dt, rendering_dt=rendering_dt)
    return world


def load_warehouse_scene(scene_path: str):
    omni.usd.get_context().open_stage(scene_path)


def spawn_jetbot(world: World,
                 position: list[float] | None = None) -> WheeledRobot:
    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError(
            "Cannot resolve Isaac Sim assets root path. "
            "Make sure Nucleus is configured or the asset cache is available."
        )

    jetbot_usd = assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"

    pos = np.array(position if position else [0.0, 0.0, 0.0])

    jetbot = world.scene.add(
        WheeledRobot(
            prim_path="/World/Jetbot",
            name="jetbot",
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
            create_robot=True,
            usd_path=jetbot_usd,
            position=pos,
        )
    )
    return jetbot


def spawn_turtlebot(world: World,
                    position: list[float] | None = None) -> WheeledRobot:
    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError(
            "Cannot resolve Isaac Sim assets root path. "
            "Make sure Nucleus is configured or the asset cache is available."
        )

    turtlebot_usd = assets_root + "/Isaac/Robots/Turtlebot/Turtlebot3/turtlebot3_burger.usd"

    pos = np.array(position if position else [0.0, 0.0, 0.05])

    turtlebot = world.scene.add(
        WheeledRobot(
            prim_path="/World/Turtlebot",
            name="turtlebot",
            wheel_dof_names=["wheel_left_joint", "wheel_right_joint"],
            create_robot=True,
            usd_path=turtlebot_usd,
            position=pos,
        )
    )
    return turtlebot
