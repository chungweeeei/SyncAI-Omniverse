#!/usr/bin/env python3
"""Launch the warehouse simulation with a keyboard-controlled TurtleBot."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# --- imports after SimulationApp ---
import os
import sys
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from syncai_omniverse.sim_launcher import create_world, load_warehouse_scene, spawn_turtlebot
from syncai_omniverse.jetbot_controller import KeyboardJetbotController
from syncai_omniverse.warehouse_builder import WarehouseBuilder

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CONFIG_PATH = os.path.join(ROOT_DIR, "config", "sim_config.yaml")

with open(CONFIG_PATH) as f:
    config = yaml.safe_load(f)

# -- Ensure warehouse scene exists --
scene_path = os.path.abspath(os.path.join(ROOT_DIR, config["output"]["scene_path"]))
if not os.path.exists(scene_path):
    print("Warehouse scene not found — generating...")
    builder = WarehouseBuilder(config["warehouse"], scene_path)
    builder.build()
    print(f"Scene saved to: {scene_path}")

# -- Create world & load scene --
sim_cfg = config["simulation"]
world = create_world(
    physics_dt=sim_cfg["physics_dt"],
    rendering_dt=sim_cfg["rendering_dt"],
)
load_warehouse_scene(scene_path)

# -- Spawn TurtleBot --
robot_cfg = config["robot"]
turtlebot = spawn_turtlebot(world, robot_cfg["spawn_position"])

# -- Keyboard controller --
controller = KeyboardJetbotController(
    wheel_radius=robot_cfg["wheel_radius"],
    wheel_base=robot_cfg["wheel_base"],
    max_linear_speed=robot_cfg["max_linear_speed"],
    max_angular_speed=robot_cfg["max_angular_speed"],
)

# -- Reset & run --
world.reset()
turtlebot.initialize()
controller.setup_keyboard()

print("Simulation running. Use W/A/S/D or arrow keys to control TurtleBot.")
print("Close the window to exit.")

while simulation_app.is_running():
    world.step(render=True)
    if world.is_playing():
        if world.current_time_step_index == 0:
            world.reset()
            controller.setup_keyboard()
        action = controller.get_action()
        turtlebot.apply_wheel_actions(action)

controller.cleanup()
simulation_app.close()
