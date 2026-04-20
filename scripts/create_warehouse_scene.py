#!/usr/bin/env python3
"""Generate the warehouse USD scene (headless)."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

# --- imports after SimulationApp ---
import os
import sys
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from syncai_omniverse.warehouse_builder import WarehouseBuilder

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CONFIG_PATH = os.path.join(ROOT_DIR, "config", "sim_config.yaml")

with open(CONFIG_PATH) as f:
    config = yaml.safe_load(f)

output_path = os.path.join(ROOT_DIR, config["output"]["scene_path"])

builder = WarehouseBuilder(config["warehouse"], output_path)
saved = builder.build()
print(f"Warehouse scene saved to: {saved}")

simulation_app.close()
