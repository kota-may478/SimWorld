# ---
# jupyter:
#   jupytext:
#     cell_metadata_filter: -all
#     formats: ipynb,py:percent
#     notebook_metadata_filter: -all,kernelspec,jupytext
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.19.1
#   kernelspec:
#     display_name: agents
#     language: python
#     name: python3
# ---

# %%
### Traffic Simulation
# This script is used to simulate traffic in the city

# %%
import sys
from pathlib import Path
sys.path.append(str(Path().resolve().parent))
from simworld.traffic.controller.traffic_controller import TrafficController
from simworld.communicator.unrealcv import UnrealCV
from simworld.communicator.communicator import Communicator
from simworld.config import Config

# %%
# Start the game in Unreal Engine first before running the following code.

# %%
config = Config()
traffic_controller = TrafficController(config, 20, 10, "<path to your roads.json>", 1, 0.2)
ucv = UnrealCV()
communicator = Communicator(ucv)
traffic_controller.init_communicator(communicator)

# %%
# Generate world if not generated
# communicator.generate_world('<path to your progen_world.json>', config['citygen.ue_asset_path'], run_time=False)

# %%
traffic_controller.spawn_objects_in_unreal_engine()
communicator.spawn_ue_manager(config['simworld.ue_manager_path'])

# %%
traffic_controller.simulation(traffic_controller.update_states)

# %%
traffic_controller.reset(20, 0, "<path to your roads.json>")

# %%
# Leave the game in Unreal Engine first before disconnecting the connection.
# Otherwise, the game will crash.

# %%
communicator.disconnect()
