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
#     display_name: simworld
#     language: python
#     name: python3
# ---

# %%
### World Generation in Unreal Engine
# This script is used to generate the world in Unreal Engine. 
# Make sure you have already generated the city layout.

# %%
import sys
from pathlib import Path
sys.path.append(str(Path().resolve().parent))
from simworld.config import Config
from simworld.communicator.communicator import Communicator
from simworld.communicator.unrealcv import UnrealCV

# %%
config = Config()

# %%
# Start the game in Unreal Engine first before running the following code.

# %%
communicator = Communicator(UnrealCV())

# %%
communicator.generate_world('<path to your progen_world.json>', config['citygen.ue_asset_path'], run_time=False)

# %%
communicator.clear_env(keep_roads=False)

# %%
# Leave the game in Unreal Engine first before disconnecting the connection.
# Otherwise, the game will crash.

# %%
communicator.disconnect()
