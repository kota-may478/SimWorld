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
#     display_name: unreal
#     language: python
#     name: python3
# ---

# %%
### This script is used to test local action planner module.

# %%
import sys
import os
from pathlib import Path
sys.path.append(str(Path().resolve().parent))
from simworld.communicator.unrealcv import UnrealCV
from simworld.communicator.communicator import Communicator
from simworld.llm.a2a_llm import A2ALLM
from simworld.agent.humanoid import Humanoid
from simworld.utils.vector import Vector
from simworld.map.map import Map
from simworld.config.config_loader import Config
from simworld.local_planner.local_planner import LocalPlanner

# %%
os.environ['OPENAI_API_KEY'] = '<your_openai_api_key>'

# %%
config = Config()

# %%
# Initialize the communicator
communicator = Communicator(UnrealCV())

# %%
# Initialize the LLM model
llm = A2ALLM(model_name='gpt-4o-mini', provider='openai')

# %%
# Initialize the map
map = Map(config)
map.initialize_map_from_file(roads_file='<path to your roads.json>')

# %%
# Initialize the humanoid agent
humanoid = Humanoid(position=Vector(1700, 1700), direction=Vector(1, 0), map=map, communicator=communicator, config=config)

# %%
# Initialize the local planner
local_planner = LocalPlanner(agent=humanoid, model=llm, rule_based=True)

# %%
communicator.spawn_object('GEN_BP_Box_1_C', '/Game/InteractableAsset/Box/BP_Interactable_Box.BP_Interactable_Box_C', (1700, -1700, 20), (0, 0, 0))

# %%
### Test parser
plan = 'Go to (1700, -1700) and pick up GEN_BP_Box_1_C.'
action_space = local_planner.parse(plan)

# %%
communicator.spawn_agent(humanoid, config['user.model_path'])

# %%
communicator.spawn_ue_manager(config['simworld.ue_manager_path'])

# %%
# Update humanoid position and direction in a separate thread
import threading
import time

def update(exit_event):
    while not exit_event.is_set():
        humanoid_ids = [humanoid.id]
        result = communicator.get_position_and_direction(humanoid_ids=humanoid_ids)
        for idx in humanoid_ids:
            pos, dir = result[('humanoid', idx)]
            humanoid.position = pos
            humanoid.direction = dir
        time.sleep(0.1)

exit_event = threading.Event()
t = threading.Thread(target=update, args=(exit_event,))
t.start()

# %%
### Test executor
local_planner.execute(action_space)

# %%
communicator.humanoid_pick_up_object(humanoid.id, 'GEN_BP_Box_1_C')

# %%
exit_event.set()
t.join()

# %%
communicator.clear_env()
