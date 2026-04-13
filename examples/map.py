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
### Map Test Notebook
# This notebook is used to test the map and waypoint system

# %%
import sys
from pathlib import Path
sys.path.append(str(Path().resolve().parent))
from simworld.config import Config
from simworld.map.map import Map

# %%
config = Config()

# %%
map = Map(config)
map.initialize_map_from_file(fine_grained=False)

# %%
map.visualize_by_type()

# %%
start = map.get_random_node()
end = map.get_random_node(exclude=[start])
path = map.get_shortest_path(start, end)

# %%
map.visualize_path(path)
