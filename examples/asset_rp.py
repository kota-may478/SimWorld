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
### Asset RP Test
# This script is used to test the asset retrieval and placement. You can use natural language to retrieve and place assets.
# This script will only generate json file, you need to use `world_generation.ipynb` to generate the world.

# %%
# You can skip this if you already have the generated city
import sys
from pathlib import Path
sys.path.append(str(Path().resolve().parent))
from simworld.citygen.function_call.city_function_call import CityFunctionCall
from simworld.config.config_loader import Config
config = Config()
cfg = CityFunctionCall(config)

# %%
# You can skip this if you already have the generated city
cfg.generate_city()
cfg.export_city()

# %%
import sys
from pathlib import Path
sys.path.append(str(Path().resolve().parent))
from simworld.assets_rp.AssetsRP import AssetsRetrieverPlacer
from simworld.config.config_loader import Config
config = Config()
arp = AssetsRetrieverPlacer(config, input_dir='<path to the folder containing the city layout>') # you can use data/example_city/map*

# %%
import os
os.environ['OPENAI_API_KEY'] = '<your_api_key>'

# %%
from simworld.citygen.dataclass.dataclass import Point
arp.city_generator.route_generator.get_point_around_label(Point(-50, -50), arp.city_generator.city_quadtrees)

# %%
arp.generate_assets_manually("Please place a streetlight besides a high beige stone office building, with arched and rectangular upper windows.")
