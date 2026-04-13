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

# %% [markdown]
# # City Generation
# This script is used to generate the city layout. `generate_city()` is the main function to randomly generate the city layout.
# `export_city()` is the function to export the city layout to the `output` folder.
# Please refer to the documentation for more functions to use.

# %%
import sys
from pathlib import Path
sys.path.append(str(Path().resolve().parent))
from simworld.citygen.function_call.city_function_call import CityFunctionCall
from simworld.config import Config

# %%
config = Config()

# %%
cfc = CityFunctionCall(config, num_segments=10)
cfc.generate_city()
cfc.export_city('<path to your output folder>')
