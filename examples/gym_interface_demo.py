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
# ## Import libraries

# %%
import sys
import os
import time
import re
import math
from pathlib import Path
sys.path.append(str(Path().resolve().parent))
from simworld.config import Config
from simworld.communicator.communicator import Communicator
from simworld.communicator.unrealcv import UnrealCV
from simworld.llm.base_llm import BaseLLM
from simworld.map.map import Map
from simworld.agent.humanoid import Humanoid
from simworld.utils.vector import Vector

# %% [markdown]
# ## Init

# %%
communicator = Communicator(UnrealCV())

# %%
import os 
os.environ['OPENAI_API_KEY'] = '<your_openai_api_key>'


# %% [markdown]
# ## Agent and Environment Classes
#
# **Agent**: Uses LLM to decide actions for navigation
# - Takes observation and target position as input
# - Calculates angle deviation to target (positive = turn left, negative = turn right)
# - Outputs simple commands: "forward `duration`", "rotate `angle` `direction`", or "wait"
# - Observation is a dict with 'position', 'direction', and 'ego_view'
#
# **Environment**: Gym-like interface that manages humanoid and executes actions
# - Defines target position at Vector(1700, -1700)
# - `reset()`: Spawns/resets humanoid at origin, returns initial observation dict
# - `step(action)`: Parses action string and calls humanoid movement functions
# - Returns: `(observation, reward, success)` where:
#   - `observation` = {'position': Vector, 'direction': Vector (direction unit vector), 'ego_view': RGB image}
#   - `reward` = negative distance to target (closer is better)
#   - `success` = Boolean indicating if action was successfully parsed and executed

# %%
class Agent():
    def __init__(self):
        self.llm = BaseLLM("gpt-4o")
        self.system_prompt = """You are an intelligent agent in a 3D simulation world.
Your task is to navigate to the target position by choosing appropriate actions. Do not keep rotating in circles.

You can only output ONE of the following actions:
- "forward <duration>": Move forward for <duration> seconds (e.g., "forward 2")
- "rotate <angle> <direction>": Rotate <angle> degrees in <direction> (left/right) (e.g., "rotate 45 left")
- "wait": Do nothing for 1 second

Output ONLY the action command, nothing else."""

    def action(self, obs, target):
        """Decide next action based on observation and target position.
        
        Args:
            obs: Dictionary containing:
                - 'position' (Vector): Current 2D position
                - 'direction' (Vector): Current direction unit vector
                - 'ego_view' (ndarray): First-person camera view
            target: Target position (Vector) to navigate to
            
        Returns:
            str: Action command
        """
        position = obs['position']
        direction = obs['direction']  # Direction is now a Vector
        ego_view = obs['ego_view']
        
        # Convert direction vector to yaw angle
        current_yaw = math.degrees(math.atan2(direction.y, direction.x))
        
        # Calculate the angle to target
        delta_x = target.x - position.x
        delta_y = target.y - position.y
        target_angle = math.degrees(math.atan2(delta_y, delta_x))
        
        # Calculate angle difference (normalized to [-180, 180])
        angle_diff = target_angle - current_yaw
        # Normalize to [-180, 180]
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        prompt = f"""Current position: {position}
Current direction: {direction} (direction vector, yaw: {current_yaw:.1f}°)
Target position: {target}
Distance to target: {position.distance(target):.1f}
Angle to target: {angle_diff:.1f}° (positive = turn left, negative = turn right)

Choose your next action to move closer to the target."""
        action, time = self.llm.generate_text(system_prompt=self.system_prompt, user_prompt=prompt)

        # print(f"Raw response: {action}")
        return action


# %%
class Environment:
    def __init__(self, communicator, config=Config()):
        self.communicator = communicator
        self.agent = None
        self.agent_name = None
        self.agent_spawned = False  # Track if agent has been spawned
        self.target = None
        self.config = config
        self.map = Map(config)
        self.map.initialize_map_from_file(roads_file=os.path.join('../data/example_city/demo_city_1/roads.json'))

    def reset(self):
        """Reset the humanoid to initial position and orientation.
        
        If agent hasn't been spawned yet, spawn it. Otherwise, reset its position and orientation.
        """
        # Blueprint path for the humanoid agent to spawn in the UE level
        agent_bp = "/Game/TrafficSystem/Pedestrian/Base_User_Agent.Base_User_Agent_C"

        # Initial spawn position and facing direction for the humanoid (2D)
        spawn_location = Vector(0, 0)
        spawn_forward = Vector(1, 0)
        
        if not self.agent_spawned:
            # First time: Create and spawn the agent
            self.agent = Humanoid(
                communicator=self.communicator,
                position=spawn_location,
                direction=spawn_forward,
                config=self.config,
                map=self.map
            )

            # Spawn the humanoid agent in the Unreal world
            self.communicator.spawn_agent(self.agent, name=None, model_path=agent_bp, type="humanoid")
            self.communicator.humanoid_set_speed(self.agent.id, 200)  # Set walking speed

            # Cache the generated UE actor name
            self.agent_name = self.communicator.get_humanoid_name(self.agent.id)
            self.agent_spawned = True
        else:
            # Agent already exists: Reset position and orientation
            # Convert 2D position to 3D (x, y, z)
            location_3d = [
                spawn_location.x,
                spawn_location.y,
                600  # Z coordinate (ground level for humanoid)
            ]
            
            # Convert 2D direction to 3D orientation (pitch, yaw, roll)
            orientation_3d = [
                0,  # Pitch
                math.degrees(math.atan2(spawn_forward.y, spawn_forward.x)),  # Yaw
                0   # Roll
            ]
            
            # Reset position and orientation
            self.communicator.unrealcv.set_location(location_3d, self.agent_name)
            self.communicator.unrealcv.set_orientation(orientation_3d, self.agent_name)
            
            # Update agent's internal state (direction setter expects yaw angle)
            spawn_yaw = math.degrees(math.atan2(spawn_forward.y, spawn_forward.x))
            self.agent.position = spawn_location
            self.agent.direction = spawn_yaw  # Pass yaw angle, not Vector

        # Define a target position the agent is encouraged to move toward
        self.target = Vector(1700, -1700)

        time.sleep(5)  # Wait for everything to spawn

        # Get initial observation (position, direction, and ego-view camera)
        loc_3d = self.communicator.unrealcv.get_location(self.agent_name)  # Returns [x, y, z]
        position = Vector(loc_3d[0], loc_3d[1])
        
        orientation = self.communicator.unrealcv.get_orientation(self.agent_name)  # Returns [pitch, yaw, roll]
        yaw = orientation[1]  # Yaw angle in degrees
        # Convert yaw to direction vector
        direction = Vector(math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
        
        ego_view = self.communicator.get_camera_observation(self.agent.camera_id, "lit")
        
        observation = {
            'position': position,
            'direction': direction,  # Direction vector (normalized)
            'ego_view': ego_view
        }
        return observation

    def step(self, action):
        """Parse and execute the action using humanoid movement functions.
        
        Returns:
            observation: Dict with 'position', 'direction', 'ego_view'
            reward: Negative distance to target
            success: Boolean indicating if action was successfully parsed and executed
        """
        # Parse the action string and execute corresponding humanoid command
        # Remove surrounding quotes if present (LLM sometimes returns with quotes)
        action_cleaned = action.strip().strip('"').strip("'")
        action_lower = action_cleaned.lower().strip()
        success = False
        
        if action_lower.startswith("forward"):
            # Extract duration: "forward 2" -> duration=2
            match = re.search(r'forward\s+(\d+\.?\d*)', action_lower)
            if match:
                duration = float(match.group(1))
                # Set move forward for duration
                self.communicator.humanoid_step_forward(self.agent.id, duration, direction=0)
                success = True
            else:
                print(f"[Warning] Failed to parse forward action: '{action_cleaned}'. Expected format: 'forward <duration>'")
            
        elif action_lower.startswith("rotate"):
            # Extract angle and direction: "rotate 45 left" -> angle=45, direction="left"
            match = re.search(r'rotate\s+(\d+\.?\d*)\s+(left|right)', action_lower)
            if match:
                angle = float(match.group(1))
                direction = match.group(2)
                self.communicator.humanoid_rotate(self.agent.id, angle, direction)
                success = True
            else:
                print(f"[Warning] Failed to parse rotate action: '{action_cleaned}'. Expected format: 'rotate <angle> <left|right>'")
                
        elif action_lower == "wait":
            time.sleep(1)
            success = True
        
        else:
            print(f"[Warning] Unknown action: '{action_cleaned}'. Valid actions: 'forward <duration>', 'rotate <angle> <left|right>', 'wait'")
            time.sleep(0.5)  # Wait a bit even if action is invalid

        # Get current position and direction from UE
        loc_3d = self.communicator.unrealcv.get_location(self.agent_name)  # Returns [x, y, z]
        position = Vector(loc_3d[0], loc_3d[1])
        
        orientation = self.communicator.unrealcv.get_orientation(self.agent_name)  # Returns [pitch, yaw, roll]
        yaw = orientation[1]  # Yaw angle in degrees
        # Convert yaw to direction vector
        direction = Vector(math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))

        # Update agent state (direction setter expects yaw angle, not Vector)
        self.agent.position = position
        self.agent.direction = yaw  # This will automatically convert yaw to direction vector

        # Get ego-view camera observation
        ego_view = self.communicator.get_camera_observation(self.agent.camera_id, "lit")

        # Observation includes position, direction, and camera view
        observation = {
            'position': position,
            'direction': direction,  # Direction vector (normalized)
            'ego_view': ego_view
        }

        # Reward: negative Euclidean distance to target
        reward = -position.distance(self.target)

        return observation, reward, success


# %% [markdown]
# ## Implementation Details
#
# **Observation Format:**
# - Dictionary with three keys:
#   - `'position'`: 2D Vector (x, y) coordinates in the simulation world
#   - `'direction'`: 2D Vector representing the direction the humanoid is facing (normalized unit vector)
#   - `'ego_view'`: RGB image from humanoid's first-person camera
#
# **Action Parsing:**
# - Valid action formats:
#   - `"forward <duration>"` - e.g., "forward 2"
#   - `"rotate <angle> <left|right>"` - e.g., "rotate 45 left"
#   - `"wait"` - pause for 1 second
# - Action cleaning: Surrounding quotes are automatically removed (LLM may return `"rotate 45 left"` with quotes)
# - If action parsing fails:
#   - A warning message is printed
#   - The agent waits 0.5 seconds
#   - `step()` returns `success=False` and step details are not displayed
#
# **Available Humanoid Movement Functions:**
# - `humanoid_set_speed(humanoid_id, speed)`: Set movement speed (e.g., 200)
# - `humanoid_step_forward(humanoid_id, duration, direction)`: Move forward for duration seconds
# - `humanoid_rotate(humanoid_id, angle, direction)`: Rotate by angle degrees ("left" or "right")

# %% [markdown]
# ## Main Loop
#
# The agent navigates from origin (0, 0) to the target position (1700, -1700) by:
# 1. Observing current position, direction (unit vector), and ego-view camera
# 2. Computing angle deviation to target from direction vector (helps agent decide to turn left or right)
# 3. Deciding next action using LLM based on observation, distance, and angle deviation
# 4. Executing action via humanoid movement functions
# 5. Receiving reward based on distance to target (negative distance)

# %%
agent = Agent()
env = Environment(communicator)

obs = env.reset()

print(f"Task: Navigate to target position {env.target}")
print(f"Starting position: {obs['position']}\n")

# Roll out a short trajectory
for i in range(100):
    # Agent decides next action based on current observation and target
    action = agent.action(obs, env.target)
    print(f"Step {i+1}: Action = '{action}'")
    
    # Execute action and get new observation
    obs, reward, success = env.step(action)
    
    # Only print details if action was successfully executed
    if success:
        position = obs['position']
        direction = obs['direction']  # Direction is now a Vector
        
        # Convert direction vector to yaw angle
        current_yaw = math.degrees(math.atan2(direction.y, direction.x))
        
        # Calculate angle to target for display
        delta_x = env.target.x - position.x
        delta_y = env.target.y - position.y
        target_angle = math.degrees(math.atan2(delta_y, delta_x))
        angle_diff = target_angle - current_yaw
        # Normalize to [-180, 180]
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        print(f"  Position: {position}, Direction: {direction} (yaw: {current_yaw:.1f}°)")
        print(f"  Reward: {reward:.2f}, Distance to target: {position.distance(env.target):.1f}")
        print(f"  Angle to target: {angle_diff:+.1f}° ({'turn left' if angle_diff > 0 else 'turn right'})\n")
        communicator.show_img(obs['ego_view'])
        
        # Check if reached target (within 100 units)
        if position.distance(env.target) < 200:
            print(f"Target reached! Final position: {position}, Direction: {direction} (yaw: {current_yaw:.1f}°)")
            break
    else:
        print(f"  [Action failed - skipping step]\n")

# %%
communicator.disconnect()
