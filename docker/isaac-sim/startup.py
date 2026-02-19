"""
Isaac Sim standalone script: warehouse scene with TurtleBot3 Burger.
Runs headless, enables ROS2 bridge for camera, lidar, odom, and cmd_vel.
"""

import sys
import argparse

# -- Isaac Sim startup (must happen before other omni imports) --
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

# -- Now safe to import omni/isaac modules --
import omni.kit.commands
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path

# Enable ROS2 bridge extension
import omni.kit.app
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)

# Wait a few frames for extension to initialize
for _ in range(10):
    simulation_app.update()

# Get NVIDIA assets root path (Nucleus server or local cache)
assets_root = get_assets_root_path()
if assets_root is None:
    print("ERROR: Could not find assets root path. Check Nucleus connection.", file=sys.stderr)
    simulation_app.close()
    sys.exit(1)

# Create the simulation world
world = World(stage_units_in_meters=1.0)

# Load warehouse environment
warehouse_usd = assets_root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
add_reference_to_stage(usd_path=warehouse_usd, prim_path="/World/Warehouse")

# Load TurtleBot3 Burger
turtlebot_usd = assets_root + "/Isaac/Robots/Turtlebot/turtlebot3_burger.usd"
add_reference_to_stage(usd_path=turtlebot_usd, prim_path="/World/TurtleBot3")

# Position the robot at warehouse entrance area
from omni.isaac.core.utils.prims import set_prim_attribute_value
from pxr import Gf
import omni.usd
stage = omni.usd.get_context().get_stage()
turtlebot_prim = stage.GetPrimAtPath("/World/TurtleBot3")
if turtlebot_prim.IsValid():
    from omni.isaac.core.utils.transformations import set_prim_transform
    # Place robot at x=0, y=0, z=0 (ground level at warehouse entrance)
    omni.kit.commands.execute(
        "TransformPrimCommand",
        path="/World/TurtleBot3",
        new_translation=Gf.Vec3d(0.0, 0.0, 0.0),
    )

# -- Configure ROS2 components via Action Graphs --
# Isaac Sim uses OmniGraph Action Graphs to wire sensors to ROS2 topics.
# We use the og (OmniGraph) API to create the graph programmatically.

import omni.graph.core as og

# Create the ROS2 action graph
(ros2_graph, _, _, _) = og.Controller.edit(
    {"graph_path": "/World/ROS2Graph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            # Differential drive controller (subscribes to /cmd_vel, drives wheels)
            ("DifferentialController", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
            ("DiffDriveController", "omni.isaac.wheeled_robots.DifferentialController"),
            # Odometry publisher
            ("ComputeOdometry", "omni.isaac.core_nodes.IsaacComputeOdometry"),
            ("PublishOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
            # Camera publisher (compressed)
            ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            # Lidar publisher
            ("LidarHelper", "omni.isaac.ros2_bridge.ROS2PublishLaserScan"),
        ],
        og.Controller.Keys.SET_VALUES: [
            # cmd_vel subscriber
            ("DifferentialController.inputs:topicName", "/cmd_vel"),
            # Odometry publisher
            ("PublishOdom.inputs:topicName", "/odom"),
            ("PublishOdom.inputs:frameId", "odom"),
            ("PublishOdom.inputs:chassisFrameId", "base_link"),
            # Camera
            ("CameraHelper.inputs:topicName", "/camera/image"),
            ("CameraHelper.inputs:type", "rgb"),
            ("CameraHelper.inputs:enableSemanticLabels", False),
            # Lidar
            ("LidarHelper.inputs:topicName", "/scan"),
            ("LidarHelper.inputs:frameId", "base_scan"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishOdom.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "LidarHelper.inputs:execIn"),
        ],
    },
)

# Reset and start the world
world.reset()

print("=== Isaac Sim scene ready. Simulation running. ===")

# Main simulation loop
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
