"""
Isaac Sim standalone script: warehouse scene with TurtleBot3 Burger.
Runs headless, enables ROS2 bridge for camera, odom, and cmd_vel.
"""

import sys

# -- Isaac Sim startup (must happen before other omni imports) --
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

# -- Now safe to import omni/isaac modules --
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

print(f"Assets root: {assets_root}")

# Create the simulation world
world = World(stage_units_in_meters=1.0)

# Load warehouse environment
warehouse_usd = assets_root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
add_reference_to_stage(usd_path=warehouse_usd, prim_path="/World/Warehouse")

# Load TurtleBot3 Burger
turtlebot_usd = assets_root + "/Isaac/Robots/Turtlebot/turtlebot3_burger.usd"
add_reference_to_stage(usd_path=turtlebot_usd, prim_path="/World/TurtleBot3")

# Position the robot
from pxr import Gf, UsdGeom
import omni.usd
stage = omni.usd.get_context().get_stage()
turtlebot_prim = stage.GetPrimAtPath("/World/TurtleBot3")
if turtlebot_prim.IsValid():
    xform = UsdGeom.Xformable(turtlebot_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

# Add a camera above the robot looking down (bird's eye for initial view)
from pxr import Sdf
camera_path = "/World/RobotCamera"
camera_prim = stage.DefinePrim(camera_path, "Camera")
camera_xform = UsdGeom.Xformable(camera_prim)
camera_xform.ClearXformOpOrder()
camera_xform.AddTranslateOp().Set(Gf.Vec3d(2.0, 2.0, 2.0))
# Point camera toward origin
camera_xform.AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 0.0, 135.0))
# Set focal length
camera_geom = UsdGeom.Camera(camera_prim)
camera_geom.GetFocalLengthAttr().Set(18.0)

# Let the stage settle
for _ in range(5):
    simulation_app.update()

# Create render product for the camera
import omni.replicator.core as rep
render_product = rep.create.render_product(camera_path, (640, 480))
render_product_path = render_product.path if hasattr(render_product, 'path') else str(render_product)

print(f"Render product path: {render_product_path}")

# -- Configure ROS2 components via OmniGraph --
import omni.graph.core as og

# Create the ROS2 action graph
# Note: ROS2SubscribeTwist outputs double3 vectors, but DifferentialController
# expects scalar doubles. We use BreakVector3 nodes to extract components.
try:
    (ros2_graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": "/World/ROS2Graph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                # Twist subscriber
                ("TwistSubscriber", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                # Break twist vectors into scalar components
                ("BreakLinearVel", "omni.graph.nodes.BreakVector3Double"),
                ("BreakAngularVel", "omni.graph.nodes.BreakVector3Double"),
                # Differential drive controller
                ("DiffDriveController", "omni.isaac.wheeled_robots.DifferentialController"),
                # Odometry
                ("ComputeOdometry", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                ("PublishOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                # Camera
                ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Twist subscriber
                ("TwistSubscriber.inputs:topicName", "/cmd_vel"),
                # Differential drive (TurtleBot3 Burger params)
                ("DiffDriveController.inputs:wheelDistance", 0.160),
                ("DiffDriveController.inputs:wheelRadius", 0.033),
                ("DiffDriveController.inputs:maxWheelSpeed", 6.67),
                # Odometry compute
                ("ComputeOdometry.inputs:chassisPrim", "/World/TurtleBot3"),
                # Odometry publisher
                ("PublishOdom.inputs:topicName", "/odom"),
                ("PublishOdom.inputs:odomFrameId", "odom"),
                ("PublishOdom.inputs:chassisFrameId", "base_link"),
                # Camera publisher
                ("CameraHelper.inputs:topicName", "/camera/image"),
                ("CameraHelper.inputs:type", "rgb"),
                ("CameraHelper.inputs:renderProductPath", render_product_path),
                ("CameraHelper.inputs:enableSemanticLabels", False),
                ("CameraHelper.inputs:frameId", "camera_link"),
            ],
            og.Controller.Keys.CONNECT: [
                # Tick all nodes
                ("OnPlaybackTick.outputs:tick", "TwistSubscriber.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "DiffDriveController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishOdom.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
                # Twist subscriber → break vectors → differential controller
                # Linear: extract x component (forward velocity)
                ("TwistSubscriber.outputs:linearVelocity", "BreakLinearVel.inputs:tuple"),
                ("BreakLinearVel.outputs:x", "DiffDriveController.inputs:linearVelocity"),
                # Angular: extract z component (yaw rate)
                ("TwistSubscriber.outputs:angularVelocity", "BreakAngularVel.inputs:tuple"),
                ("BreakAngularVel.outputs:z", "DiffDriveController.inputs:angularVelocity"),
                # Odometry compute → publish
                ("ComputeOdometry.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
            ],
        },
    )
    print("OmniGraph ROS2 pipeline created successfully.")
except Exception as e:
    print(f"WARNING: OmniGraph setup error: {e}", file=sys.stderr)
    print("Simulation will run but ROS2 topics may not work.", file=sys.stderr)

# Reset and start the world
world.reset()

print("=== Isaac Sim scene ready. Simulation running. ===")

# Main simulation loop
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
