"""
Isaac Sim 5.0 standalone script: warehouse scene with TurtleBot3 Burger.
Runs headless with WebRTC streaming for interactive 3D viewing.
Enables ROS2 bridge for camera (MJPEG chase cam), odom, and cmd_vel.
"""

import sys
import os

# Ensure stdout is unbuffered so print() shows in docker logs
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', buffering=1)

# -- Isaac Sim startup (must happen before other omni imports) --
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True, "width": 1280, "height": 720})

# -- Now safe to import omni/isaac modules --
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path

import omni.kit.app
import carb

# -- Configure and enable WebRTC streaming --
settings = carb.settings.get_settings()
settings.set("/app/livestream/enabled", True)

# Read PUBLIC_IP from environment (set by docker-compose / start.sh)
public_ip = os.environ.get("PUBLIC_IP", "127.0.0.1")
settings.set("/exts/omni.kit.livestream.webrtc/publicIp", public_ip)
settings.set("/exts/omni.kit.livestream.webrtc/signalPort", 49100)
settings.set("/exts/omni.kit.livestream.webrtc/streamPort", 47998)

ext_manager = omni.kit.app.get_app().get_extension_manager()

# Enable WebRTC streaming (signaling on 49100, media on 47998/udp)
ext_manager.set_extension_enabled_immediate("omni.kit.livestream.webrtc", True)
print("WebRTC streaming enabled.")

# Enable ROS2 bridge extension (5.0 namespace)
ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

# Wait a few frames for extensions to initialize
for _ in range(10):
    simulation_app.update()

# Get NVIDIA assets root path (Nucleus server or local cache)
assets_root = get_assets_root_path()
if assets_root is None:
    print("ERROR: Could not find assets root path. Check Nucleus connection.", file=sys.stderr)
    simulation_app.close()
    sys.exit(1)

print(f"Assets root: {assets_root}")

# -- Load scene assets --
warehouse_usd = assets_root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
add_reference_to_stage(usd_path=warehouse_usd, prim_path="/World/Warehouse")

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

# Add chase camera for MJPEG (robot-following view)
from pxr import Sdf
camera_path = "/World/TurtleBot3/base_link/FollowCamera"
camera_prim = stage.DefinePrim(camera_path, "Camera")
camera_xform = UsdGeom.Xformable(camera_prim)
camera_xform.ClearXformOpOrder()
camera_xform.AddTranslateOp().Set(Gf.Vec3d(-0.5, 0.0, 0.4))
camera_xform.AddRotateXYZOp().Set(Gf.Vec3f(70.0, 0.0, -90.0))
camera_geom = UsdGeom.Camera(camera_prim)
camera_geom.GetFocalLengthAttr().Set(14.0)

# Add bird's eye camera following robot from above
birdseye_path = "/World/TurtleBot3/base_link/BirdEyeCamera"
birdseye_prim = stage.DefinePrim(birdseye_path, "Camera")
birdseye_xform = UsdGeom.Xformable(birdseye_prim)
birdseye_xform.ClearXformOpOrder()
birdseye_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.2))
birdseye_xform.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, -90.0))
birdseye_geom = UsdGeom.Camera(birdseye_prim)
birdseye_geom.GetFocalLengthAttr().Set(18.0)

# -- Create World and initialize physics BEFORE OmniGraph --
# World.reset() initializes the physics scene and articulations
world = World(stage_units_in_meters=1.0)
world.reset()

# Let the stage and physics settle
for _ in range(10):
    simulation_app.update()

# Create render products for cameras
import omni.replicator.core as rep
render_product = rep.create.render_product(camera_path, (640, 480))
render_product_path = render_product.path if hasattr(render_product, 'path') else str(render_product)

birdseye_render = rep.create.render_product(birdseye_path, (320, 320))
birdseye_render_path = birdseye_render.path if hasattr(birdseye_render, 'path') else str(birdseye_render)

print(f"Render product path: {render_product_path}")
print(f"Bird's eye render product path: {birdseye_render_path}")

# -- Configure ROS2 components via OmniGraph --
import omni.graph.core as og

try:
    (ros2_graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": "/World/ROS2Graph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                # Twist subscriber
                ("TwistSubscriber", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                # Break twist vectors into scalar components
                ("BreakLinearVel", "omni.graph.nodes.BreakVector3"),
                ("BreakAngularVel", "omni.graph.nodes.BreakVector3"),
                # Differential drive controller
                ("DiffDriveController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                # Articulation controller (applies wheel velocities to robot joints)
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                # Odometry
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                # Cameras
                ("CameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("BirdEyeCameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Twist subscriber
                ("TwistSubscriber.inputs:topicName", "/cmd_vel"),
                # Differential drive (TurtleBot3 Burger params)
                ("DiffDriveController.inputs:wheelDistance", 0.160),
                ("DiffDriveController.inputs:wheelRadius", 0.033),
                ("DiffDriveController.inputs:maxWheelSpeed", 6.67),
                # Articulation controller (applies to TurtleBot3 wheel joints)
                ("ArticulationController.inputs:robotPath", "/World/TurtleBot3"),
                ("ArticulationController.inputs:jointNames", ["wheel_left_joint", "wheel_right_joint"]),
                # Odometry compute
                ("ComputeOdometry.inputs:chassisPrim", "/World/TurtleBot3"),
                # Odometry publisher
                ("PublishOdom.inputs:topicName", "/odom"),
                ("PublishOdom.inputs:odomFrameId", "odom"),
                ("PublishOdom.inputs:chassisFrameId", "base_link"),
                # Chase camera publisher
                ("CameraHelper.inputs:topicName", "/camera/image"),
                ("CameraHelper.inputs:type", "rgb"),
                ("CameraHelper.inputs:renderProductPath", render_product_path),
                ("CameraHelper.inputs:enableSemanticLabels", False),
                ("CameraHelper.inputs:frameId", "camera_link"),
                # Bird's eye camera publisher
                ("BirdEyeCameraHelper.inputs:topicName", "/camera/birdseye"),
                ("BirdEyeCameraHelper.inputs:type", "rgb"),
                ("BirdEyeCameraHelper.inputs:renderProductPath", birdseye_render_path),
                ("BirdEyeCameraHelper.inputs:enableSemanticLabels", False),
                ("BirdEyeCameraHelper.inputs:frameId", "birdseye_link"),
            ],
            og.Controller.Keys.CONNECT: [
                # Tick all nodes
                ("OnPlaybackTick.outputs:tick", "TwistSubscriber.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "DiffDriveController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishOdom.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "BirdEyeCameraHelper.inputs:execIn"),
                # Twist subscriber → break vectors → differential controller
                ("TwistSubscriber.outputs:linearVelocity", "BreakLinearVel.inputs:tuple"),
                ("BreakLinearVel.outputs:x", "DiffDriveController.inputs:linearVelocity"),
                ("TwistSubscriber.outputs:angularVelocity", "BreakAngularVel.inputs:tuple"),
                ("BreakAngularVel.outputs:z", "DiffDriveController.inputs:angularVelocity"),
                # Differential controller → articulation controller (apply to wheels)
                ("DiffDriveController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
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

# Start physics playback (enables OnPlaybackTick)
world.play()

# Let physics and OmniGraph settle
for _ in range(10):
    simulation_app.update()

print("=== Isaac Sim scene ready. WebRTC streaming on port 49100. ===")

# Main simulation loop
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
