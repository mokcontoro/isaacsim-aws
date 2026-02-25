"""
Isaac Sim 5.1 standalone script: warehouse scene with TurtleBot3 Burger.
Runs headless with WebRTC livestream for interactive 3D viewing.
Enables ROS2 bridge for chase camera (MJPEG), odom, and cmd_vel.
WebRTC provides interactive 3D god-view; MJPEG chase cam is a PiP overlay.
"""

import sys
import os
import math

# Kit's Python environment may redirect stdout. Use a log file + stderr as fallback.
LOG_FILE = "/tmp/startup.log"

def log(msg):
    """Print to stderr + log file so output always appears in docker logs."""
    line = f"[startup] {msg}"
    print(line, file=sys.stderr, flush=True)
    with open(LOG_FILE, "a") as f:
        f.write(line + "\n")

# -- Isaac Sim startup (must happen before other omni imports) --
log("Starting SimulationApp...")
from isaacsim import SimulationApp

# Replace _wait_for_viewport: run update frames to prime the renderer
# without waiting for viewport_handle (never set in headless Docker).
def _headless_viewport_init(self):
    for _ in range(60):
        self._app.update()
SimulationApp._wait_for_viewport = _headless_viewport_init

# headless=True  → no physical window (Docker has no display)
# hide_ui=False  → BUT still create the renderable framebuffer for streaming
# renderer       → RaytracedLighting for good quality + performance
# display_options → standard viewport display flags
simulation_app = SimulationApp({
    "headless": True,
    "hide_ui": False,
    "width": 1280,
    "height": 720,
    "window_width": 1920,
    "window_height": 1080,
    "renderer": "RaytracedLighting",
    "display_options": 3286,
})
log("SimulationApp ready")

# -- Now safe to import omni/isaac modules --
log("Importing omni.isaac.core.World...")
from omni.isaac.core import World
log("Importing stage utils...")
from omni.isaac.core.utils.stage import add_reference_to_stage
log("Importing nucleus...")
from omni.isaac.nucleus import get_assets_root_path
log("Importing enable_extension...")
from isaacsim.core.utils.extensions import enable_extension

log("Importing kit app + carb...")
import omni.kit.app
import carb
log("All imports done")

# -- Configure WebRTC streaming settings --
settings = carb.settings.get_settings()
settings.set("/app/livestream/enabled", True)
settings.set("/app/window/drawMouse", True)

# Read PUBLIC_IP from environment (set by docker-compose / start.sh)
public_ip = os.environ.get("PUBLIC_IP", "127.0.0.1")
settings.set("/exts/omni.kit.livestream.webrtc/publicIp", public_ip)
settings.set("/exts/omni.kit.livestream.webrtc/signalPort", 49100)
settings.set("/exts/omni.kit.livestream.webrtc/streamPort", 47998)

# Enable NVCF livestream service — wraps omni.kit.livestream.webrtc internally.
enable_extension("omni.services.livestream.nvcf")
log(f"WebRTC livestream enabled (publicIp={public_ip})")

ext_manager = omni.kit.app.get_app().get_extension_manager()

# Enable ROS2 bridge extension
ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

# Wait a few frames for extensions to initialize
for _ in range(10):
    simulation_app.update()

# Get NVIDIA assets root path (Nucleus server or local cache)
assets_root = get_assets_root_path()
if assets_root is None:
    log("ERROR: Could not find assets root path. Check Nucleus connection.")
    simulation_app.close()
    sys.exit(1)

log(f"Assets root: {assets_root}")

# -- Load scene assets --
warehouse_usd = assets_root + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
log(f"Loading warehouse: {warehouse_usd}")
add_reference_to_stage(usd_path=warehouse_usd, prim_path="/World/Warehouse")
log("Warehouse loaded")

# Asset path: nested under Turtlebot3/ subdirectory
turtlebot_usd = assets_root + "/Isaac/Robots/Turtlebot/Turtlebot3/turtlebot3_burger.usd"
log(f"Loading TurtleBot3: {turtlebot_usd}")
add_reference_to_stage(usd_path=turtlebot_usd, prim_path="/World/TurtleBot3")
log("TurtleBot3 loaded")

# Position the robot
from pxr import Gf, UsdGeom
import omni.usd
stage = omni.usd.get_context().get_stage()
turtlebot_prim = stage.GetPrimAtPath("/World/TurtleBot3")
if turtlebot_prim.IsValid():
    xform = UsdGeom.Xformable(turtlebot_prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

# -- Cameras --
# Created at world root, NOT parented under the articulation.
# PhysX drives articulation link transforms via its own internal state,
# so USD child prims don't follow. We manually update camera transforms
# each step using the Dynamic Control interface (see main loop below).
from pxr import Sdf

# Chase camera (follows robot from behind and above)
camera_path = "/World/FollowCamera"
camera_prim = stage.DefinePrim(camera_path, "Camera")
camera_xform = UsdGeom.Xformable(camera_prim)
camera_xform.ClearXformOpOrder()
camera_xform.AddTranslateOp().Set(Gf.Vec3d(-0.5, 0.0, 0.4))
camera_xform.AddRotateXYZOp().Set(Gf.Vec3f(70.0, 0.0, -90.0))
camera_geom = UsdGeom.Camera(camera_prim)
camera_geom.GetFocalLengthAttr().Set(14.0)

# Camera offsets relative to robot
CHASE_CAM_OFFSET = Gf.Vec3d(-0.5, 0.0, 0.4)   # behind and above
CHASE_CAM_ROTATE = Gf.Vec3f(70.0, 0.0, -90.0)  # looking forward-down

# -- Create World and initialize physics BEFORE OmniGraph --
log("Creating World...")
world = World(stage_units_in_meters=1.0)
world.reset()
log("World created and reset")

# Let the stage and physics settle
for _ in range(10):
    simulation_app.update()

# Create render products for cameras
import omni.replicator.core as rep
render_product = rep.create.render_product(camera_path, (640, 480))
render_product_path = render_product.path if hasattr(render_product, 'path') else str(render_product)

log(f"Render product path: {render_product_path}")

# -- Configure ROS2 components via OmniGraph --
import omni.graph.core as og
import usdrt.Sdf

try:
    (ros2_graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": "/World/ROS2Graph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("TwistSubscriber", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinearVel", "omni.graph.nodes.BreakVector3"),
                ("BreakAngularVel", "omni.graph.nodes.BreakVector3"),
                ("DiffDriveController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("CameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("TwistSubscriber.inputs:topicName", "/cmd_vel"),
                ("DiffDriveController.inputs:wheelDistance", 0.160),
                ("DiffDriveController.inputs:wheelRadius", 0.033),
                ("DiffDriveController.inputs:maxWheelSpeed", 6.67),
                ("ArticulationController.inputs:robotPath", "/World/TurtleBot3"),
                ("ArticulationController.inputs:jointNames", ["wheel_left_joint", "wheel_right_joint"]),
                ("ComputeOdometry.inputs:chassisPrim", [usdrt.Sdf.Path("/World/TurtleBot3")]),
                ("PublishOdom.inputs:topicName", "/odom"),
                ("PublishOdom.inputs:odomFrameId", "odom"),
                ("PublishOdom.inputs:chassisFrameId", "base_link"),
                ("CameraHelper.inputs:topicName", "/camera/image"),
                ("CameraHelper.inputs:type", "rgb"),
                ("CameraHelper.inputs:renderProductPath", render_product_path),
                ("CameraHelper.inputs:enableSemanticLabels", False),
                ("CameraHelper.inputs:frameId", "camera_link"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "TwistSubscriber.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "DiffDriveController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
                ("TwistSubscriber.outputs:linearVelocity", "BreakLinearVel.inputs:tuple"),
                ("BreakLinearVel.outputs:x", "DiffDriveController.inputs:linearVelocity"),
                ("TwistSubscriber.outputs:angularVelocity", "BreakAngularVel.inputs:tuple"),
                ("BreakAngularVel.outputs:z", "DiffDriveController.inputs:angularVelocity"),
                ("DiffDriveController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("ComputeOdometry.outputs:execOut", "PublishOdom.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
            ],
        },
    )
    log("OmniGraph ROS2 pipeline created successfully.")
except Exception as e:
    log(f"WARNING: OmniGraph setup error: {e}")
    log("Simulation will run but ROS2 topics may not work.")

# Start physics playback (enables OnPlaybackTick)
world.play()

# Let physics, OmniGraph, and streaming settle
for _ in range(20):
    simulation_app.update()

log("=== Isaac Sim scene ready. WebRTC streaming on port 49100. ===")

# -- Camera follow setup --
# Use Dynamic Control interface to read physics-updated transforms.
# The USD stage API (ComputeLocalToWorldTransform, XFormPrim.get_world_pose)
# does NOT see PhysX updates during simulation. DC reads directly from PhysX.
from omni.isaac.dynamic_control import _dynamic_control

chase_translate_op = camera_xform.GetOrderedXformOps()[0]
chase_rotate_op = camera_xform.GetOrderedXformOps()[1]

dc = _dynamic_control.acquire_dynamic_control_interface()
# TurtleBot3 prim hierarchy: /World/TurtleBot3/base_footprint/base_link
# PhysX moves base_footprint (the articulation root link)
rb_handle = dc.get_rigid_body("/World/TurtleBot3/base_footprint")
log(f"DC rigid body handle valid: {rb_handle != _dynamic_control.INVALID_HANDLE}")


def update_chase_camera():
    """Move chase camera to follow the robot using Dynamic Control (PhysX) transforms."""
    pose = dc.get_rigid_body_pose(rb_handle)
    robot_pos = (pose.p.x, pose.p.y, pose.p.z)

    # Quaternion to yaw
    w, x, y, z = pose.r.w, pose.r.x, pose.r.y, pose.r.z
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)

    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    cx, cy = CHASE_CAM_OFFSET[0], CHASE_CAM_OFFSET[1]
    chase_translate_op.Set(Gf.Vec3d(
        robot_pos[0] + cx * cos_y - cy * sin_y,
        robot_pos[1] + cx * sin_y + cy * cos_y,
        robot_pos[2] + CHASE_CAM_OFFSET[2],
    ))
    yaw_deg = math.degrees(yaw)
    chase_rotate_op.Set(Gf.Vec3f(
        CHASE_CAM_ROTATE[0],
        CHASE_CAM_ROTATE[1],
        CHASE_CAM_ROTATE[2] + yaw_deg,
    ))


# Main simulation loop — update cameras between physics and render
while simulation_app.is_running():
    world.step(render=False)   # Physics only
    update_chase_camera()      # Move chase camera to follow robot
    simulation_app.update()    # Render with updated camera positions

simulation_app.close()
