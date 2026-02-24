"""
Isaac Sim 5.0 standalone script: warehouse scene with TurtleBot3 Burger.
Runs headless with WebRTC signaling server.
Enables ROS2 bridge for cameras (MJPEG chase + bird's eye), odom, and cmd_vel.
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

# 5.0 asset path: nested under Turtlebot3/ subdirectory
turtlebot_usd = assets_root + "/Isaac/Robots/Turtlebot/Turtlebot3/turtlebot3_burger.usd"
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
# NOTE: Cameras are created at world root, NOT parented under the articulation.
# PhysX drives articulation link transforms independently of the USD hierarchy,
# so child prims of articulation links don't follow the robot.
# Instead, we manually update camera transforms each step (see main loop below).
from pxr import Sdf
camera_path = "/World/FollowCamera"
camera_prim = stage.DefinePrim(camera_path, "Camera")
camera_xform = UsdGeom.Xformable(camera_prim)
camera_xform.ClearXformOpOrder()
# Initial position — will be overwritten by step callback
camera_xform.AddTranslateOp().Set(Gf.Vec3d(-0.5, 0.0, 0.4))
camera_xform.AddRotateXYZOp().Set(Gf.Vec3f(70.0, 0.0, -90.0))
camera_geom = UsdGeom.Camera(camera_prim)
camera_geom.GetFocalLengthAttr().Set(14.0)

# Add bird's eye camera following robot from above
birdseye_path = "/World/BirdEyeCamera"
birdseye_prim = stage.DefinePrim(birdseye_path, "Camera")
birdseye_xform = UsdGeom.Xformable(birdseye_prim)
birdseye_xform.ClearXformOpOrder()
birdseye_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.2))
birdseye_xform.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, -90.0))
birdseye_geom = UsdGeom.Camera(birdseye_prim)
birdseye_geom.GetFocalLengthAttr().Set(18.0)

# Camera offsets relative to robot base_link
# Chase cam: behind and above, looking forward-down
CHASE_CAM_OFFSET = Gf.Vec3d(-0.5, 0.0, 0.4)
CHASE_CAM_ROTATE = Gf.Vec3f(70.0, 0.0, -90.0)
# Bird's eye: directly above, looking straight down
BIRDSEYE_OFFSET = Gf.Vec3d(0.0, 0.0, 1.2)
BIRDSEYE_ROTATE = Gf.Vec3f(90.0, 0.0, -90.0)

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
import usdrt.Sdf

try:
    (ros2_graph, nodes, _, _) = og.Controller.edit(
        {"graph_path": "/World/ROS2Graph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
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
                # Odometry compute (chassisPrim is a rel type in 5.0)
                ("ComputeOdometry.inputs:chassisPrim", [usdrt.Sdf.Path("/World/TurtleBot3")]),
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
                # Tick nodes
                ("OnPlaybackTick.outputs:tick", "TwistSubscriber.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "DiffDriveController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "BirdEyeCameraHelper.inputs:execIn"),
                # Twist subscriber → break vectors → differential controller
                ("TwistSubscriber.outputs:linearVelocity", "BreakLinearVel.inputs:tuple"),
                ("BreakLinearVel.outputs:x", "DiffDriveController.inputs:linearVelocity"),
                ("TwistSubscriber.outputs:angularVelocity", "BreakAngularVel.inputs:tuple"),
                ("BreakAngularVel.outputs:z", "DiffDriveController.inputs:angularVelocity"),
                # Differential controller → articulation controller (apply to wheels)
                ("DiffDriveController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                # Odometry: chain ComputeOdometry → PublishOdom (execOut → execIn)
                ("ComputeOdometry.outputs:execOut", "PublishOdom.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                # Timestamps from simulation time
                ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
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

# -- Camera follow helper --
import math
from pxr import Usd

chase_translate_op = camera_xform.GetOrderedXformOps()[0]
chase_rotate_op = camera_xform.GetOrderedXformOps()[1]
birdseye_translate_op = birdseye_xform.GetOrderedXformOps()[0]
birdseye_rotate_op = birdseye_xform.GetOrderedXformOps()[1]

# -- Find the right API to read physics-updated robot position --
# USD stage API and XFormPrim don't see PhysX updates during simulation.
# Try multiple approaches:

# Approach 1: Dynamic control interface (reads directly from PhysX)
_dc = None
_rb_handle = None
try:
    from omni.isaac.dynamic_control import _dynamic_control
    _dc = _dynamic_control.acquire_dynamic_control_interface()
    _rb_handle = _dc.get_rigid_body("/World/TurtleBot3/base_footprint")
    if _rb_handle == _dynamic_control.INVALID_HANDLE:
        print("DC: base_footprint handle INVALID, trying /World/TurtleBot3")
        _rb_handle = _dc.get_rigid_body("/World/TurtleBot3")
    if _rb_handle != _dynamic_control.INVALID_HANDLE:
        pose = _dc.get_rigid_body_pose(_rb_handle)
        print(f"DC: initial pos=({pose.p.x:.4f},{pose.p.y:.4f},{pose.p.z:.4f})")
    else:
        print("DC: could not get rigid body handle")
        _dc = None
except Exception as e:
    print(f"DC: not available: {e}")
    _dc = None

# Approach 2: Read from Fabric (USDRT) layer — where PhysX writes transforms
_fabric_prim = None
try:
    import usdrt
    from usdrt import Usd as UsdRT
    stage_id = omni.usd.get_context().get_stage_id()
    rt_stage = UsdRT.Stage.Attach(stage_id)
    _fabric_prim = rt_stage.GetPrimAtPath("/World/TurtleBot3/base_footprint")
    if _fabric_prim.IsValid():
        print("Fabric: base_footprint prim found")
    else:
        print("Fabric: base_footprint not found, trying TurtleBot3 root")
        _fabric_prim = rt_stage.GetPrimAtPath("/World/TurtleBot3")
except Exception as e:
    print(f"Fabric: not available: {e}")
    _fabric_prim = None

print(f"Position APIs: DC={_dc is not None}, Fabric={_fabric_prim is not None}")

_diag_counter = 0

def update_cameras():
    """Move cameras to follow the robot each frame using physics-aware API."""
    global _diag_counter

    robot_pos = None
    yaw = 0.0

    # Try DC interface first (reads directly from PhysX)
    if _dc is not None and _rb_handle is not None:
        try:
            pose = _dc.get_rigid_body_pose(_rb_handle)
            robot_pos = (pose.p.x, pose.p.y, pose.p.z)
            # Quaternion to yaw (PhysX quaternion is xyzw)
            w, x, y, z = pose.r.w, pose.r.x, pose.r.y, pose.r.z
            siny = 2.0 * (w * z + x * y)
            cosy = 1.0 - 2.0 * (y * y + z * z)
            yaw = math.atan2(siny, cosy)
        except Exception:
            pass

    # Fallback: try Fabric/USDRT layer
    if robot_pos is None and _fabric_prim is not None and _fabric_prim.IsValid():
        try:
            translate_attr = _fabric_prim.GetAttribute("xformOp:translate")
            if translate_attr.IsValid():
                t = translate_attr.Get()
                robot_pos = (float(t[0]), float(t[1]), float(t[2]))
            orient_attr = _fabric_prim.GetAttribute("xformOp:orient")
            if orient_attr.IsValid():
                q = orient_attr.Get()
                # USDRT quaternion might be (w,x,y,z) or (x,y,z,w)
                w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
                siny = 2.0 * (w * z + x * y)
                cosy = 1.0 - 2.0 * (y * y + z * z)
                yaw = math.atan2(siny, cosy)
        except Exception:
            pass

    if robot_pos is None:
        return

    # Diagnostic: print every 500 frames
    _diag_counter += 1
    if _diag_counter <= 3 or _diag_counter % 500 == 0:
        print(f"[cam] frame={_diag_counter} robot=({robot_pos[0]:.3f},{robot_pos[1]:.3f},{robot_pos[2]:.3f}) yaw={math.degrees(yaw):.1f}")

    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    # Chase camera: offset rotated by robot yaw
    cx = CHASE_CAM_OFFSET[0]
    cy = CHASE_CAM_OFFSET[1]
    chase_pos = Gf.Vec3d(
        robot_pos[0] + cx * cos_y - cy * sin_y,
        robot_pos[1] + cx * sin_y + cy * cos_y,
        robot_pos[2] + CHASE_CAM_OFFSET[2],
    )
    yaw_deg = math.degrees(yaw)
    chase_rot = Gf.Vec3f(
        CHASE_CAM_ROTATE[0],
        CHASE_CAM_ROTATE[1],
        CHASE_CAM_ROTATE[2] + yaw_deg,
    )
    chase_translate_op.Set(chase_pos)
    chase_rotate_op.Set(chase_rot)

    # Bird's eye camera: directly above robot, rotated with robot
    bx = BIRDSEYE_OFFSET[0]
    by = BIRDSEYE_OFFSET[1]
    birdseye_pos = Gf.Vec3d(
        robot_pos[0] + bx * cos_y - by * sin_y,
        robot_pos[1] + bx * sin_y + by * cos_y,
        robot_pos[2] + BIRDSEYE_OFFSET[2],
    )
    birdseye_rot = Gf.Vec3f(
        BIRDSEYE_ROTATE[0],
        BIRDSEYE_ROTATE[1],
        BIRDSEYE_ROTATE[2] + yaw_deg,
    )
    birdseye_translate_op.Set(birdseye_pos)
    birdseye_rotate_op.Set(birdseye_rot)


# Main simulation loop — update cameras BEFORE render
while simulation_app.is_running():
    world.step(render=False)   # Physics only
    update_cameras()           # Move cameras to follow robot
    simulation_app.update()    # Render with updated camera positions

simulation_app.close()
