# Isaac Sim AWS Remote Control — Design Document

**Date**: 2026-02-19
**Goal**: Demo-quality system to remotely control a TurtleBot3 Burger in an Isaac Sim warehouse environment via a web browser, all running on AWS.

---

## Architecture Overview

Single Docker Compose stack on an AWS g5.xlarge instance:

```
┌──────────────────────── AWS g5.xlarge (Ubuntu 22.04 AMI) ────────────────────────┐
│                                                                                   │
│  Terraform-provisioned EC2 + Security Groups + NVIDIA drivers                     │
│                                                                                   │
│  ┌─── docker-compose ──────────────────────────────────────────────────────────┐  │
│  │                                                                             │  │
│  │  ┌─────────────────┐    ROS2 DDS     ┌─────────────────────┐              │  │
│  │  │  Isaac Sim 4.2   │◄──(shared net)──►│  ROS2 Humble        │              │  │
│  │  │  (headless)      │                  │  ├─ rosbridge :9090  │              │  │
│  │  │  ├─ TurtleBot3   │                  │  ├─ web_video_server │              │  │
│  │  │  │   Burger       │                  │  │    :8080 (MJPEG)  │              │  │
│  │  │  ├─ Warehouse env│                  │  └─ nav2 (optional)  │              │  │
│  │  │  ├─ Camera sensor│                  └──────────┬────────────┘              │  │
│  │  │  └─ ROS2 bridge  │                             │                           │  │
│  │  └─────────────────┘                             │                           │  │
│  │                                                   │                           │  │
│  │  ┌────────────────────────────────────────────────┴──┐                       │  │
│  │  │  Nginx (:80/443)                                   │                       │  │
│  │  │  ├─ /           → React frontend (static files)    │                       │  │
│  │  │  ├─ /ws         → proxy to rosbridge :9090         │                       │  │
│  │  │  └─ /video      → proxy to web_video_server :8080  │                       │  │
│  │  └────────────────────────────────────────────────────┘                       │  │
│  └─────────────────────────────────────────────────────────────────────────────┘  │
│                                                                                   │
└───────────────────────────────────────────────────────── :80/443 ──► Browser      │
```

## AWS Infrastructure (Terraform)

**Resources:**
- **EC2**: g5.xlarge — 1x NVIDIA A10G (24GB VRAM), 4 vCPUs, 16GB RAM
- **AMI**: Ubuntu 22.04 Deep Learning AMI (NVIDIA drivers pre-installed)
- **Security Group**: SSH (22) from user IP, HTTPS (443) from anywhere
- **Elastic IP**: Persistent public IP across stop/start
- **EBS**: 200GB gp3 root volume

**Bootstrap** (user data script):
1. Install docker + docker-compose + nvidia-container-toolkit
2. Configure NVIDIA as default Docker runtime
3. Clone project repo
4. Pull Docker images
5. Run docker-compose up -d

**Terraform structure:**
```
terraform/
├── main.tf
├── variables.tf
├── outputs.tf
└── scripts/
    └── bootstrap.sh
```

**Cost**: ~$1.01/hr on-demand.

## Docker Compose Services

### isaac-sim
- **Image**: `nvcr.io/nvidia/isaac-sim:4.2.0`
- **Runtime**: nvidia (GPU access)
- **Mode**: Headless
- **Startup**: Runs `startup.py` which loads warehouse scene, spawns TurtleBot3 Burger with sensors, enables ROS2 bridge

### ros2
- **Image**: `ros:humble` + rosbridge + web_video_server + nav2
- **Nodes**:
  - `rosbridge_websocket` on port 9090
  - `web_video_server` on port 8080
  - Nav2 stack (optional, for waypoint navigation)

### nginx
- **Image**: `nginx:alpine`
- **Ports**: 80 (or 443 with self-signed cert) exposed to host
- **Proxy rules**: `/` → static frontend, `/ws` → rosbridge, `/video` → web_video_server

### frontend-build
- **Build step only**: Compiles React app via Vite, outputs to shared volume, exits

**Network**: Single Docker bridge network for DDS auto-discovery.

## Isaac Sim Scene

- **Environment**: Built-in NVIDIA warehouse scene
- **Robot**: TurtleBot3 Burger USD, spawned at warehouse entrance
- **Sensors**:
  - RGB Camera: 640x480, 30Hz → `/camera/image/compressed`
  - 2D LiDAR: 360-degree, 0.12-3.5m range → `/scan`
  - Odometry: Differential drive → `/odom`
- **Control**: Differential drive subscribes to `/cmd_vel`
- **ROS2 Bridge**: `omni.isaac.ros2_bridge` extension enabled on startup

## ROS2 Topics & Data Flow

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Browser → Sim | Teleop velocity |
| `/odom` | `nav_msgs/Odometry` | Sim → Browser | Position & velocity |
| `/camera/image/compressed` | `sensor_msgs/CompressedImage` | Sim → Browser (MJPEG) | Camera feed |
| `/scan` | `sensor_msgs/LaserScan` | Sim → Browser | LiDAR data |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Browser → Nav2 → Sim | Waypoint nav |

**Video path**: Isaac Sim → ROS2 topic → web_video_server → MJPEG over HTTP → Nginx proxy → browser `<img>` tag

**Control path**: Browser → roslibjs → WebSocket → Nginx proxy → rosbridge → ROS2 topic → Isaac Sim

## Video Streaming

- **Method**: MJPEG via `web_video_server` (ROS2 package)
- **Performance**: 15-30 fps, ~200-500ms latency
- **Upgrade path**: Can swap to WebRTC later if lower latency is needed

## Web Frontend

**Tech**: React + Vite + roslibjs

**Layout:**
```
┌──────────────────────────────────────────────────┐
│  Isaac Sim Remote Control          [Connected]    │
├──────────────────────────┬───────────────────────┤
│                          │  Teleop Controls       │
│   Camera Feed            │  [D-pad / WASD]        │
│   (MJPEG stream)         │  Speed: [slider]       │
│                          │                        │
│                          │  Nav2 Waypoint          │
│                          │  X: [__] Y: [__] [Go]  │
│                          │                        │
│                          │  Status                 │
│                          │  Position: x, y, θ     │
│                          │  Velocity: lin, ang    │
├──────────────────────────┴───────────────────────┤
│  Connection status          │ Latency indicator    │
└──────────────────────────────────────────────────┘
```

**Burger constraints enforced:**
- Linear velocity: [-0.22, 0.22] m/s
- Angular velocity: [-2.84, 2.84] rad/s
- Speed slider default: 0.1 m/s

**Keyboard shortcuts**: W/Up = forward, S/Down = back, A/Left = turn left, D/Right = turn right, Space = stop

## Project File Structure

```
isaacsim_aws/
├── terraform/
│   ├── main.tf
│   ├── variables.tf
│   ├── outputs.tf
│   └── scripts/
│       └── bootstrap.sh
├── docker/
│   ├── docker-compose.yml
│   ├── isaac-sim/
│   │   ├── Dockerfile
│   │   └── startup.py
│   ├── ros2/
│   │   ├── Dockerfile
│   │   └── launch/
│   │       └── bringup.launch.py
│   └── nginx/
│       └── nginx.conf
├── frontend/
│   ├── package.json
│   ├── vite.config.ts
│   ├── index.html
│   └── src/
│       ├── main.tsx
│       ├── App.tsx
│       ├── components/
│       │   ├── CameraView.tsx
│       │   ├── TeleopPad.tsx
│       │   ├── SpeedSlider.tsx
│       │   ├── NavGoal.tsx
│       │   └── StatusBar.tsx
│       └── lib/
│           └── ros.ts
└── docs/
    └── plans/
        └── 2026-02-19-isaacsim-aws-design.md
```
