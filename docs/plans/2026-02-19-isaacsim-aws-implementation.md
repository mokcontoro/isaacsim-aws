# Isaac Sim AWS Remote Control — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Deploy a TurtleBot3 Burger in an Isaac Sim warehouse on AWS g5.xlarge, controlled remotely via a web browser with live MJPEG video streaming.

**Architecture:** Single Docker Compose stack (Isaac Sim + ROS2 + Nginx + frontend) on a Terraform-provisioned g5.xlarge. Video via web_video_server (MJPEG), control via rosbridge WebSocket, frontend in React + roslibjs.

**Tech Stack:** Terraform, Docker Compose, NVIDIA Isaac Sim 4.2, ROS2 Humble, rosbridge, web_video_server, Nginx, React, Vite, TypeScript, roslibjs

**Design doc:** `docs/plans/2026-02-19-isaacsim-aws-design.md`

---

## Task 1: Initialize Git Repository & Project Skeleton

**Files:**
- Create: `.gitignore`
- Create: `terraform/` (empty dirs)
- Create: `docker/` (empty dirs)
- Create: `frontend/` (empty dir)

**Step 1: Initialize git repo**

Run: `git init`

**Step 2: Create .gitignore**

```gitignore
# Terraform
terraform/.terraform/
terraform/*.tfstate
terraform/*.tfstate.backup
terraform/*.tfplan
terraform/.terraform.lock.hcl

# Node
frontend/node_modules/
frontend/dist/

# OS
.DS_Store
Thumbs.db

# IDE
.vscode/
.idea/

# Environment
.env
*.pem
```

**Step 3: Create directory skeleton**

Run:
```bash
mkdir -p terraform/scripts
mkdir -p docker/isaac-sim
mkdir -p docker/ros2/launch
mkdir -p docker/nginx
mkdir -p frontend/src/components
mkdir -p frontend/src/lib
```

**Step 4: Commit**

```bash
git add .
git commit -m "chore: initialize project skeleton"
```

---

## Task 2: Terraform — Variables & Provider

**Files:**
- Create: `terraform/variables.tf`
- Create: `terraform/main.tf` (provider block only)

**Step 1: Write variables.tf**

```hcl
variable "aws_region" {
  description = "AWS region to deploy in"
  type        = string
  default     = "us-east-1"
}

variable "instance_type" {
  description = "EC2 instance type (must have NVIDIA GPU)"
  type        = string
  default     = "g5.xlarge"
}

variable "key_name" {
  description = "Name of existing AWS key pair for SSH access"
  type        = string
}

variable "allowed_ssh_cidr" {
  description = "CIDR block allowed to SSH (your IP, e.g. 1.2.3.4/32)"
  type        = string
}

variable "project_repo_url" {
  description = "Git repo URL to clone onto the instance"
  type        = string
  default     = ""
}

variable "root_volume_size" {
  description = "Root EBS volume size in GB"
  type        = number
  default     = 200
}
```

**Step 2: Write provider block in main.tf**

```hcl
terraform {
  required_version = ">= 1.5"

  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = "~> 5.0"
    }
  }
}

provider "aws" {
  region = var.aws_region
}
```

**Step 3: Validate**

Run: `cd terraform && terraform init && terraform validate`
Expected: "Success! The configuration is valid."

**Step 4: Commit**

```bash
git add terraform/variables.tf terraform/main.tf
git commit -m "feat(terraform): add provider config and variables"
```

---

## Task 3: Terraform — Bootstrap Script

**Files:**
- Create: `terraform/scripts/bootstrap.sh`

**Step 1: Write bootstrap.sh**

```bash
#!/bin/bash
set -euxo pipefail

# Log everything for debugging
exec > >(tee /var/log/user-data.log) 2>&1

echo "=== Starting bootstrap ==="

# Wait for dpkg lock (cloud-init may still be running)
while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
  echo "Waiting for dpkg lock..."
  sleep 5
done

# Update system
apt-get update -y
apt-get upgrade -y

# Install Docker
apt-get install -y ca-certificates curl gnupg
install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null
apt-get update -y
apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Install NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
apt-get update -y
apt-get install -y nvidia-container-toolkit

# Configure NVIDIA runtime as default for Docker
nvidia-ctk runtime configure --runtime=docker
systemctl restart docker

# Add ubuntu user to docker group
usermod -aG docker ubuntu

# Clone project repo if provided
if [ -n "${PROJECT_REPO_URL}" ]; then
  cd /home/ubuntu
  git clone "${PROJECT_REPO_URL}" isaacsim_aws
  chown -R ubuntu:ubuntu isaacsim_aws
fi

echo "=== Bootstrap complete ==="
```

**Step 2: Make executable**

Run: `chmod +x terraform/scripts/bootstrap.sh`

**Step 3: Commit**

```bash
git add terraform/scripts/bootstrap.sh
git commit -m "feat(terraform): add EC2 bootstrap script"
```

---

## Task 4: Terraform — EC2 Instance, Security Group, Elastic IP, Outputs

**Files:**
- Modify: `terraform/main.tf`
- Create: `terraform/outputs.tf`

**Step 1: Add data sources and resources to main.tf**

Append to `terraform/main.tf`:

```hcl
# Look up the latest Ubuntu 22.04 Deep Learning AMI with NVIDIA drivers
data "aws_ami" "deep_learning" {
  most_recent = true
  owners      = ["amazon"]

  filter {
    name   = "name"
    values = ["Deep Learning AMI GPU PyTorch * (Ubuntu 22.04) *"]
  }

  filter {
    name   = "architecture"
    values = ["x86_64"]
  }

  filter {
    name   = "state"
    values = ["available"]
  }
}

# Security group
resource "aws_security_group" "isaacsim" {
  name_prefix = "isaacsim-"
  description = "Isaac Sim remote control"

  # SSH from allowed IP
  ingress {
    from_port   = 22
    to_port     = 22
    protocol    = "tcp"
    cidr_blocks = [var.allowed_ssh_cidr]
    description = "SSH access"
  }

  # HTTP (web frontend)
  ingress {
    from_port   = 80
    to_port     = 80
    protocol    = "tcp"
    cidr_blocks = [var.allowed_ssh_cidr]
    description = "HTTP web frontend"
  }

  # HTTPS (web frontend)
  ingress {
    from_port   = 443
    to_port     = 443
    protocol    = "tcp"
    cidr_blocks = [var.allowed_ssh_cidr]
    description = "HTTPS web frontend"
  }

  # All outbound
  egress {
    from_port   = 0
    to_port     = 0
    protocol    = "-1"
    cidr_blocks = ["0.0.0.0/0"]
    description = "All outbound"
  }

  tags = {
    Name = "isaacsim-sg"
  }
}

# EC2 instance
resource "aws_instance" "isaacsim" {
  ami                    = data.aws_ami.deep_learning.id
  instance_type          = var.instance_type
  key_name               = var.key_name
  vpc_security_group_ids = [aws_security_group.isaacsim.id]

  root_block_device {
    volume_size = var.root_volume_size
    volume_type = "gp3"
  }

  user_data = templatefile("${path.module}/scripts/bootstrap.sh", {
    PROJECT_REPO_URL = var.project_repo_url
  })

  tags = {
    Name = "isaacsim-remote"
  }
}

# Elastic IP
resource "aws_eip" "isaacsim" {
  instance = aws_instance.isaacsim.id

  tags = {
    Name = "isaacsim-eip"
  }
}
```

**Step 2: Write outputs.tf**

```hcl
output "public_ip" {
  description = "Public IP address of the Isaac Sim instance"
  value       = aws_eip.isaacsim.public_ip
}

output "ssh_command" {
  description = "SSH command to connect to the instance"
  value       = "ssh -i ~/.ssh/${var.key_name}.pem ubuntu@${aws_eip.isaacsim.public_ip}"
}

output "web_url" {
  description = "URL to access the web frontend"
  value       = "http://${aws_eip.isaacsim.public_ip}"
}
```

**Step 3: Validate**

Run: `cd terraform && terraform validate`
Expected: "Success! The configuration is valid."

**Step 4: Commit**

```bash
git add terraform/main.tf terraform/outputs.tf
git commit -m "feat(terraform): add EC2 instance, security group, elastic IP"
```

---

## Task 5: Docker Compose & Networking

**Files:**
- Create: `docker/docker-compose.yml`

**Step 1: Write docker-compose.yml**

```yaml
services:
  isaac-sim:
    image: nvcr.io/nvidia/isaac-sim:4.2.0
    container_name: isaac-sim
    runtime: nvidia
    environment:
      - ACCEPT_EULA=Y
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
      # ROS2 domain ID (must match ros2 container)
      - ROS_DOMAIN_ID=0
      # Use Cyclone DDS for reliable cross-container discovery
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    volumes:
      - ./isaac-sim/startup.py:/app/startup.py:ro
    entrypoint: ["./python.sh", "/app/startup.py"]
    networks:
      - rosnet
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]

  ros2:
    build:
      context: ./ros2
      dockerfile: Dockerfile
    container_name: ros2
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    volumes:
      - ./ros2/launch:/ros2_ws/launch:ro
    command: ros2 launch /ros2_ws/launch/bringup.launch.py
    networks:
      - rosnet
    depends_on:
      - isaac-sim

  nginx:
    image: nginx:alpine
    container_name: nginx
    ports:
      - "80:80"
    volumes:
      - ./nginx/nginx.conf:/etc/nginx/nginx.conf:ro
      - frontend-dist:/usr/share/nginx/html:ro
    networks:
      - rosnet
    depends_on:
      - ros2

  frontend-build:
    image: node:20-alpine
    container_name: frontend-build
    working_dir: /app
    volumes:
      - ../frontend:/app:ro
      - frontend-dist:/output
    command: sh -c "npm ci && npm run build && cp -r dist/* /output/"

volumes:
  frontend-dist:

networks:
  rosnet:
    driver: bridge
```

**Step 2: Validate YAML syntax**

Run: `cd docker && docker compose config --quiet 2>&1 || echo "Syntax check (full validation requires images)"`

**Step 3: Commit**

```bash
git add docker/docker-compose.yml
git commit -m "feat(docker): add docker-compose with all 4 services"
```

---

## Task 6: ROS2 Container — Dockerfile & Launch File

**Files:**
- Create: `docker/ros2/Dockerfile`
- Create: `docker/ros2/launch/bringup.launch.py`

**Step 1: Write Dockerfile**

```dockerfile
FROM ros:humble

# Install required ROS2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-suite \
    ros-humble-web-video-server \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Set DDS implementation
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Create workspace
RUN mkdir -p /ros2_ws/launch

WORKDIR /ros2_ws

# Source ROS2 on entry
ENTRYPOINT ["/ros_entrypoint.sh"]
```

**Step 2: Write bringup.launch.py**

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # rosbridge WebSocket server — exposes ROS2 topics to browser via JSON/WebSocket
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0',
                'unregister_timeout': 10.0,
            }],
            output='screen',
        ),

        # web_video_server — serves camera topics as MJPEG streams over HTTP
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{
                'port': 8080,
                'address': '0.0.0.0',
                'default_stream_type': 'mjpeg',
            }],
            output='screen',
        ),
    ])
```

> **Note:** Nav2 is intentionally left out of the launch file for now. It adds significant startup time. We'll add it as a follow-up task once basic teleop works end-to-end.

**Step 3: Commit**

```bash
git add docker/ros2/Dockerfile docker/ros2/launch/bringup.launch.py
git commit -m "feat(ros2): add Dockerfile and launch file with rosbridge + web_video_server"
```

---

## Task 7: Isaac Sim Startup Script

**Files:**
- Create: `docker/isaac-sim/startup.py`

**Step 1: Write startup.py**

This is the Isaac Sim standalone Python script that sets up the scene. It uses the `isaacsim` and `omni` APIs available inside the Isaac Sim container.

```python
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
```

> **Important note:** The exact OmniGraph node names and API may vary slightly between Isaac Sim versions. This script targets Isaac Sim 4.2 and will need validation on the actual container. The node wiring pattern (OnPlaybackTick → sensor/controller nodes) is the standard approach. During integration testing on AWS, we'll verify and adjust node names if needed by checking `omni.graph.core` documentation inside the container.

**Step 2: Commit**

```bash
git add docker/isaac-sim/startup.py
git commit -m "feat(isaac-sim): add startup script with warehouse + Burger + ROS2 action graph"
```

---

## Task 8: Nginx Configuration

**Files:**
- Create: `docker/nginx/nginx.conf`

**Step 1: Write nginx.conf**

```nginx
worker_processes auto;

events {
    worker_connections 1024;
}

http {
    include       /etc/nginx/mime.types;
    default_type  application/octet-stream;
    sendfile      on;
    keepalive_timeout 65;

    # Increase buffer sizes for video streaming
    proxy_buffering off;

    upstream rosbridge {
        server ros2:9090;
    }

    upstream video_server {
        server ros2:8080;
    }

    server {
        listen 80;
        server_name _;

        # Frontend static files
        location / {
            root /usr/share/nginx/html;
            index index.html;
            try_files $uri $uri/ /index.html;
        }

        # WebSocket proxy to rosbridge
        location /ws {
            proxy_pass http://rosbridge;
            proxy_http_version 1.1;
            proxy_set_header Upgrade $http_upgrade;
            proxy_set_header Connection "upgrade";
            proxy_set_header Host $host;
            proxy_set_header X-Real-IP $remote_addr;
            proxy_read_timeout 86400;
        }

        # MJPEG video stream proxy to web_video_server
        location /video/ {
            proxy_pass http://video_server/;
            proxy_http_version 1.1;
            proxy_set_header Host $host;
            proxy_set_header X-Real-IP $remote_addr;
            proxy_buffering off;
            proxy_cache off;
            # Needed for MJPEG streaming
            proxy_read_timeout 86400;
            chunked_transfer_encoding off;
        }
    }
}
```

**Step 2: Commit**

```bash
git add docker/nginx/nginx.conf
git commit -m "feat(nginx): add reverse proxy config for frontend, rosbridge, and video"
```

---

## Task 9: Frontend — Project Setup & ROS2 Connection Library

**Files:**
- Create: `frontend/package.json`
- Create: `frontend/tsconfig.json`
- Create: `frontend/vite.config.ts`
- Create: `frontend/index.html`
- Create: `frontend/src/lib/ros.ts`

**Step 1: Initialize frontend project**

Run (from project root):
```bash
cd frontend
npm init -y
npm install react react-dom roslib
npm install -D vite @vitejs/plugin-react typescript @types/react @types/react-dom
```

**Step 2: Write tsconfig.json**

```json
{
  "compilerOptions": {
    "target": "ES2020",
    "useDefineForClassFields": true,
    "lib": ["ES2020", "DOM", "DOM.Iterable"],
    "module": "ESNext",
    "skipLibCheck": true,
    "moduleResolution": "bundler",
    "allowImportingTsExtensions": true,
    "isolatedModules": true,
    "moduleDetection": "force",
    "noEmit": true,
    "jsx": "react-jsx",
    "strict": true,
    "noUnusedLocals": true,
    "noUnusedParameters": true
  },
  "include": ["src"]
}
```

**Step 3: Write vite.config.ts**

```typescript
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/ws': {
        target: 'ws://localhost:9090',
        ws: true,
      },
      '/video': {
        target: 'http://localhost:8080',
        rewrite: (path) => path.replace(/^\/video/, ''),
      },
    },
  },
})
```

**Step 4: Write index.html**

```html
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Isaac Sim Remote Control</title>
  </head>
  <body>
    <div id="root"></div>
    <script type="module" src="/src/main.tsx"></script>
  </body>
</html>
```

**Step 5: Write ros.ts — roslibjs connection singleton**

```typescript
import ROSLIB from 'roslib';

// Connect to rosbridge via Nginx proxy
const wsUrl = `ws://${window.location.host}/ws`;

export const ros = new ROSLIB.Ros({ url: wsUrl });

ros.on('connection', () => console.log('Connected to rosbridge'));
ros.on('error', (err) => console.error('rosbridge error:', err));
ros.on('close', () => console.log('rosbridge connection closed'));

// Typed topic helpers
export const cmdVelTopic = new ROSLIB.Topic({
  ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/msg/Twist',
});

export const odomTopic = new ROSLIB.Topic({
  ros,
  name: '/odom',
  messageType: 'nav_msgs/msg/Odometry',
});

export const goalPoseTopic = new ROSLIB.Topic({
  ros,
  name: '/goal_pose',
  messageType: 'geometry_msgs/msg/PoseStamped',
});

// Video stream URL (MJPEG via web_video_server through Nginx)
export const videoStreamUrl =
  `/video/stream?topic=/camera/image/compressed&type=mjpeg&quality=80`;

// Publish a Twist message
export function publishTwist(linear: number, angular: number) {
  const twist = new ROSLIB.Message({
    linear: { x: linear, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angular },
  });
  cmdVelTopic.publish(twist);
}

// Publish a zero-velocity stop command
export function publishStop() {
  publishTwist(0, 0);
}
```

**Step 6: Commit**

```bash
git add frontend/
git commit -m "feat(frontend): initialize Vite React project with roslibjs connection library"
```

---

## Task 10: Frontend — CameraView Component

**Files:**
- Create: `frontend/src/components/CameraView.tsx`

**Step 1: Write CameraView.tsx**

```tsx
import { videoStreamUrl } from '../lib/ros';

export function CameraView() {
  return (
    <div style={{ position: 'relative', width: '100%', background: '#1a1a1a' }}>
      <img
        src={videoStreamUrl}
        alt="Robot camera feed"
        style={{
          width: '100%',
          height: 'auto',
          display: 'block',
          minHeight: '360px',
        }}
        onError={(e) => {
          // Show placeholder when stream is unavailable
          (e.target as HTMLImageElement).style.display = 'none';
        }}
      />
      <noscript>Camera feed requires JavaScript</noscript>
    </div>
  );
}
```

**Step 2: Commit**

```bash
git add frontend/src/components/CameraView.tsx
git commit -m "feat(frontend): add CameraView MJPEG stream component"
```

---

## Task 11: Frontend — TeleopPad Component

**Files:**
- Create: `frontend/src/components/TeleopPad.tsx`

**Step 1: Write TeleopPad.tsx**

```tsx
import { useEffect, useCallback } from 'react';
import { publishTwist, publishStop } from '../lib/ros';

// TurtleBot3 Burger limits
const MAX_LINEAR = 0.22; // m/s
const MAX_ANGULAR = 2.84; // rad/s

interface TeleopPadProps {
  speedScale: number; // 0.0 - 1.0
}

export function TeleopPad({ speedScale }: TeleopPadProps) {
  const linear = MAX_LINEAR * speedScale;
  const angular = MAX_ANGULAR * speedScale;

  const handleKey = useCallback(
    (e: KeyboardEvent) => {
      // Prevent page scroll on arrow keys
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(e.key)) {
        e.preventDefault();
      }

      switch (e.key) {
        case 'w': case 'ArrowUp':
          publishTwist(linear, 0); break;
        case 's': case 'ArrowDown':
          publishTwist(-linear, 0); break;
        case 'a': case 'ArrowLeft':
          publishTwist(0, angular); break;
        case 'd': case 'ArrowRight':
          publishTwist(0, -angular); break;
        case ' ':
          publishStop(); break;
      }
    },
    [linear, angular]
  );

  const handleKeyUp = useCallback(
    (e: KeyboardEvent) => {
      if (['w', 's', 'a', 'd', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) {
        publishStop();
      }
    },
    []
  );

  useEffect(() => {
    window.addEventListener('keydown', handleKey);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKey);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [handleKey, handleKeyUp]);

  const btnStyle = {
    width: '60px',
    height: '60px',
    fontSize: '24px',
    cursor: 'pointer',
    border: '1px solid #555',
    borderRadius: '8px',
    background: '#2a2a2a',
    color: '#fff',
  };

  return (
    <div>
      <h3 style={{ margin: '0 0 8px' }}>Teleop Controls</h3>
      <p style={{ fontSize: '12px', color: '#888', margin: '0 0 12px' }}>
        WASD or Arrow Keys &middot; Space = Stop
      </p>
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 60px)', gap: '4px', justifyContent: 'center' }}>
        <div />
        <button style={btnStyle} onMouseDown={() => publishTwist(linear, 0)} onMouseUp={publishStop}>&#9650;</button>
        <div />
        <button style={btnStyle} onMouseDown={() => publishTwist(0, angular)} onMouseUp={publishStop}>&#9664;</button>
        <button style={{ ...btnStyle, background: '#c0392b' }} onClick={publishStop}>&#9632;</button>
        <button style={btnStyle} onMouseDown={() => publishTwist(0, -angular)} onMouseUp={publishStop}>&#9654;</button>
        <div />
        <button style={btnStyle} onMouseDown={() => publishTwist(-linear, 0)} onMouseUp={publishStop}>&#9660;</button>
        <div />
      </div>
    </div>
  );
}
```

**Step 2: Commit**

```bash
git add frontend/src/components/TeleopPad.tsx
git commit -m "feat(frontend): add TeleopPad with keyboard and button controls"
```

---

## Task 12: Frontend — SpeedSlider Component

**Files:**
- Create: `frontend/src/components/SpeedSlider.tsx`

**Step 1: Write SpeedSlider.tsx**

```tsx
interface SpeedSliderProps {
  value: number;
  onChange: (value: number) => void;
}

export function SpeedSlider({ value, onChange }: SpeedSliderProps) {
  return (
    <div style={{ margin: '16px 0' }}>
      <label style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
        <span>Speed:</span>
        <input
          type="range"
          min={0.05}
          max={1.0}
          step={0.05}
          value={value}
          onChange={(e) => onChange(parseFloat(e.target.value))}
          style={{ flex: 1 }}
        />
        <span style={{ fontFamily: 'monospace', minWidth: '60px' }}>
          {(value * 0.22).toFixed(2)} m/s
        </span>
      </label>
    </div>
  );
}
```

**Step 2: Commit**

```bash
git add frontend/src/components/SpeedSlider.tsx
git commit -m "feat(frontend): add SpeedSlider component"
```

---

## Task 13: Frontend — NavGoal Component

**Files:**
- Create: `frontend/src/components/NavGoal.tsx`

**Step 1: Write NavGoal.tsx**

```tsx
import { useState } from 'react';
import { goalPoseTopic } from '../lib/ros';
import ROSLIB from 'roslib';

export function NavGoal() {
  const [x, setX] = useState('0.0');
  const [y, setY] = useState('0.0');

  function sendGoal() {
    const goal = new ROSLIB.Message({
      header: {
        frame_id: 'map',
        stamp: { sec: 0, nanosec: 0 },
      },
      pose: {
        position: { x: parseFloat(x) || 0, y: parseFloat(y) || 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
    });
    goalPoseTopic.publish(goal);
  }

  const inputStyle = {
    width: '70px',
    padding: '4px 8px',
    background: '#2a2a2a',
    border: '1px solid #555',
    borderRadius: '4px',
    color: '#fff',
    fontFamily: 'monospace',
  };

  return (
    <div style={{ margin: '16px 0' }}>
      <h3 style={{ margin: '0 0 8px' }}>Nav2 Waypoint</h3>
      <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
        <label>
          X: <input style={inputStyle} value={x} onChange={(e) => setX(e.target.value)} />
        </label>
        <label>
          Y: <input style={inputStyle} value={y} onChange={(e) => setY(e.target.value)} />
        </label>
        <button
          onClick={sendGoal}
          style={{
            padding: '4px 16px',
            background: '#2980b9',
            color: '#fff',
            border: 'none',
            borderRadius: '4px',
            cursor: 'pointer',
          }}
        >
          Send Goal
        </button>
      </div>
    </div>
  );
}
```

**Step 2: Commit**

```bash
git add frontend/src/components/NavGoal.tsx
git commit -m "feat(frontend): add NavGoal waypoint input component"
```

---

## Task 14: Frontend — StatusBar Component

**Files:**
- Create: `frontend/src/components/StatusBar.tsx`

**Step 1: Write StatusBar.tsx**

```tsx
import { useEffect, useState } from 'react';
import { ros, odomTopic } from '../lib/ros';

interface OdomState {
  x: number;
  y: number;
  theta: number;
  linearVel: number;
  angularVel: number;
}

export function StatusBar() {
  const [connected, setConnected] = useState(false);
  const [odom, setOdom] = useState<OdomState>({
    x: 0, y: 0, theta: 0, linearVel: 0, angularVel: 0,
  });

  useEffect(() => {
    const onConnect = () => setConnected(true);
    const onClose = () => setConnected(false);
    ros.on('connection', onConnect);
    ros.on('close', onClose);

    // Check current state
    if ((ros as any).isConnected) setConnected(true);

    return () => {
      ros.off('connection', onConnect);
      ros.off('close', onClose);
    };
  }, []);

  useEffect(() => {
    const callback = (msg: any) => {
      const pos = msg.pose.pose.position;
      const orient = msg.pose.pose.orientation;
      // Convert quaternion to yaw (theta)
      const siny = 2.0 * (orient.w * orient.z + orient.x * orient.y);
      const cosy = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z);
      const theta = Math.atan2(siny, cosy);

      setOdom({
        x: pos.x,
        y: pos.y,
        theta,
        linearVel: msg.twist.twist.linear.x,
        angularVel: msg.twist.twist.angular.z,
      });
    };

    odomTopic.subscribe(callback);
    return () => odomTopic.unsubscribe(callback);
  }, []);

  const statusColor = connected ? '#2ecc71' : '#e74c3c';
  const statusText = connected ? 'Connected' : 'Disconnected';

  return (
    <div
      style={{
        display: 'flex',
        justifyContent: 'space-between',
        padding: '8px 16px',
        background: '#1a1a1a',
        borderTop: '1px solid #333',
        fontSize: '13px',
        fontFamily: 'monospace',
        color: '#ccc',
      }}
    >
      <span>
        <span style={{ color: statusColor }}>{'\u25CF'}</span> {statusText}
      </span>
      <span>
        Pos: ({odom.x.toFixed(2)}, {odom.y.toFixed(2)}) &theta;: {(odom.theta * 180 / Math.PI).toFixed(1)}&deg;
      </span>
      <span>
        Lin: {odom.linearVel.toFixed(2)} m/s &middot; Ang: {odom.angularVel.toFixed(2)} rad/s
      </span>
    </div>
  );
}
```

**Step 2: Commit**

```bash
git add frontend/src/components/StatusBar.tsx
git commit -m "feat(frontend): add StatusBar with connection status and odometry display"
```

---

## Task 15: Frontend — App Layout & Entry Point

**Files:**
- Create: `frontend/src/App.tsx`
- Create: `frontend/src/main.tsx`

**Step 1: Write App.tsx**

```tsx
import { useState } from 'react';
import { CameraView } from './components/CameraView';
import { TeleopPad } from './components/TeleopPad';
import { SpeedSlider } from './components/SpeedSlider';
import { NavGoal } from './components/NavGoal';
import { StatusBar } from './components/StatusBar';

export function App() {
  const [speedScale, setSpeedScale] = useState(0.45); // ~0.1 m/s default

  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      height: '100vh',
      background: '#111',
      color: '#eee',
      fontFamily: 'system-ui, sans-serif',
    }}>
      {/* Header */}
      <header style={{
        padding: '12px 16px',
        background: '#1a1a1a',
        borderBottom: '1px solid #333',
        fontSize: '18px',
        fontWeight: 600,
      }}>
        Isaac Sim Remote Control
      </header>

      {/* Main content */}
      <main style={{
        display: 'flex',
        flex: 1,
        overflow: 'hidden',
      }}>
        {/* Left: Camera feed */}
        <div style={{ flex: 2, display: 'flex', alignItems: 'center', justifyContent: 'center', padding: '16px' }}>
          <CameraView />
        </div>

        {/* Right: Controls panel */}
        <div style={{
          flex: 1,
          padding: '16px',
          borderLeft: '1px solid #333',
          overflowY: 'auto',
          minWidth: '280px',
        }}>
          <TeleopPad speedScale={speedScale} />
          <SpeedSlider value={speedScale} onChange={setSpeedScale} />
          <NavGoal />
        </div>
      </main>

      {/* Footer: Status bar */}
      <StatusBar />
    </div>
  );
}
```

**Step 2: Write main.tsx**

```tsx
import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import { App } from './App';

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>
);
```

**Step 3: Verify frontend builds locally**

Run:
```bash
cd frontend
npm run build
```

Expected: Build succeeds, `dist/` directory created with `index.html` + bundled JS.

**Step 4: Commit**

```bash
git add frontend/src/App.tsx frontend/src/main.tsx
git commit -m "feat(frontend): add App layout and entry point"
```

---

## Task 16: Integration — Deploy & Smoke Test on AWS

**Files:**
- No new files — this is deployment and validation

**Step 1: Deploy infrastructure with Terraform**

```bash
cd terraform
terraform init
terraform plan -var="key_name=YOUR_KEY" -var="allowed_ssh_cidr=YOUR_IP/32"
# Review the plan, then:
terraform apply -var="key_name=YOUR_KEY" -var="allowed_ssh_cidr=YOUR_IP/32"
```

Expected: EC2 instance created, Elastic IP assigned, outputs printed.

**Step 2: Wait for bootstrap to complete, then SSH in**

```bash
ssh -i ~/.ssh/YOUR_KEY.pem ubuntu@<ELASTIC_IP>
# Check bootstrap log:
tail -f /var/log/user-data.log
# Wait until "Bootstrap complete" appears
```

**Step 3: Clone project and start Docker Compose**

```bash
cd ~/isaacsim_aws/docker
docker compose pull
docker compose up -d
# Watch logs:
docker compose logs -f
```

**Step 4: Verify each service**

```bash
# Check Isaac Sim is running:
docker compose logs isaac-sim | grep "Simulation running"

# Check rosbridge is up:
docker compose logs ros2 | grep "Rosbridge WebSocket server started"

# Check web_video_server is up:
docker compose logs ros2 | grep "web_video_server"

# Check nginx is serving:
curl -s http://localhost/ | head -5
```

**Step 5: Open browser and test**

- Navigate to `http://<ELASTIC_IP>`
- Verify: camera feed visible, D-pad buttons work, WASD keys move the robot, odometry updates in status bar.

**Step 6: Commit any fixes found during integration**

```bash
git add -A
git commit -m "fix: integration adjustments from smoke test"
```

---

## Task 17: Add Nav2 Support (Follow-Up)

> **Prerequisite:** Tasks 1-16 complete and basic teleop works end-to-end.

**Files:**
- Modify: `docker/ros2/launch/bringup.launch.py`

**Step 1: Add Nav2 bringup to launch file**

Add after the existing nodes in `bringup.launch.py`:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# Add Nav2 bringup (include in the LaunchDescription list)
nav2_bringup_dir = get_package_share_directory('nav2_bringup')
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    ),
    launch_arguments={
        'use_sim_time': 'true',
    }.items(),
)
```

**Step 2: Rebuild and test**

```bash
cd docker
docker compose up -d --build ros2
docker compose logs -f ros2
```

Expected: Nav2 nodes start, `/goal_pose` topic accepts goals from the browser.

**Step 3: Test from browser**

Enter X/Y coordinates in the Nav2 Waypoint panel and click "Send Goal". Robot should navigate autonomously.

**Step 4: Commit**

```bash
git add docker/ros2/launch/bringup.launch.py
git commit -m "feat(ros2): add Nav2 bringup for autonomous waypoint navigation"
```

---

## Summary

| Task | Component | Estimated Complexity |
|------|-----------|---------------------|
| 1 | Git init + skeleton | Trivial |
| 2 | Terraform variables + provider | Small |
| 3 | Bootstrap script | Small |
| 4 | Terraform EC2 + SG + EIP | Medium |
| 5 | Docker Compose | Medium |
| 6 | ROS2 Dockerfile + launch | Small |
| 7 | Isaac Sim startup script | Large (needs on-device validation) |
| 8 | Nginx config | Small |
| 9 | Frontend project setup + ros.ts | Medium |
| 10 | CameraView component | Small |
| 11 | TeleopPad component | Medium |
| 12 | SpeedSlider component | Trivial |
| 13 | NavGoal component | Small |
| 14 | StatusBar component | Small |
| 15 | App layout + entry point | Small |
| 16 | Deploy + smoke test on AWS | Large (integration) |
| 17 | Nav2 follow-up | Medium |

**Critical path:** Tasks 1→7 (Isaac Sim startup script) is the riskiest — OmniGraph API may need adjustments on the real container. Plan for iteration time during Task 16.
