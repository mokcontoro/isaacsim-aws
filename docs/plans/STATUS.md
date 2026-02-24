# Project Status — Isaac Sim AWS Remote Control

**Last updated:** 2026-02-23
**Repo:** https://github.com/mokcontoro/isaacsim-aws
**EC2 IP (current session):** 3.217.125.75 (Elastic IP — survives reboots, destroyed with `terraform destroy`)

---

## Current State: Camera UX Complete, Pre-Deploy Fixes Pending

The system deploys and runs on AWS g5.xlarge. Camera feed streams to the browser with third-person chase view and bird's eye picture-in-picture overlay. Robot responds to cmd_vel. The web UI loads but the rosbridge WebSocket shows "Disconnected" due to a DDS mismatch (see Remaining Fixes below). The nginx proxy path fix has been applied locally.

### What Works (verified on EC2)
- Isaac Sim 4.2 headless with TurtleBot3 Burger in warehouse scene
- OmniGraph ROS2 pipeline: cmd_vel → DifferentialController → ArticulationController → wheel joints
- Camera streaming at 640x480 via ROS2CameraHelper → web_video_server → nginx `/video/`
- Third-person chase camera attached to robot
- Bird's eye camera with picture-in-picture overlay (`/camera/birdseye`)
- Odometry publishing via IsaacComputeOdometry → ROS2PublishOdometry
- Robot movement verified: sent 0.1 m/s for 3s, robot moved to x=0.311
- Frontend loads in browser with camera feed, version display in header

### What's Broken
1. **Rosbridge WebSocket "Disconnected"** — DDS mismatch: ROS2 container uses `rmw_cyclonedds_cpp`, Isaac Sim uses `rmw_fastrtps_cpp`. Topics won't be visible across containers. **FIX NEEDED**: remove Cyclone DDS from ROS2 Dockerfile (see below).

---

## Remaining Fixes (before next deployment)

### Fix 1: ROS2 Dockerfile — Remove Cyclone DDS (**CRITICAL**)
File: `docker/ros2/Dockerfile`

The ROS2 container still has `ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. Isaac Sim's internal ROS2 bridge ONLY supports FastDDS. Both containers MUST use the same DDS implementation or they cannot see each other's topics.

**Change:**
```dockerfile
# REMOVE this line:
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Optionally REMOVE from apt-get (saves image size):
ros-humble-rmw-cyclonedds-cpp
```

FastDDS is the default for ROS2 Humble, so no explicit ENV is needed in the ROS2 container.

### Fix 2: nginx WebSocket proxy path (**DONE locally**)
File: `docker/nginx/nginx.conf` line 37

Changed `proxy_pass http://rosbridge;` → `proxy_pass http://rosbridge/;`

The trailing `/` tells nginx to replace the matched `/ws` prefix with `/`, so rosbridge receives `GET /` instead of `GET /ws`.

### Fix 3: bootstrap.sh — gpg --dearmor needs --batch
File: `terraform/scripts/bootstrap.sh` lines 22, 29

On non-interactive Ubuntu (cloud-init), `gpg --dearmor` hangs without `--batch` and also fails if the output file already exists. Fix both occurrences:

```bash
# Line 22:
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --batch --yes --dearmor -o /etc/apt/keyrings/docker.gpg

# Line 29:
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --batch --yes --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
```

### Fix 4: bootstrap.sh — git clone directory name
File: `terraform/scripts/bootstrap.sh` line 47

The bootstrap clones to `isaacsim_aws` (underscore) but the actual repo name is `isaacsim-aws` (hyphen). When SSH'ing in, the project is at `/home/ubuntu/isaacsim-aws/`. Update the clone target to match:

```bash
git clone "${PROJECT_REPO_URL}" isaacsim-aws
```

---

## Completed Tasks (1-15 + Review Fixes + Deployment Iteration)

All code written, reviewed, committed. 20+ commits on `master`.

### Component Status

| Component | Key Files | Status |
|-----------|-----------|--------|
| Git repo + skeleton | `.gitignore`, directory structure | Done |
| Terraform (infra) | `terraform/main.tf`, `variables.tf`, `outputs.tf`, `scripts/bootstrap.sh` | Done (needs Fix 3 & 4) |
| Docker Compose | `docker/docker-compose.yml` (4 services) | Done |
| Isaac Sim container | `docker/isaac-sim/Dockerfile`, `startup.py` | Done (battle-tested) |
| ROS2 container | `docker/ros2/Dockerfile`, `launch/bringup.launch.py` | Done (needs Fix 1) |
| Nginx reverse proxy | `docker/nginx/nginx.conf` | Done (Fix 2 applied) |
| Frontend (React) | `frontend/src/` — App, CameraView, TeleopPad, SpeedSlider, NavGoal, StatusBar, ros.ts | Done |

### Commits Made During Deployment (Task 16)

1. `fix: resolve Isaac Sim startup crashes (DDS + TransformPrim)`
2. `fix: set LD_LIBRARY_PATH for Isaac Sim internal ROS2 bridge`
3. `fix: correct OmniGraph attributes and type mismatches in startup.py`
4. `fix: use correct BreakVector3 node type (not BreakVector3Double)`
5. `fix: add world.play() and unbuffered stdout for Isaac Sim`
6. `fix: add ArticulationController to apply wheel velocities to robot`

### Camera UX Improvements (2026-02-23)

7. `feat: attach camera to robot for third-person chase view`
8. `feat: add bird's eye camera with picture-in-picture overlay`
9. `fix: bird's eye PiP loading + add version to header`
10. `fix: bird's eye camera rotation + fill camera view area`

---

## Deployment Playbook

### Prerequisites (one-time setup)
1. AWS CLI: `winget install Amazon.AWSCLI`
2. Terraform: `winget install Hashicorp.Terraform`
3. `aws configure` (region: `us-east-1`)
4. SSH key pair:
   ```bash
   aws ec2 create-key-pair --key-name isaacsim-key --query 'KeyMaterial' --output text > ~/.ssh/isaacsim-key.pem
   chmod 400 ~/.ssh/isaacsim-key.pem
   ```

### Deploy
```bash
cd terraform
terraform init
terraform apply -var="key_name=isaacsim-key" -var="allowed_ssh_cidr=$(curl -s ifconfig.me)/32"
# Note the public_ip from output
```

### Wait for bootstrap (~5 min)
```bash
ssh -i ~/.ssh/isaacsim-key.pem ubuntu@<IP>
tail -f /var/log/user-data.log
# Wait for "Bootstrap complete"
```

### Start containers (~15 min first time)
```bash
cd /home/ubuntu/isaacsim-aws/docker
docker compose up -d --build
docker compose logs -f
```

**Timing breakdown (first run):**
- Isaac Sim image pull: ~5 min (27GB)
- ROS2 image build: ~3 min
- Frontend build: ~1 min
- Isaac Sim extension loading: ~2 min
- RTX shader compilation: ~5 min
- Asset download from S3: ~5 min

**Timing (subsequent runs, images cached):**
- Container start: ~30s
- Extension loading: ~2 min
- Shader compilation: ~5 min (no cache across recreations)

### Verify
```bash
# Check all containers are running
docker ps

# Check Isaac Sim is ready
docker logs isaac-sim 2>&1 | grep "scene ready"

# Check ROS2 topics
docker exec ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

# Expected topics: /cmd_vel, /odom, /camera/image, /rosout, /parameter_events

# Test robot movement
docker exec ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'"
```

### Browse
Open `http://<IP>` (NOT https — no TLS configured)

### Tear down (IMPORTANT — $1.01/hr)
```bash
cd terraform
terraform destroy -var="key_name=isaacsim-key" -var="allowed_ssh_cidr=$(curl -s ifconfig.me)/32"
```

---

## Architecture

```
Browser ──HTTP──► nginx:80
                   ├── /        → static frontend (React)
                   ├── /ws      → rosbridge WebSocket (ros2:9090)
                   └── /video/  → web_video_server MJPEG (ros2:8080)

isaac-sim container (GPU)          ros2 container
  ├── TurtleBot3 Burger              ├── rosbridge_websocket :9090
  ├── Warehouse environment          └── web_video_server :8080
  ├── OmniGraph ROS2 pipeline
  │   ├── ROS2SubscribeTwist (/cmd_vel)
  │   ├── BreakVector3 (x2, type conversion)
  │   ├── DifferentialController
  │   ├── ArticulationController
  │   ├── IsaacComputeOdometry
  │   ├── ROS2PublishOdometry (/odom)
  │   ├── ROS2CameraHelper (/camera/image)
  │   └── ROS2CameraHelper (/camera/birdseye)
  └── FastDDS (RMW_IMPLEMENTATION)

Both containers on Docker network "rosnet", communicate via FastDDS multicast.
```

## Key Architecture Decisions

- **Isaac Sim 4.2** (not latest) — stability and documentation
- **FastDDS** (not Cyclone) — only DDS available in Isaac Sim 4.2 container
- **web_video_server** for MJPEG — balanced latency vs. complexity
- **Single Docker Compose** — simplest for demo
- **Nginx single entry point** — browser connects to one port (80)
- **TurtleBot3 Burger** — 0.22 m/s max linear, 2.84 rad/s max angular

## Cost Warning

g5.xlarge costs ~$1.01/hr on-demand. **Always `terraform destroy` when done.**

## Reference Documents

- Design: `docs/plans/2026-02-19-isaacsim-aws-design.md`
- Implementation plan: `docs/plans/2026-02-19-isaacsim-aws-implementation.md`
- Isaac Sim gotchas: Claude memory at `~/.claude/projects/.../memory/isaac-sim-gotchas.md`

## Future Work

### Task 17: Nav2 Autonomous Navigation (Pending)
After basic teleop is fully verified and pre-deploy fixes are applied, add Nav2 to the ROS2 launch for autonomous waypoint navigation. The frontend NavGoal component is already built and ready.

### Task 18: Interactive 3D Camera Viewer (Planned)
Replace the MJPEG camera stream with a WebRTC livestream from Isaac Sim for interactive camera controls (zoom, pan, orbit). This would provide a richer 3D viewing experience in the browser. Explored on 2026-02-23 and deferred — the current MJPEG approach is sufficient for teleop, and WebRTC adds significant complexity (signaling server, STUN/TURN, Isaac Sim rendering API integration).

### Pre-Deploy Fixes (Still Pending)
The 4 remaining fixes listed above (DDS, nginx trailing slash, gpg --batch, clone dir name) must be applied before the next AWS deployment.
