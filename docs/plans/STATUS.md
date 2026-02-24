# Project Status — Isaac Sim AWS Remote Control

**Last updated:** 2026-02-24
**Repo:** https://github.com/mokcontoro/isaacsim-aws
**EC2 IP (current session):** 3.217.125.75 (Elastic IP — survives reboots, destroyed with `terraform destroy`)

---

## Current State: Isaac Sim 5.0 — Deployed & Verified

Upgraded from Isaac Sim 4.2 to 5.0 with host networking. All ROS2 topics verified working on EC2. WebRTC signaling server runs on port 49100 but no browser client is available (NVIDIA's WebRTC HTML client isn't bundled in the 5.0 headless container). Frontend uses MJPEG dual-camera view (chase cam + bird's eye PiP).

### Architecture

```
Browser → nginx:80
  ├── /           → React SPA (static files)
  ├── /ws         → rosbridge WebSocket (localhost:9090)
  └── /video/     → web_video_server MJPEG (localhost:8080)
All containers use host networking + ipc:host (required for DDS shared memory)
```

### Verified Working (2026-02-24)
- `/odom` publishing at ~25.6 Hz
- `/camera/image` publishing at ~19 Hz (640x480 chase cam)
- `/camera/birdseye` publishing at ~19 Hz (320x320 bird's eye)
- `/cmd_vel` subscriber receives commands, robot moves
- MJPEG streaming through nginx on port 80
- Robot movement confirmed via odometry after cmd_vel publish

### What Changed (v0.3.0)
- **Isaac Sim 5.0**: Base image `nvcr.io/nvidia/isaac-sim:5.0.0`, new extension namespaces
- **Host networking + ipc:host**: All containers use `network_mode: host` + `ipc: host` (required for FastDDS shared memory transport)
- **5.0 API changes**: Extension namespaces (`isaacsim.ros2.bridge`, `isaacsim.core.nodes`, etc.), `usdrt.Sdf.Path` for rel-type OmniGraph attributes, new TurtleBot3 asset path
- **Volume mounts**: Shader cache, asset data, and logs persisted across container restarts
- **Frontend**: Dual MJPEG cameras (chase cam main + bird's eye PiP)
- **Security group**: WebRTC ports 49100/tcp, 47998/udp, 8211/tcp opened
- **start.sh**: New helper script auto-detects PUBLIC_IP via EC2 metadata

### Issues Found & Fixed During Deploy
1. **`omni.services.streamclient.webrtc` not bundled** — Extension doesn't exist in 5.0 headless. Removed.
2. **TurtleBot3 asset path changed** — `Turtlebot/turtlebot3_burger.usd` → `Turtlebot/Turtlebot3/turtlebot3_burger.usd`
3. **`ROS_DISTRO` env var needed** — ROS2 bridge extension requires `ENV ROS_DISTRO=humble`
4. **`chassisPrim` is rel type** — Must use `[usdrt.Sdf.Path(...)]` not plain string
5. **`ArticulationController.inputs:targetPrim` doesn't exist at runtime** — Use `robotPath` (string) instead
6. **DDS shared memory transport** — `ipc: host` required for FastDDS SHM between containers
7. **No WebRTC HTML client** — NVIDIA's `@nvidia/omniverse-webrtc-streaming-library` is on a private registry; reverted to MJPEG dual-camera view

---

## Completed Tasks

### v0.1.0–v0.2.0 (Isaac Sim 4.2)
All code written, reviewed, committed. 20+ commits on `master`.

| Component | Key Files | Status |
|-----------|-----------|--------|
| Git repo + skeleton | `.gitignore`, directory structure | Done |
| Terraform (infra) | `terraform/main.tf`, `variables.tf`, `outputs.tf`, `scripts/bootstrap.sh` | Done |
| Docker Compose | `docker/docker-compose.yml` | Done |
| Isaac Sim container | `docker/isaac-sim/Dockerfile`, `startup.py` | Done |
| ROS2 container | `docker/ros2/Dockerfile`, `launch/bringup.launch.py` | Done |
| Nginx reverse proxy | `docker/nginx/nginx.conf` | Done |
| Frontend (React) | `frontend/src/` — App, CameraView, TeleopPad, SpeedSlider, NavGoal, StatusBar, ros.ts | Done |

### v0.3.0 (Isaac Sim 5.0 — Deployed & Verified)

| File | Change |
|------|--------|
| `docker/isaac-sim/Dockerfile` | Base image 5.0, WebRTC + ROS2 env vars, ports |
| `docker/isaac-sim/startup.py` | 5.0 APIs, usdrt.Sdf.Path for rel attrs, correct asset path |
| `docker/docker-compose.yml` | Host networking, ipc:host, volume mounts for caching |
| `docker/nginx/nginx.conf` | localhost upstreams, /streaming/ proxy |
| `docker/start.sh` | New — PUBLIC_IP auto-detection helper |
| `terraform/main.tf` | WebRTC port rules in security group |
| `scripts/ec2-start.sh` | Pass PUBLIC_IP to docker compose |
| `frontend/src/components/CameraView.tsx` | Dual MJPEG cameras (chase + bird's eye PiP) |
| `frontend/src/App.tsx` | Version bump to v0.3.0 |
| `frontend/vite.config.ts` | /streaming proxy for local dev |
| `frontend/src/lib/ros.ts` | birdseyeStreamUrl for bird's eye camera |

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

### Start containers (~15 min first time, ~5 min with cached volumes)
```bash
cd /home/ubuntu/isaacsim-aws/docker
./start.sh   # auto-detects PUBLIC_IP, runs docker compose up -d --build
docker compose logs -f
```

Or via the convenience script from your local machine:
```bash
./scripts/ec2-start.sh   # starts instance + containers with PUBLIC_IP
```

### Verify
```bash
# Check all containers are running
docker ps

# Check Isaac Sim is ready (look for scene ready in Kit log)
docker exec isaac-sim grep 'scene ready' '/isaac-sim/kit/logs/Kit/Isaac-Sim Python/5.0/'kit_*.log

# Check ROS2 topics are publishing
docker exec ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic hz /odom"
docker exec ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic hz /camera/image"

# Test robot movement
docker exec ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'"
```

### Browse
Open `http://<IP>` in Chrome/Edge:
- Main camera view (chase cam following robot)
- Bird's eye PiP overlay (top-right corner)
- WASD teleop controls to drive the robot
- Speed slider to adjust velocity
- Status bar shows rosbridge connection + odometry

### Tear down (IMPORTANT — $1.01/hr)
```bash
cd terraform
terraform destroy -var="key_name=isaacsim-key" -var="allowed_ssh_cidr=$(curl -s ifconfig.me)/32"
```

---

## Key Architecture Decisions

- **Isaac Sim 5.0** — improved ROS2 bridge, WebRTC signaling server (no HTML client in headless)
- **Host networking + ipc:host** — required for DDS shared memory transport between containers
- **FastDDS** (not Cyclone) — only DDS available in Isaac Sim's internal ROS2 bridge
- **MJPEG dual cameras** — chase cam (640x480) + bird's eye PiP (320x320)
- **Single Docker Compose** — simplest for demo
- **Nginx single entry point** — browser connects to one port (80)
- **TurtleBot3 Burger** — 0.22 m/s max linear, 2.84 rad/s max angular
- **Volume mounts** — shader cache + asset data persisted to speed up restarts

## Cost Warning

g5.xlarge costs ~$1.01/hr on-demand. **Always `terraform destroy` when done.**

## Reference Documents

- Design: `docs/plans/2026-02-19-isaacsim-aws-design.md`
- Implementation plan: `docs/plans/2026-02-19-isaacsim-aws-implementation.md`
- Isaac Sim gotchas: Claude memory at `~/.claude/projects/.../memory/isaac-sim-gotchas.md`

## Future Work

### WebRTC Interactive 3D Viewer
The WebRTC signaling server runs on port 49100 but NVIDIA's HTML client (`omni.services.livestream.webrtc`) isn't bundled in the 5.0 headless container and the npm library (`@nvidia/omniverse-webrtc-streaming-library`) is on a private registry. Options:
1. Build a custom WebRTC client using the signaling protocol
2. Wait for NVIDIA to publish the npm library publicly
3. Extract the client from a non-headless Isaac Sim installation

### Task 17: Nav2 Autonomous Navigation (Pending)
Add Nav2 to the ROS2 launch for autonomous waypoint navigation. The frontend NavGoal component is already built and ready.

### TURN Server (If Needed)
Only add if WebRTC fails from restrictive networks. See Phase 6 in the upgrade plan.
