# Project Status — Isaac Sim AWS Remote Control

**Last updated:** 2026-02-25
**Repo:** https://github.com/mokcontoro/isaacsim-aws
**EC2 IP (current session):** 3.217.125.75 (Elastic IP — survives reboots, destroyed with `terraform destroy`)

---

## Current State: v0.5.0 — Isaac Sim 5.1 + WebRTC 3D Viewer

Fully working interactive 3D viewer via WebRTC + MJPEG robot chase cam PiP overlay. Teleop controls (WASD) move the robot. Single concurrent viewer limitation (NVIDIA WebRTC supports one client at a time).

### Architecture

```
Browser → nginx:80
  ├── /           → React SPA (static files)
  ├── /ws         → rosbridge WebSocket (localhost:9090)
  └── /video/     → web_video_server MJPEG (localhost:8080)
Browser ←WebRTC→ EC2:49100 (signaling) + EC2:47998/udp (media)
All containers use host networking (required for WebRTC UDP + DDS)
```

### Verified Working (2026-02-25)
- WebRTC interactive 3D viewer (orbit, pan, zoom) — single viewer at a time
- MJPEG robot chase cam as PiP overlay (auto-follows robot)
- `/odom` publishing, displayed in status bar
- `/cmd_vel` teleop via WASD keys and on-screen buttons
- Robot movement confirmed in both 3D viewer and odometry
- Security group opened: web/WebRTC ports to all IPs, SSH restricted

### What Changed (v0.5.0 from v0.3.0)

| File | Change |
|------|--------|
| `docker/isaac-sim/Dockerfile` | Base image 5.0 → 5.1.0 |
| `docker/isaac-sim/startup.py` | `_wait_for_viewport` monkey-patch, NVCF readiness flags, correct WebRTC settings paths |
| `docker/docker-compose.yml` | Removed `ipc: host` from isaac-sim, added `user: "0:0"`, updated volume paths for 5.1 |
| `terraform/main.tf` | Web/WebRTC ports open to `0.0.0.0/0`, SSH remains IP-restricted |
| `frontend/src/lib/ros.ts` | Fixed ROSLIB.Message not exported from CJS bundle — use plain object for publish |
| `frontend/src/components/IsaacSimViewer.tsx` | WebRTC viewer via `@nvidia/omniverse-webrtc-streaming-library` |
| `frontend/src/components/CameraView.tsx` | WebRTC main view + MJPEG PiP overlay + view toggle |
| `frontend/src/App.tsx` | Version bump to v0.5.0 |

### Issues Found & Fixed During Deploy

#### Isaac Sim 5.1 Upgrade
1. **`ipc: host` crashes 5.1** — `carb-RStringInternals` shared memory assertion failure. Removed from isaac-sim service.
2. **Volume permissions** — 5.1 runs as user 1234:1234, Docker volumes are root-owned. Fixed with `user: "0:0"`.
3. **Volume paths changed** — `/root/.cache/...` → `/isaac-sim/.cache/...` etc.
4. **`_wait_for_viewport()` still hangs** — Even 5.1's MAX_FRAMES fix doesn't help because `_app.update()` blocks. Monkey-patch: replace with 60 `_app.update()` calls without viewport_handle check.

#### WebRTC Streaming
5. **Wrong settings paths** — Must use `/app/livestream/publicEndpointAddress`, not `/exts/omni.kit.livestream.webrtc/publicIp`.
6. **NVCF "App not ready"** — Extension subscribes to `EVENT_APP_READY` but event fires before extension is enabled. Fix: manually set `_nvcf_api.app_ready = True`.
7. **NVCF "Rtx not ready"** — `NEW_FRAME` event doesn't fire in headless main loop. Fix: manually set `_nvcf_api.rtx_ready = True`.

#### Frontend
8. **ROSLIB.Message not exported** — roslib's CJS bundle doesn't export `Message` as named export. `new ROSLIB.Message()` throws TypeError at runtime. Fix: pass plain object to `topic.publish()`.

### Known Limitations
- **Single viewer** — NVIDIA WebRTC streaming supports only one concurrent client. Second viewer stays in "connecting" forever.
- **Shader compilation** — First cold start after container recreation takes ~10 min (CPU-bound shader compilation at ~300% CPU before GPU rendering begins).
- **PUBLIC_IP must be set** — `docker compose up` without `PUBLIC_IP` env var defaults to `127.0.0.1`, breaking WebRTC for remote clients. Always use `start.sh` or pass explicitly.

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
Upgrade from 4.2 to 5.0 with host networking, new extension namespaces, volume mounts.

### v0.5.0 (Isaac Sim 5.1 + WebRTC — Deployed & Verified)
Upgrade from 5.0 to 5.1 with WebRTC interactive 3D viewer, NVCF streaming fixes, teleop fix.

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

### Start containers (~10 min first time, ~3 min with cached volumes)
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

# Check Isaac Sim startup logs
docker logs isaac-sim 2>&1 | grep '\[startup\]'
# Look for: "=== Isaac Sim scene ready. WebRTC streaming on port 49100. ==="

# Check ROS2 topics
docker exec ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

# Test robot movement
docker exec ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'"
```

### Browse
Open `http://<IP>` in Chrome/Edge:
- **Main view:** Interactive 3D WebRTC viewer (orbit: ALT+drag, pan: middle-click, zoom: scroll)
- **PiP overlay:** MJPEG robot chase camera (top-right corner)
- **Toggle:** "Camera View" / "3D View" button to switch main view
- **Teleop:** WASD or arrow keys to drive, Space to stop
- **Status bar:** rosbridge connection + odometry data

### Pause (stop EC2 to save costs)
```bash
./scripts/ec2-stop.sh   # stops EC2 instance, ~$0.01/hr for EBS storage only
```

### Resume
```bash
./scripts/ec2-start.sh   # starts instance + containers with PUBLIC_IP
```

### Tear down (IMPORTANT — $1.01/hr when running)
```bash
cd terraform
terraform destroy -var="key_name=isaacsim-key" -var="allowed_ssh_cidr=$(curl -s ifconfig.me)/32"
```

---

## Key Architecture Decisions

- **Isaac Sim 5.1** — fixes viewport hang bug, improved headless mode, WebRTC streaming
- **WebRTC via NVCF extension** — `omni.services.livestream.nvcf` wraps `omni.kit.livestream.webrtc`
- **Host networking** — required for WebRTC UDP + DDS multicast
- **No `ipc: host` on isaac-sim** — causes 5.1 shared memory crash
- **FastDDS** (not Cyclone) — only DDS available in Isaac Sim's internal ROS2 bridge
- **Single Docker Compose** — simplest for demo
- **Nginx single entry point** — browser connects to one port (80)
- **TurtleBot3 Burger** — 0.22 m/s max linear, 2.84 rad/s max angular
- **Volume mounts** — shader cache + asset data persisted to speed up restarts
- **Security group** — SSH restricted to deployer IP, web/WebRTC open to all

## Cost Warning

g5.xlarge costs ~$1.01/hr on-demand. **Always stop or destroy when done.**
- Stop: `./scripts/ec2-stop.sh` (~$0.01/hr for EBS storage)
- Destroy: `terraform destroy` (no ongoing cost)

## Reference Documents

- Design: `docs/plans/2026-02-19-isaacsim-aws-design.md`
- Implementation plan: `docs/plans/2026-02-19-isaacsim-aws-implementation.md`
- Isaac Sim gotchas: Claude memory at `~/.claude/projects/.../memory/isaac-sim-gotchas.md`

## Future Work

### Nav2 Autonomous Navigation
Add Nav2 to the ROS2 launch for autonomous waypoint navigation. The frontend NavGoal component is already built and ready.

### Multi-Viewer Support
NVIDIA WebRTC supports only one client. Options for multi-viewer:
1. Screen sharing (Zoom/Discord) — simplest
2. Re-stream WebRTC via a media server (Janus, mediasoup)
3. Multiple render products + separate streams (heavy GPU usage)

### TURN Server
Only needed if WebRTC fails from restrictive networks (corporate firewalls blocking UDP). See Phase 6 in the upgrade plan.
