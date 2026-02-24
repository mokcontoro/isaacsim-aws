# Project Status — Isaac Sim AWS Remote Control

**Last updated:** 2026-02-23
**Repo:** https://github.com/mokcontoro/isaacsim-aws
**EC2 IP (current session):** 3.217.125.75 (Elastic IP — survives reboots, destroyed with `terraform destroy`)

---

## Current State: Isaac Sim 5.0 + WebRTC Upgrade (Untested)

Upgraded from Isaac Sim 4.2 to 5.0 with native WebRTC streaming for interactive 3D viewing. All containers switched to host networking for WebRTC UDP + DDS compatibility. Not yet deployed — needs first test on EC2.

### Architecture

```
Browser → nginx:80
  ├── /           → React SPA (static files)
  ├── /ws         → rosbridge WebSocket (localhost:9090)
  ├── /video/     → web_video_server MJPEG (localhost:8080)
  └── /streaming/ → Isaac Sim WebRTC client (localhost:8211)
Browser ←WebRTC UDP→ EC2:49100 (signaling) + EC2:47998 (media)
All containers use host networking (required for WebRTC UDP + DDS)
```

### What Changed (v0.3.0)
- **Isaac Sim 5.0**: Base image `nvcr.io/nvidia/isaac-sim:5.0.0`, new extension namespaces
- **WebRTC streaming**: LIVESTREAM=1, `omni.kit.livestream.webrtc` + `omni.services.streamclient.webrtc`
- **Host networking**: All containers use `network_mode: host` (no more Docker bridge)
- **Frontend**: WebRTC iframe as main view, MJPEG chase cam as PiP overlay, toggle button
- **Simplified cameras**: Bird's eye camera removed (WebRTC provides interactive god-view with orbit/pan/zoom)
- **Security group**: WebRTC ports 49100/tcp, 47998/udp, 8211/tcp opened
- **start.sh**: New helper script auto-detects PUBLIC_IP via EC2 metadata
- **Pre-deploy fixes applied**: DDS mismatch, nginx trailing slash, gpg --batch, clone dir name

### Risks to Watch
1. **OmniGraph node names**: 5.0 may use different type strings — same debug process as 4.2
2. **LD_LIBRARY_PATH**: ROS2 bridge path may differ in 5.0 container filesystem
3. **WebRTC iframe**: Should work since both served through nginx on same origin
4. **NVENC**: Verify GPU encoding works headless with `nvidia-smi`

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

### v0.3.0 (Isaac Sim 5.0 + WebRTC)

| File | Change |
|------|--------|
| `docker/isaac-sim/Dockerfile` | Base image 5.0, WebRTC env vars + ports |
| `docker/isaac-sim/startup.py` | 5.0 APIs, WebRTC streaming, single chase cam |
| `docker/docker-compose.yml` | Host networking, PUBLIC_IP env |
| `docker/nginx/nginx.conf` | localhost upstreams, /streaming/ proxy |
| `docker/start.sh` | New — PUBLIC_IP auto-detection helper |
| `terraform/main.tf` | WebRTC port rules in security group |
| `scripts/ec2-start.sh` | Pass PUBLIC_IP to docker compose |
| `frontend/src/components/CameraView.tsx` | WebRTC iframe + MJPEG PiP + toggle |
| `frontend/src/App.tsx` | Version bump to v0.3.0 |
| `frontend/vite.config.ts` | /streaming proxy for local dev |
| `frontend/src/lib/ros.ts` | Remove unused birdseyeStreamUrl |

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

# Check Isaac Sim is ready (look for WebRTC + scene ready)
docker logs isaac-sim 2>&1 | grep -E "WebRTC|scene ready"

# Check ROS2 topics
docker exec ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

# Test robot movement
docker exec ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'"

# Verify WebRTC signaling port
nc -zv <IP> 49100
```

### Browse
Open `http://<IP>` in Chrome/Edge:
- WebRTC 3D viewer loads in main area (orbit=left-click, pan=middle-click, zoom=scroll)
- MJPEG robot camera shows in PiP corner
- Toggle "Camera"/"3D View" button to switch views
- WASD teleop controls still work

### Tear down (IMPORTANT — $1.01/hr)
```bash
cd terraform
terraform destroy -var="key_name=isaacsim-key" -var="allowed_ssh_cidr=$(curl -s ifconfig.me)/32"
```

---

## Key Architecture Decisions

- **Isaac Sim 5.0** — native WebRTC streaming, improved ROS2 bridge
- **Host networking** — required for WebRTC UDP + DDS multicast
- **FastDDS** (not Cyclone) — only DDS available in Isaac Sim's internal ROS2 bridge
- **WebRTC (main) + MJPEG PiP** — interactive 3D god-view with robot chase cam overlay
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
After WebRTC viewer is verified, add Nav2 to the ROS2 launch for autonomous waypoint navigation. The frontend NavGoal component is already built and ready.

### TURN Server (If Needed)
Only add if WebRTC fails from restrictive networks. See Phase 6 in the upgrade plan.
