# Project Status — Isaac Sim AWS Remote Control

**Last updated:** 2026-02-19
**Repo:** https://github.com/mokcontoro/isaacsim-aws

---

## What's Done (Tasks 1-15 + Review Fixes)

All code is written, reviewed, and committed. 14 commits on `master`.

### Completed Components

| Component | Key Files | Status |
|-----------|-----------|--------|
| Git repo + skeleton | `.gitignore`, directory structure | Done |
| Terraform (infra) | `terraform/main.tf`, `variables.tf`, `outputs.tf`, `scripts/bootstrap.sh` | Done |
| Docker Compose | `docker/docker-compose.yml` (4 services) | Done |
| Isaac Sim container | `docker/isaac-sim/Dockerfile`, `startup.py` | Done |
| ROS2 container | `docker/ros2/Dockerfile`, `launch/bringup.launch.py` | Done |
| Nginx reverse proxy | `docker/nginx/nginx.conf` | Done |
| Frontend (React) | `frontend/src/` — App, CameraView, TeleopPad, SpeedSlider, NavGoal, StatusBar, ros.ts | Done |
| Code review fixes | OmniGraph wiring, camera topic, race condition, key repeat, unused imports | Done |

### Review Findings Already Fixed (commit `8a38f7c`)

- **C1:** Added missing `docker/isaac-sim/Dockerfile`, updated docker-compose to use `build:`
- **C2:** Connected Twist subscriber output to DiffDriveController, added wheel params, execution tick
- **C3:** Fixed camera topic mismatch (`/camera/image` used consistently)
- **I3:** Added `depends_on: frontend-build` to nginx service
- **I4/I5:** Removed unused imports (`argparse`, `set_prim_attribute_value`, `set_prim_transform`)
- **I7:** Added `if (e.repeat) return;` to TeleopPad keydown handler

---

## What Remains

### Task 16: Deploy & Smoke Test on AWS

**AWS setup already completed (on yunyo's work machine):**

- AWS CLI installed
- Terraform installed (path: `C:\Users\yunyo\AppData\Local\Microsoft\WinGet\Packages\Hashicorp.Terraform_Microsoft.Winget.Source_8wekyb3d8bbwe\terraform.exe`)
- AWS credentials configured for `isaacsim-admin` user
- SSH key pair created: `isaacsim-key` (private key at `~/.ssh/isaacsim-key.pem`)
- Security group created on AWS: `sg-0c60813e47c2e7ec4`
- **BLOCKED: GPU quota (G and VT instances) is 0 vCPUs. Request pending (ID: a213920cc34047fe88a87fcbc2414ccfQ2iJnSmP)**
- Check quota status: `aws service-quotas get-requested-service-quota-change --request-id a213920cc34047fe88a87fcbc2414ccfQ2iJnSmP`

**If deploying from a different machine, you need:**

1. Install AWS CLI: `winget install Amazon.AWSCLI`
2. Install Terraform: `winget install Hashicorp.Terraform`
3. Configure AWS credentials: `aws configure` (region: `us-east-1`)
4. Create SSH key pair:
   ```bash
   aws ec2 create-key-pair --key-name isaacsim-key --query 'KeyMaterial' --output text > ~/.ssh/isaacsim-key.pem
   ```

**Deployment steps:**

```bash
cd terraform
terraform init
terraform apply -var="key_name=isaacsim-key" -var="allowed_ssh_cidr=YOUR_IP/32"
# Note the public IP from output

ssh -i ~/.ssh/isaacsim-key.pem ubuntu@<PUBLIC_IP>
# Wait for bootstrap: tail -f /var/log/user-data.log (until "Bootstrap complete")

git clone https://github.com/mokcontoro/isaacsim-aws.git
cd isaacsim-aws/docker
docker compose up -d
docker compose logs -f
```

**Verification:**
- `http://<PUBLIC_IP>` should show the web UI
- Camera feed should display warehouse scene
- WASD keys should move the TurtleBot3 Burger
- Status bar should show odometry updating

**Known risk:** The Isaac Sim OmniGraph node names in `startup.py` may need adjustment on the actual Isaac Sim 4.2 container. Plan for ~1 hour of iteration. Common issues:
- Node type names may differ (check `omni.graph.core` docs inside container)
- Camera/lidar prim paths may need to reference specific prims on the Burger USD model
- DiffDriveController may need explicit joint name configuration

### Task 17: Add Nav2 Support (Follow-Up)

After basic teleop works, add Nav2 to the ROS2 launch file for autonomous waypoint navigation. See implementation plan Task 17 for details.

---

## Key Architecture Decisions

- **Isaac Sim 4.2** (not latest) — chosen for stability and documentation
- **web_video_server** for MJPEG streaming — balanced latency vs. complexity
- **Single Docker Compose stack** — simplest for a demo
- **Cyclone DDS** — better cross-container discovery than FastDDS
- **Nginx as single entry point** — browser connects to one port only
- **TurtleBot3 Burger** — specific model with 0.22 m/s max linear, 2.84 rad/s max angular

## Cost Warning

g5.xlarge costs ~$1.01/hr on-demand. Always tear down when done:
```bash
cd terraform
terraform destroy -var="key_name=isaacsim-key" -var="allowed_ssh_cidr=YOUR_IP/32"
```

## Reference Documents

- Design doc: `docs/plans/2026-02-19-isaacsim-aws-design.md`
- Full implementation plan: `docs/plans/2026-02-19-isaacsim-aws-implementation.md`
