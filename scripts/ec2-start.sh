#!/bin/bash
# Start the Isaac Sim EC2 instance and restart Docker containers
# Resume time: ~2-5 min (instance boot + container startup + shader compile)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TF_DIR="$SCRIPT_DIR/../terraform"

# Find terraform binary (may not be in PATH on Windows)
TF_BIN=$(command -v terraform 2>/dev/null || echo "")
if [ -z "$TF_BIN" ]; then
  # WinGet default install location
  TF_BIN=$(ls /c/Users/*/AppData/Local/Microsoft/WinGet/Packages/Hashicorp.Terraform*/terraform.exe 2>/dev/null | head -1)
fi
if [ -z "$TF_BIN" ]; then
  echo "ERROR: terraform not found. Install it or add it to PATH."
  exit 1
fi

# Find AWS CLI binary (may not be in PATH on Windows/Git Bash)
AWS_BIN=$(command -v aws 2>/dev/null || echo "")
if [ -z "$AWS_BIN" ]; then
  for p in "/c/Program Files/Amazon/AWSCLIV2/aws.exe" "/c/Program Files (x86)/Amazon/AWSCLIV2/aws.exe"; do
    [ -f "$p" ] && AWS_BIN="$p" && break
  done
fi
if [ -z "$AWS_BIN" ]; then
  echo "ERROR: aws CLI not found. Install it or add it to PATH."
  exit 1
fi

# Get instance ID and IP from Terraform state
INSTANCE_ID=$("$TF_BIN" -chdir="$TF_DIR" output -raw instance_id 2>/dev/null) || {
  echo "ERROR: Could not read instance_id from Terraform state."
  echo "Make sure you've run 'terraform apply' at least once."
  exit 1
}
PUBLIC_IP=$("$TF_BIN" -chdir="$TF_DIR" output -raw public_ip 2>/dev/null) || true

echo "Starting EC2 instance: $INSTANCE_ID ($PUBLIC_IP)"

# Start the instance
"$AWS_BIN" ec2 start-instances --instance-ids "$INSTANCE_ID" --output text > /dev/null
echo "Start request sent. Waiting for instance to be running..."

"$AWS_BIN" ec2 wait instance-running --instance-ids "$INSTANCE_ID"
echo "Instance is running."

# Wait for SSH to become available
echo "Waiting for SSH..."
for i in $(seq 1 30); do
  if ssh -i ~/.ssh/isaacsim-key.pem -o ConnectTimeout=3 -o StrictHostKeyChecking=no \
    ubuntu@"$PUBLIC_IP" "echo ok" 2>/dev/null; then
    break
  fi
  if [ "$i" -eq 30 ]; then
    echo "WARNING: SSH not available after 90s. Instance may still be booting."
    echo "Try manually: ssh -i ~/.ssh/isaacsim-key.pem ubuntu@$PUBLIC_IP"
    exit 1
  fi
  sleep 3
done

# Start Docker containers
echo "Starting Docker containers..."
ssh -i ~/.ssh/isaacsim-key.pem -o StrictHostKeyChecking=no \
  ubuntu@"$PUBLIC_IP" \
  "cd /home/ubuntu/isaacsim-aws/docker && docker compose up -d"

echo ""
echo "Instance is up! Container startup takes ~2-5 min (shader compile)."
echo "  Web UI:  http://$PUBLIC_IP"
echo "  SSH:     ssh -i ~/.ssh/isaacsim-key.pem ubuntu@$PUBLIC_IP"
echo "  Logs:    ssh -i ~/.ssh/isaacsim-key.pem ubuntu@$PUBLIC_IP 'cd /home/ubuntu/isaacsim-aws/docker && docker compose logs -f'"
