#!/bin/bash
# Stop the Isaac Sim EC2 instance (saves ~$1/hr, keeps disk + EIP)
# Idle cost while stopped: ~$0.60/day (200GB EBS + Elastic IP)
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

echo "Stopping EC2 instance: $INSTANCE_ID ($PUBLIC_IP)"

# Stop containers gracefully via SSH first (optional, best-effort)
echo "Stopping Docker containers..."
ssh -i ~/.ssh/isaacsim-key.pem -o ConnectTimeout=5 -o StrictHostKeyChecking=no \
  ubuntu@"$PUBLIC_IP" \
  "cd /home/ubuntu/isaacsim-aws/docker && docker compose down" 2>/dev/null || {
  echo "  (Could not reach instance via SSH — stopping instance anyway)"
}

# Stop the instance
"$AWS_BIN" ec2 stop-instances --instance-ids "$INSTANCE_ID" --output text > /dev/null
echo "Stop request sent. Waiting for instance to stop..."

"$AWS_BIN" ec2 wait instance-stopped --instance-ids "$INSTANCE_ID"
echo "Instance stopped. No compute charges accruing."
echo "To restart: ./scripts/ec2-start.sh"
