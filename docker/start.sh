#!/bin/bash
# Detect public IP and start all containers with WebRTC support
set -euo pipefail

cd "$(dirname "$0")"

# Detect public IP (EC2 metadata or fallback to ifconfig.me)
PUBLIC_IP=$(curl -sf --connect-timeout 2 http://169.254.169.254/latest/meta-data/public-ipv4 2>/dev/null \
  || curl -sf --connect-timeout 5 https://ifconfig.me 2>/dev/null \
  || echo "127.0.0.1")

echo "Detected PUBLIC_IP: $PUBLIC_IP"

export PUBLIC_IP
docker compose up -d --build
