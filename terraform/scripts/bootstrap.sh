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
