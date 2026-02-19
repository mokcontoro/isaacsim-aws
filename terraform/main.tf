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
