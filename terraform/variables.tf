variable "aws_region" {
  description = "AWS region to deploy in"
  type        = string
  default     = "us-east-1"
}

variable "instance_type" {
  description = "EC2 instance type (must have NVIDIA GPU)"
  type        = string
  default     = "g5.xlarge"
}

variable "key_name" {
  description = "Name of existing AWS key pair for SSH access"
  type        = string
}

variable "allowed_ssh_cidr" {
  description = "CIDR block allowed to SSH (your IP, e.g. 1.2.3.4/32)"
  type        = string
}

variable "project_repo_url" {
  description = "Git repo URL to clone onto the instance"
  type        = string
  default     = ""
}

variable "root_volume_size" {
  description = "Root EBS volume size in GB"
  type        = number
  default     = 200
}
