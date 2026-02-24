output "instance_id" {
  description = "EC2 instance ID (for stop/start scripts)"
  value       = aws_instance.isaacsim.id
}

output "public_ip" {
  description = "Public IP address of the Isaac Sim instance"
  value       = aws_eip.isaacsim.public_ip
}

output "ssh_command" {
  description = "SSH command to connect to the instance"
  value       = "ssh -i ~/.ssh/${var.key_name}.pem ubuntu@${aws_eip.isaacsim.public_ip}"
}

output "web_url" {
  description = "URL to access the web frontend"
  value       = "http://${aws_eip.isaacsim.public_ip}"
}
