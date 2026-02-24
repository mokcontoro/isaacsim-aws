import * as ROSLIB from 'roslib';

// Connect to rosbridge via Nginx proxy
const wsUrl = `ws://${window.location.host}/ws`;

export const ros = new ROSLIB.Ros({ url: wsUrl });

ros.on('connection', () => console.log('Connected to rosbridge'));
ros.on('error', (err) => console.error('rosbridge error:', err));
ros.on('close', () => console.log('rosbridge connection closed'));

// Typed topic helpers
export const cmdVelTopic = new ROSLIB.Topic({
  ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/msg/Twist',
});

export const odomTopic = new ROSLIB.Topic({
  ros,
  name: '/odom',
  messageType: 'nav_msgs/msg/Odometry',
});

export const goalPoseTopic = new ROSLIB.Topic({
  ros,
  name: '/goal_pose',
  messageType: 'geometry_msgs/msg/PoseStamped',
});

// Video stream URLs (MJPEG via web_video_server through Nginx)
export const videoStreamUrl =
  `/video/stream?topic=/camera/image&type=mjpeg&quality=80`;

export const birdseyeStreamUrl =
  `/video/stream?topic=/camera/birdseye&type=mjpeg&quality=80`;

// Publish a Twist message
export function publishTwist(linear: number, angular: number) {
  const msg = new ROSLIB.Message({
    linear: { x: linear, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angular },
  });
  console.log(`[teleop] publish cmd_vel: linear=${linear.toFixed(3)}, angular=${angular.toFixed(3)}`);
  cmdVelTopic.publish(msg);
}

// Publish a zero-velocity stop command
export function publishStop() {
  publishTwist(0, 0);
}
