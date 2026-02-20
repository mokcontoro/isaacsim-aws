import { useEffect, useState } from 'react';
import { ros, odomTopic } from '../lib/ros';

interface OdomState {
  x: number;
  y: number;
  theta: number;
  linearVel: number;
  angularVel: number;
}

export function StatusBar() {
  const [connected, setConnected] = useState(false);
  const [odom, setOdom] = useState<OdomState>({
    x: 0, y: 0, theta: 0, linearVel: 0, angularVel: 0,
  });

  useEffect(() => {
    const onConnect = () => setConnected(true);
    const onClose = () => setConnected(false);
    ros.on('connection', onConnect);
    ros.on('close', onClose);

    // Check current state
    if ((ros as any).isConnected) setConnected(true);

    return () => {
      ros.off('connection', onConnect);
      ros.off('close', onClose);
    };
  }, []);

  useEffect(() => {
    const callback = (msg: any) => {
      const pos = msg.pose.pose.position;
      const orient = msg.pose.pose.orientation;
      // Convert quaternion to yaw (theta)
      const siny = 2.0 * (orient.w * orient.z + orient.x * orient.y);
      const cosy = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z);
      const theta = Math.atan2(siny, cosy);

      setOdom({
        x: pos.x,
        y: pos.y,
        theta,
        linearVel: msg.twist.twist.linear.x,
        angularVel: msg.twist.twist.angular.z,
      });
    };

    odomTopic.subscribe(callback);
    return () => odomTopic.unsubscribe(callback);
  }, []);

  const statusColor = connected ? '#2ecc71' : '#e74c3c';
  const statusText = connected ? 'Connected' : 'Disconnected';

  return (
    <div
      style={{
        display: 'flex',
        justifyContent: 'space-between',
        padding: '8px 16px',
        background: '#1a1a1a',
        borderTop: '1px solid #333',
        fontSize: '13px',
        fontFamily: 'monospace',
        color: '#ccc',
      }}
    >
      <span>
        <span style={{ color: statusColor }}>{'\u25CF'}</span> {statusText}
      </span>
      <span>
        Pos: ({odom.x.toFixed(2)}, {odom.y.toFixed(2)}) &theta;: {(odom.theta * 180 / Math.PI).toFixed(1)}&deg;
      </span>
      <span>
        Lin: {odom.linearVel.toFixed(2)} m/s &middot; Ang: {odom.angularVel.toFixed(2)} rad/s
      </span>
    </div>
  );
}
