import { useState } from 'react';
import { goalPoseTopic } from '../lib/ros';
import ROSLIB from 'roslib';

export function NavGoal() {
  const [x, setX] = useState('0.0');
  const [y, setY] = useState('0.0');

  function sendGoal() {
    const goal = new ROSLIB.Message({
      header: {
        frame_id: 'map',
        stamp: { sec: 0, nanosec: 0 },
      },
      pose: {
        position: { x: parseFloat(x) || 0, y: parseFloat(y) || 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
    });
    goalPoseTopic.publish(goal);
  }

  const inputStyle = {
    width: '70px',
    padding: '4px 8px',
    background: '#2a2a2a',
    border: '1px solid #555',
    borderRadius: '4px',
    color: '#fff',
    fontFamily: 'monospace',
  };

  return (
    <div style={{ margin: '16px 0' }}>
      <h3 style={{ margin: '0 0 8px' }}>Nav2 Waypoint</h3>
      <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
        <label>
          X: <input style={inputStyle} value={x} onChange={(e) => setX(e.target.value)} />
        </label>
        <label>
          Y: <input style={inputStyle} value={y} onChange={(e) => setY(e.target.value)} />
        </label>
        <button
          onClick={sendGoal}
          style={{
            padding: '4px 16px',
            background: '#2980b9',
            color: '#fff',
            border: 'none',
            borderRadius: '4px',
            cursor: 'pointer',
          }}
        >
          Send Goal
        </button>
      </div>
    </div>
  );
}
