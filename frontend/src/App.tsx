import { useState } from 'react';
import { CameraView } from './components/CameraView';
import { TeleopPad } from './components/TeleopPad';
import { SpeedSlider } from './components/SpeedSlider';
import { NavGoal } from './components/NavGoal';
import { StatusBar } from './components/StatusBar';

export function App() {
  const [speedScale, setSpeedScale] = useState(0.45); // ~0.1 m/s default

  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      height: '100vh',
      background: '#111',
      color: '#eee',
      fontFamily: 'system-ui, sans-serif',
    }}>
      {/* Header */}
      <header style={{
        padding: '12px 16px',
        background: '#1a1a1a',
        borderBottom: '1px solid #333',
        fontSize: '18px',
        fontWeight: 600,
      }}>
        Isaac Sim Remote Control
        <span style={{ fontSize: '12px', fontWeight: 400, color: '#888', marginLeft: '12px' }}>
          v0.3.0
        </span>
      </header>

      {/* Main content */}
      <main style={{
        display: 'flex',
        flex: 1,
        minHeight: 0,
        overflow: 'hidden',
      }}>
        {/* Left: Camera feed */}
        <div style={{ flex: 2, display: 'flex', overflow: 'hidden', height: '100%' }}>
          <CameraView />
        </div>

        {/* Right: Controls panel */}
        <div style={{
          flex: 1,
          padding: '16px',
          borderLeft: '1px solid #333',
          overflowY: 'auto',
          minWidth: '280px',
        }}>
          <TeleopPad speedScale={speedScale} />
          <SpeedSlider value={speedScale} onChange={setSpeedScale} />
          <NavGoal />
        </div>
      </main>

      {/* Footer: Status bar */}
      <StatusBar />
    </div>
  );
}
