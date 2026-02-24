import { useEffect, useCallback, useState } from 'react';
import { publishTwist, publishStop } from '../lib/ros';

// TurtleBot3 Burger limits
const MAX_LINEAR = 0.22; // m/s
const MAX_ANGULAR = 2.84; // rad/s

interface TeleopPadProps {
  speedScale: number; // 0.0 - 1.0
}

export function TeleopPad({ speedScale }: TeleopPadProps) {
  const linear = MAX_LINEAR * speedScale;
  const angular = MAX_ANGULAR * speedScale;
  const [activeKey, setActiveKey] = useState<string | null>(null);

  const handleKey = useCallback(
    (e: KeyboardEvent) => {
      if (e.repeat) return;
      // Prevent page scroll on arrow keys
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(e.key)) {
        e.preventDefault();
      }

      switch (e.key) {
        case 'w': case 'ArrowUp':
          setActiveKey('W');
          publishTwist(linear, 0); break;
        case 's': case 'ArrowDown':
          setActiveKey('S');
          publishTwist(-linear, 0); break;
        case 'a': case 'ArrowLeft':
          setActiveKey('A');
          publishTwist(0, angular); break;
        case 'd': case 'ArrowRight':
          setActiveKey('D');
          publishTwist(0, -angular); break;
        case ' ':
          setActiveKey('STOP');
          publishStop(); break;
      }
    },
    [linear, angular]
  );

  const handleKeyUp = useCallback(
    (e: KeyboardEvent) => {
      if (['w', 's', 'a', 'd', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(e.key)) {
        setActiveKey(null);
        publishStop();
      }
    },
    []
  );

  useEffect(() => {
    window.addEventListener('keydown', handleKey);
    window.addEventListener('keyup', handleKeyUp);
    return () => {
      window.removeEventListener('keydown', handleKey);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [handleKey, handleKeyUp]);

  const btnStyle = {
    width: '60px',
    height: '60px',
    fontSize: '24px',
    cursor: 'pointer',
    border: '1px solid #555',
    borderRadius: '8px',
    background: '#2a2a2a',
    color: '#fff',
  };

  return (
    <div>
      <h3 style={{ margin: '0 0 8px' }}>Teleop Controls</h3>
      <p style={{ fontSize: '12px', color: '#888', margin: '0 0 12px' }}>
        WASD or Arrow Keys &middot; Space = Stop
        {activeKey && (
          <span style={{ marginLeft: '8px', color: '#2ecc71', fontWeight: 600 }}>
            [{activeKey}]
          </span>
        )}
      </p>
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 60px)', gap: '4px', justifyContent: 'center' }}>
        <div />
        <button style={btnStyle} onMouseDown={() => publishTwist(linear, 0)} onMouseUp={publishStop}>&#9650;</button>
        <div />
        <button style={btnStyle} onMouseDown={() => publishTwist(0, angular)} onMouseUp={publishStop}>&#9664;</button>
        <button style={{ ...btnStyle, background: '#c0392b' }} onClick={publishStop}>&#9632;</button>
        <button style={btnStyle} onMouseDown={() => publishTwist(0, -angular)} onMouseUp={publishStop}>&#9654;</button>
        <div />
        <button style={btnStyle} onMouseDown={() => publishTwist(-linear, 0)} onMouseUp={publishStop}>&#9660;</button>
        <div />
      </div>
    </div>
  );
}
