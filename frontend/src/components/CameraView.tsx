import { useState } from 'react';
import { videoStreamUrl, birdseyeStreamUrl } from '../lib/ros';

export function CameraView() {
  const [birdseyeLoaded, setBirdseyeLoaded] = useState(false);

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%', overflow: 'hidden', background: '#1a1a1a' }}>
      {/* Main chase camera — absolute positioning forces fill regardless of MJPEG natural dimensions */}
      <img
        src={videoStreamUrl}
        alt="Robot camera feed"
        style={{
          position: 'absolute',
          top: 0,
          left: 0,
          width: '100%',
          height: '100%',
          display: 'block',
          objectFit: 'contain',
        }}
        onError={(e) => {
          (e.target as HTMLImageElement).style.display = 'none';
        }}
      />

      {/* Bird's eye PiP overlay */}
      <div style={{
        position: 'absolute',
        top: '12px',
        right: '12px',
        width: '180px',
        height: '180px',
        zIndex: 2,
        border: '2px solid rgba(255,255,255,0.6)',
        borderRadius: '6px',
        overflow: 'hidden',
        boxShadow: '0 2px 12px rgba(0,0,0,0.6)',
        background: '#222',
      }}>
        <div style={{
          position: 'absolute',
          top: '4px',
          left: '6px',
          fontSize: '11px',
          fontWeight: 600,
          color: '#fff',
          textShadow: '0 1px 3px rgba(0,0,0,0.9)',
          zIndex: 1,
          pointerEvents: 'none',
        }}>
          Bird's Eye
        </div>
        {!birdseyeLoaded && (
          <div style={{
            position: 'absolute',
            inset: 0,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            color: '#666',
            fontSize: '12px',
          }}>
            Loading...
          </div>
        )}
        <img
          src={birdseyeStreamUrl}
          alt="Bird's eye view"
          style={{
            width: '100%',
            height: '100%',
            display: 'block',
            objectFit: 'cover',
          }}
          onLoad={() => setBirdseyeLoaded(true)}
        />
      </div>

      <noscript>Camera feed requires JavaScript</noscript>
    </div>
  );
}
