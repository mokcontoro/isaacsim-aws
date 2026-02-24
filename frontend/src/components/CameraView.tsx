import { useState } from 'react';
import { videoStreamUrl } from '../lib/ros';
import { IsaacSimViewer } from './IsaacSimViewer';

type ViewMode = 'webrtc' | 'camera';

export function CameraView() {
  const [viewMode, setViewMode] = useState<ViewMode>('webrtc');
  const [webrtcStatus, setWebrtcStatus] = useState<string>('connecting');

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%', overflow: 'hidden', background: '#1a1a1a' }}>
      {/* Main view */}
      {viewMode === 'webrtc' ? (
        <IsaacSimViewer onStatusChange={setWebrtcStatus} />
      ) : (
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
      )}

      {/* MJPEG PiP overlay — only shown in WebRTC mode */}
      {viewMode === 'webrtc' && (
        <div style={{
          position: 'absolute',
          top: '12px',
          right: '12px',
          width: '200px',
          height: '150px',
          zIndex: 3,
          border: '2px solid rgba(255,255,255,0.5)',
          borderRadius: '6px',
          overflow: 'hidden',
          boxShadow: '0 2px 12px rgba(0,0,0,0.6)',
          background: '#222',
          cursor: 'pointer',
        }}
          title="Click to switch to camera view"
          onClick={() => setViewMode('camera')}
        >
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
            Robot Cam
          </div>
          <img
            src={videoStreamUrl}
            alt="Robot camera"
            style={{
              width: '100%',
              height: '100%',
              display: 'block',
              objectFit: 'cover',
            }}
          />
        </div>
      )}

      {/* View toggle button */}
      <button
        onClick={() => setViewMode(viewMode === 'webrtc' ? 'camera' : 'webrtc')}
        style={{
          position: 'absolute',
          bottom: '12px',
          left: '12px',
          zIndex: 3,
          padding: '6px 14px',
          fontSize: '12px',
          fontWeight: 600,
          color: '#fff',
          background: 'rgba(40,40,40,0.85)',
          border: '1px solid rgba(255,255,255,0.3)',
          borderRadius: '6px',
          cursor: 'pointer',
          backdropFilter: 'blur(4px)',
        }}
      >
        {viewMode === 'webrtc' ? 'Camera View' : '3D View'}
        {viewMode === 'webrtc' && webrtcStatus === 'connecting' && (
          <span style={{ marginLeft: '6px', color: '#f39c12' }}>...</span>
        )}
      </button>

      {/* Controls hint for WebRTC mode */}
      {viewMode === 'webrtc' && webrtcStatus === 'connected' && (
        <div style={{
          position: 'absolute',
          bottom: '12px',
          right: '12px',
          zIndex: 3,
          padding: '4px 10px',
          fontSize: '11px',
          color: 'rgba(255,255,255,0.5)',
          background: 'rgba(0,0,0,0.5)',
          borderRadius: '4px',
        }}>
          ALT+Drag: Orbit &middot; Middle: Pan &middot; Scroll: Zoom
        </div>
      )}

      <noscript>Camera feed requires JavaScript</noscript>
    </div>
  );
}
