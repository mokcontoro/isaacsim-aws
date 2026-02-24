import { useState, useRef, useEffect } from 'react';
import { videoStreamUrl } from '../lib/ros';

type ViewMode = '3d' | 'camera';

export function CameraView() {
  const [viewMode, setViewMode] = useState<ViewMode>('3d');
  const [iframeError, setIframeError] = useState(false);
  const iframeTimeout = useRef<ReturnType<typeof setTimeout>>(null);
  const iframeRef = useRef<HTMLIFrameElement>(null);

  const webrtcUrl = `/streaming/webrtc-client?server=${window.location.hostname}`;

  // Detect iframe load failure after 15s timeout
  useEffect(() => {
    if (viewMode !== '3d') return;
    setIframeError(false);

    iframeTimeout.current = setTimeout(() => {
      setIframeError(true);
    }, 15000);

    return () => {
      if (iframeTimeout.current) clearTimeout(iframeTimeout.current);
    };
  }, [viewMode]);

  const handleIframeLoad = () => {
    if (iframeTimeout.current) clearTimeout(iframeTimeout.current);
  };

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%', overflow: 'hidden', background: '#1a1a1a' }}>
      {/* Main view area */}
      {viewMode === '3d' ? (
        iframeError ? (
          <div style={{
            width: '100%',
            height: '100%',
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            color: '#999',
            gap: '12px',
          }}>
            <div style={{ fontSize: '16px' }}>WebRTC stream unavailable</div>
            <div style={{ fontSize: '13px', color: '#666' }}>
              Check that Isaac Sim is running with streaming enabled
            </div>
            <button
              onClick={() => setViewMode('camera')}
              style={{
                marginTop: '8px',
                padding: '8px 16px',
                background: '#333',
                color: '#eee',
                border: '1px solid #555',
                borderRadius: '4px',
                cursor: 'pointer',
                fontSize: '13px',
              }}
            >
              Switch to MJPEG Camera
            </button>
          </div>
        ) : (
          <iframe
            ref={iframeRef}
            src={webrtcUrl}
            onLoad={handleIframeLoad}
            style={{
              width: '100%',
              height: '100%',
              border: 'none',
              display: 'block',
            }}
            allow="autoplay"
          />
        )
      ) : (
        <img
          src={videoStreamUrl}
          alt="Robot camera feed"
          style={{
            width: '100%',
            height: '100%',
            display: 'block',
            objectFit: 'cover',
          }}
          onError={(e) => {
            (e.target as HTMLImageElement).style.display = 'none';
          }}
        />
      )}

      {/* MJPEG PiP overlay (chase cam — visible in 3D view mode) */}
      {viewMode === '3d' && !iframeError && (
        <div style={{
          position: 'absolute',
          top: '12px',
          right: '12px',
          width: '180px',
          height: '135px',
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
            Robot Cam
          </div>
          <img
            src={videoStreamUrl}
            alt="Robot chase camera"
            style={{
              width: '100%',
              height: '100%',
              display: 'block',
              objectFit: 'cover',
            }}
          />
        </div>
      )}

      {/* View mode toggle */}
      <button
        onClick={() => setViewMode(viewMode === '3d' ? 'camera' : '3d')}
        style={{
          position: 'absolute',
          bottom: '12px',
          right: '12px',
          padding: '6px 14px',
          background: 'rgba(30,30,30,0.85)',
          color: '#eee',
          border: '1px solid rgba(255,255,255,0.3)',
          borderRadius: '4px',
          cursor: 'pointer',
          fontSize: '13px',
          fontWeight: 500,
          zIndex: 3,
          backdropFilter: 'blur(4px)',
        }}
      >
        {viewMode === '3d' ? 'Camera' : '3D View'}
      </button>

      <noscript>Camera feed requires JavaScript</noscript>
    </div>
  );
}
