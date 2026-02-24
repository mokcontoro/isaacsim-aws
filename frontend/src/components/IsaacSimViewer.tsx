import { useEffect, useRef, useState, useCallback } from 'react';
import { AppStreamer, StreamEvent, StreamType } from '@nvidia/omniverse-webrtc-streaming-library';

type StreamStatus = 'connecting' | 'connected' | 'error' | 'disconnected';

interface IsaacSimViewerProps {
  onStatusChange?: (status: StreamStatus) => void;
}

export function IsaacSimViewer({ onStatusChange }: IsaacSimViewerProps) {
  const [status, setStatus] = useState<StreamStatus>('connecting');
  const connectedRef = useRef(false);

  const updateStatus = useCallback((s: StreamStatus) => {
    setStatus(s);
    onStatusChange?.(s);
  }, [onStatusChange]);

  useEffect(() => {
    if (connectedRef.current) return;
    connectedRef.current = true;

    // Connect to Isaac Sim's WebRTC signaling server.
    // The browser accesses the app at http://<host>, so window.location.hostname
    // gives us the EC2 public IP (or localhost for local dev).
    const signalingServer = window.location.hostname;

    const streamConfig = {
      videoElementId: 'isaac-sim-video',
      audioElementId: 'isaac-sim-audio',
      authenticate: false,
      maxReconnects: 20,
      reconnectDelay: 3000,
      connectivityTimeout: 5000,
      signalingServer,
      signalingPort: 49100,
      mediaServer: signalingServer,
      mediaPort: 47998,
      nativeTouchEvents: true,
      width: 1280,
      height: 720,
      fps: 30,
      cursor: 'free' as const,
      onStart: (message: StreamEvent) => {
        if (message.action === 'start' && message.status === 'success') {
          updateStatus('connected');
        }
        if (message.status === 'error') {
          console.error('WebRTC stream error:', message);
          updateStatus('error');
        }
      },
      onStop: (message: StreamEvent) => {
        console.log('WebRTC stream stopped:', message);
        updateStatus('disconnected');
      },
      onTerminate: (message: StreamEvent) => {
        console.log('WebRTC stream terminated:', message);
        updateStatus('disconnected');
      },
      onUpdate: (message: StreamEvent) => {
        console.log('WebRTC update:', message);
      },
    };

    AppStreamer.connect({
      streamConfig,
      streamSource: StreamType.DIRECT,
    })
      .then((result: StreamEvent) => {
        console.info('WebRTC connected:', result);
      })
      .catch((error: StreamEvent) => {
        console.error('WebRTC connection failed:', error);
        updateStatus('error');
      });

    return () => {
      AppStreamer.terminate();
    };
  }, [updateStatus]);

  return (
    <div
      tabIndex={0}
      style={{
        position: 'relative',
        width: '100%',
        height: '100%',
        background: '#111',
        outline: 'none',
      }}
    >
      {/* Status overlay */}
      {status !== 'connected' && (
        <div style={{
          position: 'absolute',
          inset: 0,
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          zIndex: 2,
          background: 'rgba(0,0,0,0.7)',
          color: '#aaa',
          fontSize: '14px',
          gap: '8px',
        }}>
          {status === 'connecting' && (
            <>
              <div style={{ fontSize: '16px', color: '#fff' }}>Connecting to 3D viewer...</div>
              <div>Isaac Sim WebRTC on {window.location.hostname}:49100</div>
              <div style={{ fontSize: '12px', color: '#666', marginTop: '8px' }}>
                First connection may take a few minutes (shader compilation)
              </div>
            </>
          )}
          {status === 'error' && (
            <>
              <div style={{ fontSize: '16px', color: '#e74c3c' }}>WebRTC connection failed</div>
              <div>Could not connect to Isaac Sim streaming server</div>
              <div style={{ fontSize: '12px', color: '#666', marginTop: '8px' }}>
                Check that Isaac Sim is running and ports 49100/47998 are open
              </div>
            </>
          )}
          {status === 'disconnected' && (
            <div style={{ fontSize: '16px', color: '#f39c12' }}>Stream disconnected</div>
          )}
        </div>
      )}

      {/* WebRTC video element — the library binds to this by ID */}
      <video
        id="isaac-sim-video"
        style={{
          width: '100%',
          height: '100%',
          display: 'block',
          objectFit: 'contain',
          background: '#000',
        }}
        tabIndex={-1}
        playsInline
        muted
        autoPlay
      />
      <audio id="isaac-sim-audio" muted />
    </div>
  );
}
