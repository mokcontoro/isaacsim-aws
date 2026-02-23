import { videoStreamUrl, birdseyeStreamUrl } from '../lib/ros';

export function CameraView() {
  return (
    <div style={{ position: 'relative', width: '100%', background: '#1a1a1a' }}>
      {/* Main chase camera */}
      <img
        src={videoStreamUrl}
        alt="Robot camera feed"
        style={{
          width: '100%',
          height: 'auto',
          display: 'block',
          minHeight: '360px',
        }}
        onError={(e) => {
          (e.target as HTMLImageElement).style.display = 'none';
        }}
      />

      {/* Bird's eye PiP overlay */}
      <div style={{
        position: 'absolute',
        top: '8px',
        right: '8px',
        border: '2px solid rgba(255,255,255,0.6)',
        borderRadius: '4px',
        overflow: 'hidden',
        boxShadow: '0 2px 8px rgba(0,0,0,0.5)',
        background: '#000',
      }}>
        <div style={{
          position: 'absolute',
          top: '2px',
          left: '4px',
          fontSize: '10px',
          color: 'rgba(255,255,255,0.8)',
          textShadow: '0 1px 2px rgba(0,0,0,0.8)',
          zIndex: 1,
          pointerEvents: 'none',
        }}>
          Bird's Eye
        </div>
        <img
          src={birdseyeStreamUrl}
          alt="Bird's eye view"
          style={{
            width: '200px',
            height: '200px',
            display: 'block',
            objectFit: 'cover',
          }}
          onError={(e) => {
            (e.target as HTMLImageElement).style.display = 'none';
          }}
        />
      </div>

      <noscript>Camera feed requires JavaScript</noscript>
    </div>
  );
}
