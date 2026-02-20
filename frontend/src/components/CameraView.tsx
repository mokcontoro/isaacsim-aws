import { videoStreamUrl } from '../lib/ros';

export function CameraView() {
  return (
    <div style={{ position: 'relative', width: '100%', background: '#1a1a1a' }}>
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
          // Show placeholder when stream is unavailable
          (e.target as HTMLImageElement).style.display = 'none';
        }}
      />
      <noscript>Camera feed requires JavaScript</noscript>
    </div>
  );
}
