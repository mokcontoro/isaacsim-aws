interface SpeedSliderProps {
  value: number;
  onChange: (value: number) => void;
}

export function SpeedSlider({ value, onChange }: SpeedSliderProps) {
  return (
    <div style={{ margin: '16px 0' }}>
      <label style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
        <span>Speed:</span>
        <input
          type="range"
          min={0.05}
          max={1.0}
          step={0.05}
          value={value}
          onChange={(e) => onChange(parseFloat(e.target.value))}
          style={{ flex: 1 }}
        />
        <span style={{ fontFamily: 'monospace', minWidth: '60px' }}>
          {(value * 0.22).toFixed(2)} m/s
        </span>
      </label>
    </div>
  );
}
