import type {Sensor} from "@/lib/types";

export function SensorHealth({sensors}: {sensors: Sensor[]}) {
  return (
    <div className="panel">
      <div className="panel-heading">SENSOR HEALTH</div>
      <div className="sensor-table">
        {sensors.map((sensor) => (
          <div className="sensor-row" key={sensor.id}>
            <span>{sensor.label}</span>
            <span className={`status status-${sensor.health.toLowerCase()}`}>
              {sensor.health}
            </span>
            <span>R {sensor.estimatedR.toFixed(2)} / {sensor.configuredR.toFixed(2)}</span>
            <span>loss {sensor.packetLossPct.toFixed(1)}%</span>
          </div>
        ))}
      </div>
    </div>
  );
}
