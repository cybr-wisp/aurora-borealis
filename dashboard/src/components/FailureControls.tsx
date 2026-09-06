import type {
  DashboardCommand,
  Sensor,
} from "@/lib/types";

interface Props {
  sensors: Sensor[];
  send: (command: DashboardCommand) => void;
}

export function FailureControls({sensors, send}: Props) {
  return (
    <div className="panel">
      <div className="panel-heading">
        <span>FAILURE CONTROLS</span>
        <span className="muted">per-sensor + global</span>
      </div>

      <div className="sensor-controls">
        {sensors.map((sensor) => (
          <div className="sensor-control-card" key={sensor.id}>
            <div className="sensor-control-header">
              <strong>{sensor.label}</strong>
              <span
                className={`status status-${sensor.health.toLowerCase()}`}
              >
                {sensor.health}
              </span>
            </div>

            <div className="sensor-control-grid">
              <button
                disabled={sensor.health === "OFFLINE"}
                onClick={() =>
                  send({
                    action: "kill_sensor",
                    sensorId: sensor.id,
                  })
                }
              >
                Kill
              </button>

              <button
                disabled={sensor.health !== "OFFLINE"}
                onClick={() =>
                  send({
                    action: "restore_sensor",
                    sensorId: sensor.id,
                  })
                }
              >
                Restore
              </button>

              <button
                className={sensor.biasActive ? "control-active" : ""}
                onClick={() =>
                  send({
                    action: "inject_bias",
                    sensorId: sensor.id,
                  })
                }
              >
                Bias {sensor.biasActive ? "ON" : "OFF"}
              </button>

              <button
                className={sensor.noiseActive ? "control-active" : ""}
                onClick={() =>
                  send({
                    action: "add_noise",
                    sensorId: sensor.id,
                  })
                }
              >
                Noise {sensor.noiseActive ? "ON" : "OFF"}
              </button>
            </div>
          </div>
        ))}
      </div>

      <div className="global-controls">
        <button onClick={() => send({action: "trigger_maneuver"})}>
          Maneuver selected track
        </button>
        <button onClick={() => send({action: "add_target"})}>
          Add target
        </button>
        <button onClick={() => send({action: "toggle_filter"})}>
          EKF ↔ UKF
        </button>
        <button onClick={() => send({action: "reset"})}>
          Reset
        </button>
      </div>
    </div>
  );
}
