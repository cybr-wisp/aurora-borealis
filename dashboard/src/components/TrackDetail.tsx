import type {Track} from "@/lib/types";

export function TrackDetail({track}: {track?: Track}) {
  if (!track) {
    return (
      <div className="panel">
        <div className="panel-heading">TRACK</div>
        <div className="empty">No active track.</div>
      </div>
    );
  }

  const speed = Math.hypot(...track.velocity);

  return (
    <div className="panel">
      <div className="panel-heading">
        <span>{track.label}</span>
        <span className={`status status-${track.status.toLowerCase()}`}>
          {track.status}
        </span>
      </div>
      <dl className="kv">
        <dt>Position</dt>
        <dd>{track.position.map((v) => v.toFixed(1)).join(", ")} m</dd>
        <dt>Velocity</dt>
        <dd>{speed.toFixed(2)} m/s</dd>
        <dt>NEES</dt>
        <dd>{track.nees.toFixed(2)}</dd>
        <dt>Filter</dt>
        <dd>{track.filter}</dd>
      </dl>
    </div>
  );
}
