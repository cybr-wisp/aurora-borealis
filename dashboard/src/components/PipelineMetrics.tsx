import type {PipelineMetrics as Metrics} from "@/lib/types";

export function PipelineMetrics({metrics}: {metrics: Metrics}) {
  const cells = [
    ["OBS/S", metrics.observationsPerSecond.toFixed(0)],
    ["TRACKS", String(metrics.activeTracks)],
    ["LIVE UPDATE", `${metrics.liveUpdateMs.toFixed(2)} ms`],
    ["BENCH P99", `${metrics.benchAssociationP99Ms.toFixed(2)} ms`],
    ["LOSS", `${metrics.packetLossPct.toFixed(1)}%`],
  ];

  return (
    <div className="metrics-grid">
      {cells.map(([label, value]) => (
        <div className="metric" key={label}>
          <span>{label}</span>
          <strong>{value}</strong>
        </div>
      ))}
    </div>
  );
}
