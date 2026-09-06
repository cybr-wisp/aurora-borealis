export type TrackStatus =
  | "TENTATIVE"
  | "CONFIRMED"
  | "COASTING"
  | "DELETED";

export type SensorHealthState =
  | "NOMINAL"
  | "DEGRADED"
  | "OFFLINE";

export interface Track {
  id: number;
  label: string;
  position: [number, number, number];
  velocity: [number, number, number];
  covariance2d: [number, number, number];
  nees: number;
  status: TrackStatus;
  filter: "EKF" | "UKF";
  trail: Array<[number, number]>;
}

export interface Sensor {
  id: number;
  label: string;
  position: [number, number, number];
  health: SensorHealthState;
  configuredR: number;
  estimatedR: number;
  packetsReceived: number;
  packetLossPct: number;
  biasActive: boolean;
  noiseActive: boolean;
}

export interface Observation {
  sensorId: number;
  targetId: number;
  position: [number, number];
}

export interface PipelineMetrics {
  observationsPerSecond: number;
  activeTracks: number;
  liveUpdateMs: number;
  benchAssociationP99Ms: number;
  packetLossPct: number;
}

export interface NeesSample {
  timeSec: number;
  value: number;
}

export interface WorldState {
  timestampSec: number;
  connected: boolean;
  tracks: Track[];
  sensors: Sensor[];
  observations: Observation[];
  neesHistory: NeesSample[];
  metrics: PipelineMetrics;
  message?: string;
}

export interface DashboardCommand {
  action:
    | "kill_sensor"
    | "restore_sensor"
    | "inject_bias"
    | "add_noise"
    | "trigger_maneuver"
    | "add_target"
    | "toggle_filter"
    | "reset";
  sensorId?: number;
}
