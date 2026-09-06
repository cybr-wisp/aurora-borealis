"use client";

import {useEffect, useState} from "react";

import {FailureControls} from "@/components/FailureControls";
import {NEESPlot} from "@/components/NEESPlot";
import {PipelineMetrics} from "@/components/PipelineMetrics";
import {SensorHealth} from "@/components/SensorHealth";
import {TrackDetail} from "@/components/TrackDetail";
import {TrackingCanvas} from "@/components/TrackingCanvas";
import {useAuroraSocket} from "@/hooks/useWebSocket";

export default function Home() {
  const {state, connected, send} = useAuroraSocket();
  const [selectedTrackId, setSelectedTrackId] =
    useState<number | null>(null);

  useEffect(() => {
    if (!state) return;

    if (state.tracks.length === 0) {
      if (selectedTrackId !== null) {
        setSelectedTrackId(null);
      }
      return;
    }

    const selectionStillExists = state.tracks.some(
      (track) => track.id === selectedTrackId,
    );

    if (!selectionStillExists) {
      setSelectedTrackId(state.tracks[0].id);
    }
  }, [state, selectedTrackId]);

  if (!state) {
    return (
      <main className="boot">
        <div className="eyebrow">AURORA BOREALIS</div>
        <h1>Waiting for state stream</h1>
        <p>
          Start <code>python simulation/live_operator.py</code>.
        </p>
      </main>
    );
  }

  const selectedTrack =
    state.tracks.find((track) => track.id === selectedTrackId) ??
    state.tracks[0];

  const sendFromControls = (
    payload: Parameters<typeof send>[0],
  ) => {
    if (
      payload.action === "trigger_maneuver" &&
      selectedTrack
    ) {
      send(
        {
          ...payload,
          targetId: selectedTrack.id,
        } as Parameters<typeof send>[0],
      );
      return;
    }

    send(payload);
  };

  return (
    <main className="shell">
      <header className="topbar">
        <div>
          <div className="eyebrow">AURORA BOREALIS</div>
          <h1>Operator Console</h1>
        </div>
        <div className={connected ? "live" : "offline"}>
          {connected ? "LIVE" : "DISCONNECTED"}
        </div>
      </header>

      <PipelineMetrics metrics={state.metrics} />

      <div className="layout">
        <TrackingCanvas
          state={state}
          selectedTrackId={selectedTrack?.id ?? null}
          onSelectTrack={setSelectedTrackId}
        />
        <div className="sidebar">
          <TrackDetail track={selectedTrack} />
          <SensorHealth sensors={state.sensors} />
          <NEESPlot samples={state.neesHistory} />
          <FailureControls sensors={state.sensors} send={sendFromControls} />
        </div>
      </div>

      <footer>
        t={state.timestampSec.toFixed(1)} s
        {state.message ? ` · ${state.message}` : ""}
      </footer>
    </main>
  );
}
