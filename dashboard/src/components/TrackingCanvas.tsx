"use client";

import {useEffect, useRef, useState} from "react";
import * as d3 from "d3";
import type {WorldState} from "@/lib/types";

interface Props {
  state: WorldState;
  selectedTrackId?: number | null;
  onSelectTrack?: (trackId: number) => void;
}

type ViewMode = "tracks" | "focus" | "full";

function paddedDomain(
  minValue: number,
  maxValue: number,
  minSpan: number,
  padFraction = 0.12,
): [number, number] {
  const safeMin = Number.isFinite(minValue) ? minValue : -minSpan / 2;
  const safeMax = Number.isFinite(maxValue) ? maxValue : minSpan / 2;

  const center = (safeMin + safeMax) / 2;
  const span = Math.max(Math.abs(safeMax - safeMin), minSpan);
  const half = (span * (1 + padFraction)) / 2;

  return [center - half, center + half];
}

export function TrackingCanvas({
  state,
  selectedTrackId,
  onSelectTrack,
}: Props) {
  const ref = useRef<SVGSVGElement | null>(null);
  const [viewMode, setViewMode] = useState<ViewMode>("tracks");

  useEffect(() => {
    if (!ref.current) return;

    const svg = d3.select(ref.current);
    const width = 900;
    const height = 520;
    const pad = 42;
    const plotWidth = width - 2 * pad;
    const plotHeight = height - 2 * pad;
    const plotAspect = plotWidth / plotHeight;

    svg.selectAll("*").remove();

    const trackXs = [
      ...state.tracks.flatMap((track) => track.trail.map((p) => p[0])),
      ...state.tracks.map((track) => track.position[0]),
    ];

    const trackYs = [
      ...state.tracks.flatMap((track) => track.trail.map((p) => p[1])),
      ...state.tracks.map((track) => track.position[1]),
    ];

    const fullXs = [
      ...trackXs,
      ...state.sensors.map((sensor) => sensor.position[0]),
      ...state.observations.map((obs) => obs.position[0]),
    ];

    const fullYs = [
      ...trackYs,
      ...state.sensors.map((sensor) => sensor.position[1]),
      ...state.observations.map((obs) => obs.position[1]),
    ];

    const selectedTrack =
      state.tracks.find((track) => track.id === selectedTrackId) ??
      state.tracks[0];

    const focusTrail =
      selectedTrack?.trail.slice(-120) ??
      [];

    const focusXs = selectedTrack
      ? [
          ...focusTrail.map((p) => p[0]),
          selectedTrack.position[0],
        ]
      : [];

    const focusYs = selectedTrack
      ? [
          ...focusTrail.map((p) => p[1]),
          selectedTrack.position[1],
        ]
      : [];

    let sourceXs = trackXs;
    let sourceYs = trackYs;
    let minSpanX = 3200;
    let minSpanY = 2200;

    if (viewMode === "focus" && selectedTrack) {
      sourceXs = focusXs;
      sourceYs = focusYs;
      minSpanX = 1400;
      minSpanY = 950;
    } else if (viewMode === "full") {
      sourceXs = fullXs;
      sourceYs = fullYs;
      minSpanX = 5000;
      minSpanY = 3200;
    }

    const xExtent = d3.extent(
      sourceXs.length ? sourceXs : [-1000, 1000],
    ) as [number, number];

    const yExtent = d3.extent(
      sourceYs.length ? sourceYs : [-1000, 1000],
    ) as [number, number];

    let [xMin, xMax] = paddedDomain(
      xExtent[0],
      xExtent[1],
      minSpanX,
    );

    let [yMin, yMax] = paddedDomain(
      yExtent[0],
      yExtent[1],
      minSpanY,
    );

    const domainAspect = (xMax - xMin) / (yMax - yMin);

    if (domainAspect > plotAspect) {
      const yCenter = (yMin + yMax) / 2;
      const ySpan = (xMax - xMin) / plotAspect;
      yMin = yCenter - ySpan / 2;
      yMax = yCenter + ySpan / 2;
    } else {
      const xCenter = (xMin + xMax) / 2;
      const xSpan = (yMax - yMin) * plotAspect;
      xMin = xCenter - xSpan / 2;
      xMax = xCenter + xSpan / 2;
    }

    const x = d3.scaleLinear()
      .domain([xMin, xMax])
      .range([pad, width - pad]);

    const y = d3.scaleLinear()
      .domain([yMin, yMax])
      .range([height - pad, pad]);

    svg.append("g")
      .attr("class", "grid")
      .attr("transform", `translate(0,${height - pad})`)
      .call(
        d3.axisBottom(x)
          .ticks(12)
          .tickSize(-(height - 2 * pad))
          .tickFormat(() => ""),
      );

    svg.append("g")
      .attr("class", "grid")
      .attr("transform", `translate(${pad},0)`)
      .call(
        d3.axisLeft(y)
          .ticks(9)
          .tickSize(-(width - 2 * pad))
          .tickFormat(() => ""),
      );

    const observations = svg.append("g");
    observations.selectAll("circle")
      .data(state.observations)
      .join("circle")
      .attr("cx", (d) => x(d.position[0]))
      .attr("cy", (d) => y(d.position[1]))
      .attr("r", 2.2)
      .attr("class", "observation");

    const line = d3.line<[number, number]>()
      .x((d) => x(d[0]))
      .y((d) => y(d[1]));

    for (const track of state.tracks) {
      const isSelected = track.id === selectedTrackId;
      const dimUnselected =
        viewMode === "focus" &&
        selectedTrackId !== null &&
        !isSelected;

      const trackOpacity = dimUnselected ? 0.22 : 1.0;

      svg.append("path")
        .datum(track.trail)
        .attr("d", line)
        .attr(
          "class",
          isSelected
            ? "track-trail track-trail-selected"
            : "track-trail",
        )
        .attr("opacity", trackOpacity);

      const [xx, xy, yy] = track.covariance2d;
      const trace = xx + yy;
      const detTerm = Math.sqrt(
        Math.max(0, (xx - yy) ** 2 + 4 * xy * xy),
      );
      const lambda1 = Math.max(1e-9, (trace + detTerm) / 2);
      const lambda2 = Math.max(1e-9, (trace - detTerm) / 2);
      const angle =
        0.5 * Math.atan2(2 * xy, xx - yy) * 180 / Math.PI;

      const pxPerMeter = Math.abs(x(1) - x(0));
      const sigmaScale = 2.4477;

      svg.append("ellipse")
        .attr("cx", x(track.position[0]))
        .attr("cy", y(track.position[1]))
        .attr("rx", sigmaScale * Math.sqrt(lambda1) * pxPerMeter)
        .attr("ry", sigmaScale * Math.sqrt(lambda2) * pxPerMeter)
        .attr(
          "transform",
          `rotate(${-angle} ${x(track.position[0])} ${y(track.position[1])})`,
        )
        .attr("class", "covariance")
        .attr("opacity", trackOpacity);

      svg.append("circle")
        .attr("cx", x(track.position[0]))
        .attr("cy", y(track.position[1]))
        .attr("r", isSelected ? 8 : 6)
        .attr(
          "class",
          isSelected
            ? "track-point track-point-selected"
            : "track-point",
        )
        .attr("opacity", trackOpacity)
        .style("cursor", "pointer")
        .style("pointer-events", "all")
        .on("pointerdown", () => onSelectTrack?.(track.id));

      svg.append("text")
        .attr("x", x(track.position[0]) + 10)
        .attr("y", y(track.position[1]) - 10)
        .attr(
          "class",
          isSelected
            ? "track-label track-label-selected"
            : "track-label",
        )
        .attr("opacity", trackOpacity)
        .style("cursor", "pointer")
        .style("pointer-events", "all")
        .text(track.label)
        .on("pointerdown", () => onSelectTrack?.(track.id));
    }

    for (const sensor of state.sensors) {
      const dimSensor = viewMode === "focus";

      svg.append("circle")
        .attr("cx", x(sensor.position[0]))
        .attr("cy", y(sensor.position[1]))
        .attr("r", 7)
        .attr(
          "class",
          `sensor sensor-id-${sensor.id} sensor-${sensor.health.toLowerCase()}`,
        )
        .attr("opacity", dimSensor ? 0.35 : 1.0);

      svg.append("text")
        .attr("x", x(sensor.position[0]) + 10)
        .attr("y", y(sensor.position[1]) + 4)
        .attr("class", "sensor-label")
        .attr("opacity", dimSensor ? 0.35 : 1.0)
        .text(sensor.label);
    }
  }, [
    state,
    selectedTrackId,
    onSelectTrack,
    viewMode,
  ]);

  return (
    <div className="panel tracking-panel">
      <div className="panel-heading">
        <span>TRACKING FIELD</span>

        <div className="tracking-heading-actions">
          <span className="muted">
            ENU / top-down · click track to inspect
          </span>

          <div className="view-toggle-group">
            <button
              type="button"
              className={viewMode === "tracks" ? "control-active" : ""}
              onClick={() => setViewMode("tracks")}
            >
              TRACKS
            </button>

            <button
              type="button"
              className={viewMode === "focus" ? "control-active" : ""}
              disabled={selectedTrackId === null}
              onClick={() => setViewMode("focus")}
            >
              FOCUS SELECTED
            </button>

            <button
              type="button"
              className={viewMode === "full" ? "control-active" : ""}
              onClick={() => setViewMode("full")}
            >
              FULL FIELD
            </button>
          </div>
        </div>
      </div>

      <svg
        ref={ref}
        viewBox="0 0 900 520"
        className="tracking-canvas"
        role="img"
        aria-label="Aurora live tracking field"
      />
    </div>
  );
}
