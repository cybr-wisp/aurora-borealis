"use client";

import {useMemo} from "react";
import type {NeesSample} from "@/lib/types";

export function NEESPlot({samples}: {samples: NeesSample[]}) {
  const path = useMemo(() => {
    if (samples.length < 2) return "";
    const values = samples.slice(-120);
    const w = 520;
    const h = 160;
    const maxY = 20;
    return values.map((sample, i) => {
      const x = i / (values.length - 1) * w;
      const y = h - Math.min(maxY, sample.value) / maxY * h;
      return `${i === 0 ? "M" : "L"}${x.toFixed(1)},${y.toFixed(1)}`;
    }).join(" ");
  }, [samples]);

  const loY = 160 - 1.237 / 20 * 160;
  const hiY = 160 - 14.449 / 20 * 160;

  return (
    <div className="panel">
      <div className="panel-heading">
        <span>NEES</span>
        <span className="muted">95% χ² bounds</span>
      </div>
      <svg viewBox="0 0 520 160" className="nees-plot">
        <rect x="0" y={hiY} width="520" height={loY - hiY} className="nees-band" />
        <line x1="0" y1={hiY} x2="520" y2={hiY} className="bound-line" />
        <line x1="0" y1={loY} x2="520" y2={loY} className="bound-line" />
        <path d={path} className="nees-line" />
      </svg>
    </div>
  );
}
