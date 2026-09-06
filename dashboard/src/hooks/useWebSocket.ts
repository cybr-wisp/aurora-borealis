"use client";

import {useCallback, useEffect, useRef, useState} from "react";
import type {DashboardCommand, WorldState} from "@/lib/types";

const DEFAULT_URL =
  process.env.NEXT_PUBLIC_AURORA_WS_URL ?? "ws://127.0.0.1:8765";

export function useAuroraSocket() {
  const [state, setState] = useState<WorldState | null>(null);
  const [connected, setConnected] = useState(false);
  const socketRef = useRef<WebSocket | null>(null);
  const retryRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    let closed = false;

    const connect = () => {
      if (closed) return;

      const socket = new WebSocket(DEFAULT_URL);
      socketRef.current = socket;

      socket.onopen = () => setConnected(true);
      socket.onclose = () => {
        setConnected(false);
        if (!closed) {
          retryRef.current = setTimeout(connect, 1000);
        }
      };
      socket.onerror = () => socket.close();
      socket.onmessage = (event) => {
        const parsed = JSON.parse(event.data) as WorldState;
        setState(parsed);
      };
    };

    connect();

    return () => {
      closed = true;
      if (retryRef.current) clearTimeout(retryRef.current);
      socketRef.current?.close();
    };
  }, []);

  const send = useCallback((command: DashboardCommand) => {
    const socket = socketRef.current;
    if (socket?.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify(command));
    }
  }, []);

  return {state, connected, send};
}
