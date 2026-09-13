from __future__ import annotations

import asyncio
import json
import time

import websockets

from simulation.operator_simulation import DT, HOST, PORT, OperatorSimulation

simulation = OperatorSimulation()

clients: set[asyncio.Queue[str]] = set()
latest_payload: str | None = None


async def client_handler(websocket) -> None:
    queue: asyncio.Queue[str] = asyncio.Queue(maxsize=2)
    clients.add(queue)

    print(
        f"[ws] client connected; subscribers={len(clients)}",
        flush=True,
    )

    if latest_payload is not None:
        queue.put_nowait(latest_payload)

    async def sender() -> None:
        while True:
            payload = await queue.get()
            await websocket.send(payload)

    async def receiver() -> None:
        async for raw in websocket:
            simulation.command(json.loads(raw))

    sender_task = asyncio.create_task(sender())
    receiver_task = asyncio.create_task(receiver())

    try:
        done, pending = await asyncio.wait(
            {sender_task, receiver_task},
            return_when=asyncio.FIRST_COMPLETED,
        )

        for task in done:
            if task.cancelled():
                continue

            exc = task.exception()

            if exc is None:
                continue

            if isinstance(
                exc,
                websockets.exceptions.ConnectionClosedOK,
            ):
                continue

            raise exc

        for task in pending:
            task.cancel()

        await asyncio.gather(
            *pending,
            return_exceptions=True,
        )
    finally:
        sender_task.cancel()
        receiver_task.cancel()

        await asyncio.gather(
            sender_task,
            receiver_task,
            return_exceptions=True,
        )

        clients.discard(queue)

        print(
            f"[ws] client disconnected; subscribers={len(clients)}",
            flush=True,
        )


async def broadcast_loop() -> None:
    global latest_payload

    while True:
        started = time.perf_counter()

        state = simulation.step()
        latest_payload = json.dumps(state)

        for queue in list(clients):
            if queue.full():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass

            try:
                queue.put_nowait(latest_payload)
            except asyncio.QueueFull:
                pass

        elapsed = time.perf_counter() - started
        await asyncio.sleep(max(0.0, DT - elapsed))


async def main() -> None:
    print(
        f"Aurora operator state stream: ws://{HOST}:{PORT}",
        flush=True,
    )

    async with websockets.serve(
        client_handler,
        HOST,
        PORT,
    ):
        await broadcast_loop()
