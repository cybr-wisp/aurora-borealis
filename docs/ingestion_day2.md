# Day 2 — UDP/Protobuf ingestion and overload semantics

## Baseline measured on Windows Debug build

The September 5 baseline passed all 12 C++ tests before Day 5 work.

### Parse + enqueue microbenchmark

- payload: 69 bytes
- messages: 300,000
- throughput: **391,782 msg/s**
- p50: **2.0 µs**
- p95: **2.6 µs**
- p99: **3.0 µs**

### UDP end-to-end

| Case | Rate | Drops | p50 | p95 | p99 |
|---|---:|---:|---:|---:|---:|
| nominal_1k | 1,000 msg/s | 0 | 67.1 µs | 243.3 µs | 349.1 µs |
| nominal_5k | 5,000 msg/s | 0 | 35.3 µs | 195.3 µs | 292.0 µs |
| padded_5k_64 | 5,000 msg/s | 0 | 87.5 µs | 271.5 µs | 440.3 µs |
| padded_5k_256 | 5,000 msg/s | 0 | 129.1 µs | 5.195 ms | 19.657 ms |
| unpaced | 42,371 recv/s | 0 | 39.8 µs | 408.2 µs | 1.413 ms |
| saturation | 42,798 recv/s | 40,869 queue-full | 874.979 ms | 970.268 ms | 972.312 ms |

The saturation result is intentionally a failure characterization: the
receiver never blocks, but a slow consumer can build nearly one second of
stale backlog.

## Day 2 hardening added

`SpscRing` now exposes approximate depth and a high-water mark. The consumer
may shed the oldest backlog when depth exceeds a threshold. Only the consumer
advances `tail_`, so the SPSC ownership invariant remains intact.

The engine now reports separate counters for:

- malformed Protobuf
- duplicate/old sequence
- stale timestamp
- invalid measurement
- producer queue-full drops
- consumer oldest-backlog shedding
- queue high-water mark
- sensor timeout and recovery events

Run:

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
.\build\Debug\bench_ingestion.exe
.\build\Debug\bench_udp_e2e.exe
.\build\Debug\bench_backpressure.exe
```

The backpressure benchmark is the post-hardening counterpart to the original
saturation case.
