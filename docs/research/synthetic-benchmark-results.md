# Synthetic tracking-core benchmark

Observed run: **2026-11-15 03:34:56 UTC**. This measures a deterministic software workload, not physical tracking accuracy, sensor/network latency, or performance relative to SlimeVR.

This timing snapshot preceded the addition of explicit firmware-held provenance. The benchmark source now marks its inputs as actual synthetic measurements; it does not exercise or validate held firmware input behavior. Current held-input correctness is covered separately by targeted regressions. These recorded timing values are retained as the observed baseline run rather than silently presented as a new measurement of later source changes.

Machine: AMD Ryzen 9 5900HX with Radeon Graphics; Windows 11 Home Insider Preview 10.0.26220; x86-64; `rustc 1.100.0 (0ef1bc386 2026-11-09)`. Release build used the workspace optimization settings. The desktop was not an isolated performance laboratory: no process affinity, fixed power profile or OS scheduling control was applied.

Reproduce from the workspace root:

```powershell
cargo run --release -p tracking-core --example benchmark -- --json local-data/benchmarks/tracking-core-synthetic.json
```

Implementation: [benchmark.rs](../../crates/tracking-core/examples/benchmark.rs). The JSON report is generated locally under the gitignored `local-data/benchmarks` directory and includes per-trial timing summaries, parameter values, methodology and error statistics.

## Timing

Each case used three independently calibrated trials. Every trial ran 1,000 untimed moving warmup frames, then 10,000 timed frames. Case order rotated between trials. Synthetic timestamps advanced at exactly 100 Hz, but the benchmark executed without real-time pacing.

The timer covers `solve` and returned-frame construction. It excludes input creation/ingestion, caller output destruction, serialization, UI rendering, Wi-Fi, firmware, SteamVR driver work and the display pipeline. Elapsed wall time can include allocator behavior and scheduling interruptions. Percentiles use linear interpolation at `p × (N−1)`; these are observed distributions, not confidence intervals.

| Input sensors | Experimental correction | p50 (µs) | p95 (µs) | p99 (µs) | Maximum (µs) | Valid timed frames |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 6 | Off | 1.5 | 2.4 | 3.0 | 76.6 | 30,000 / 30,000 |
| 6 | On | 2.1 | 3.8 | 4.6 | 145.9 | 30,000 / 30,000 |
| 10 | Off | 1.8 | 3.1 | 3.7 | 100.4 | 30,000 / 30,000 |
| 10 | On | 2.3 | 4.4 | 6.0 | 73.5 | 30,000 / 30,000 |

Timer-pair p50 was 0.0 µs and p99 was 0.1 µs. The zero median reflects timer quantization and does not mean measuring time is free; overhead was not subtracted. No measured solver percentile is an end-to-end latency result.

## Synthetic estimator behavior

Four paired scenarios ran for 120 seconds of synthetic time. Baseline and experimental runs received identical input; only the experimental toggle differed. Thresholds and the 0.2 degree/second relative correction limit used the default configuration. The following mean absolute heading errors cover the final ten simulated seconds. Headings are projected local segment X axes against analytic ground truth.

| Scenario | Estimator | Relative knee error | Thigh error | Calf error | Pair common heading error |
| --- | --- | ---: | ---: | ---: | ---: |
| Calf measurement drifts to +15°; true motion is a hinge | Baseline | 15.0° | 0.0° | 15.0° | 7.5° |
| Same injected drift | Experimental | 6.0° | 4.5° | 10.5° | 7.5° |
| Same observations interpreted as true non-hinge calf rotation | Baseline | 0.0° | 0.0° | 0.0° | 0.0° |
| Same non-hinge ground truth | Experimental | 9.0° | 4.5° | 4.5° | 0.0° |
| All body IMUs drift together by +15° | Baseline | 0.0° | 15.0° | 15.0° | 15.0° |
| Same common-mode drift | Experimental | 0.0° | 15.0° | 15.0° | 15.0° |
| True common whole-body turn, no sensor error | Baseline | 0.0° | 0.0° | 0.0° | 0.0° |
| Same true turn | Experimental | 0.0° | 0.0° | 0.0° | 0.0° |

In the injected calf-drift case, relative knee heading RMSE over the complete sequence decreased from 12.2478° to 5.9713°. The correction shares the disagreement between thigh and calf, so the originally accurate thigh acquires error. It does not identify the faulty sensor or remove the pair's common heading error.

The non-hinge case deliberately uses the same observations with a different ground-truth interpretation. The model incorrectly changes that true motion, creating 9° relative error. This is a mathematical counterexample, not a validated biomechanical knee model; it exposes the ambiguity of the hinge assumption. It supports keeping the feature experimental and off by default until measurements establish useful operating conditions.

All expected targets remained valid in these ideal streams. That only checks software behavior with fresh, well-formed synthetic inputs. Physical packet timing, sparse firmware updates, mounting slip, sensor noise, motion artifacts and real calibration quality are outside this benchmark. The [research evaluation plan](tracking-research.md) describes the hardware and comparative work needed before an accuracy or superiority claim.
