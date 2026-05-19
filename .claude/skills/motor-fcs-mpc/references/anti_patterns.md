# Common Mistakes (Anti-Pattern Catalog)

Each row: a known mistake, its symptom, and the fix.

| # | Mistake | Symptom | Fix |
|---|---|---|---|
| 1 | Forgot ZOH on **one** chart input | `wm` < 50% target despite algorithm correct | Add ZOH @ Tsc to every chart input port without exception (A-CRIT) |
| 2 | Used DISCRETE+Tsc on long chart | Compile error "Cannot determine size and/or data type of output port" | Switch to INHERITED + (-1) + dual ZOH (G-CRIT) |
| 3 | Forced `ch.Inputs(k).DataType='double'` or `Size='1'` | Same as #2 | Delete those `set_param` calls; leave Inputs at Inherit |
| 4 | PI without saturation | `\|mean(id) − id_ref\| > 0.5 A` (drift, not ripple) with `id_ref=0`; `iabc` large oscillation | Add `LimitOutput='on'` to Discrete PID with sized `iq_max` (D-CRIT). Distinguish from healthy FCS-MPC ripple via the ratio `\|mean(id)\| / mean\|id\|` — < 0.4 means ripple-dominated and is normal |
| 5 | MPC params in external `.m` file | Plant params changed but predictions don't match | Move all params into chart Script via `sprintf` (K-CRIT); delete the external `.m` |
| 6 | Used Step + RateLimiter for `ω_ref` (default Continuous) | Steady-state PASS but transient ramp is 10× too fast | Switch to From Workspace inline matrix (H-CRIT), OR set RateLim `SampleTimeMode='specified'`, `SampleTime='Tsc'` |
| 7 | Used Signal Editor with external `.mat` | Model not self-contained; reload+sim fails in fresh session | Use From Workspace inline matrix |
| 8 | `assignin` only, no InitFcn | `clear all` + reload+sim fails with "undefined parameter" | Add `set_param(mdl, 'InitFcn', sprintf(...))` before `save_system` (J-CRIT) |
| 9 | PI gains designed in rad/s but Sum block in RPM domain | System tracks ~10× too slowly (factor of `30/π ≈ 9.55`) | Either redesign gains for RPM domain, or convert PI feedback path: keep all in rad/s |
| 10 | `OutputAfterFinalValue = 'Hold final value'` (no `-ing`) | Simulink fuzzy-matches but emits warning every sim | Use canonical `'Holding final value'` (with `-ing`) |
| 11 | `From Workspace` `SampleTime='-1'` | Back-inheritance warning every sim | Use `SampleTime='0'` (Continuous) |
| 12 | Forgot output ZOH between chart and Universal Bridge | Inverter receives gate at chart's natural rate (not Tsc); switching pattern unstable | Add `ZOH @ Tsc` between chart output and UB input port |
| 13 | Used PMSM bus `theta` for Park transform | Inconsistent results in some R2024b configs | Integrate `Pn·w` inside chart with persistent var (D13) |
| 14 | Tried Fixed-step solver with SimPowerSystems | Compile/init errors | Use Variable-step Auto + powergui Discrete @ Ts (D15) |
| 15 | Build script not idempotent (re-run fails) | Second `run('rebuild_build.m')` errors on `add_block` collision | Add `if bdIsLoaded(mdl_name); close_system(mdl_name, 0); end` and `delete(mdl_path)` at script start |
