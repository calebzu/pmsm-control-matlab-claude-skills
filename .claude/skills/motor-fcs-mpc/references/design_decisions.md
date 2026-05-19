# Design Decisions Checklist

For each decision, present the user with options + your recommended default, then get explicit confirmation.

| ID | Decision | Recommended | Alternatives |
|---|---|---|---|
| D01 | `ω_ref` source | From Workspace + inline matrix (H-CRIT) | Step + RateLimiter + ZOH only if From Workspace unavailable; **must** set RateLim `SampleTimeMode='specified'`, `SampleTime='Tsc'` |
| D02 | Plant topology | SimPowerSystems (DC + UB + PMSM) | Math dq (NOT recommended — no gate→voltage→current physical chain for FCS-MPC) |
| D03 | PMSM rotor type | Salient-pole (handles both `Ld=Lq` SPMSM and `Ld<Lq` IPMSM) | Round (legacy, less flexible) |
| D04 | Cost function weights | Common starting point in balanced range: `λ_d=1, λ_q=20`. Confirm objective with user. | `λ_q/λ_d ∈ [5, 10]` ripple-priority; `[10, 30]` balanced; `[50, 100]` torque-priority; symmetric `1:1` for ripple research |
| D05 | Outer PI saturation | **Mandatory** `[-iq_max, +iq_max]` (D-CRIT) | None ❌ NOT allowed |
| D06 | PI domain | RPM (engineering convention) | rad/s (must redesign gains; equivalent if scaled by `30/π`) |
| D07 | Action space | Ask user (7 or 8 equivalent — 7 saves ~12% loop count via V0/V7 dedup; 8 is textbook full enumeration) | 7-vector dedup `[V0..V6]` (skip V7=[1 1 1] electrically identical to V0=[0 0 0]) **OR** 8-vector full `[V0..V7]` explicit. Both yield mathematically identical minimum cost. |
| D08 | Discretization | Forward Euler N=1 | Backward Euler / Tustin / multi-step (out of scope) |
| D09 | Delay compensation | None | 1-step / multi-step (out of scope) |
| D10 | `id_ref` strategy | SPMSM: `id_ref = 0`. IPMSM: ask user. | MTPA, weak-field (out of scope) |
| D11 | Chart config | INHERITED + `(-1)` + dual ZOH (G-CRIT) | DISCRETE+Tsc ❌ deadlocks long charts |
| D12 | Chart input ZOH | Every input ZOH @ Tsc (A-CRIT) | None ❌ breaks persistent integrator step assumption |
| D13 | `θ_e` source | Integrate `Pn·w` with persistent var inside chart | PMSM bus `theta` via Discrete Integrator outside chart (subject to bus consistency caveats) |
| D14 | MPC param source | Chart hardcode + `sprintf` from build-script workspace (K-CRIT) | External `.m` file ❌ drifts from plant; chart input port ❌ deprecated |
| D15 | Solver | Variable-step Auto + powergui Discrete @ Ts | `ode23tb` for very stiff cases; Fixed-step ❌ usually conflicts with SPS |
| D16 | Logging | `To Workspace`, `SaveFormat='StructureWithTime'`, `SampleTime='Tsc'` | Outport + `sim()` return (more boilerplate); Scope only (no `.mat` persistence) |
| D17 | InitFcn injection | Mandatory (J-CRIT) | `assignin` only ❌ not self-contained |
