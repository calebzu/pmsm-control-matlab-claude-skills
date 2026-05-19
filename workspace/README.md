# Workspace

This directory is your sandbox. Put your built `.slx` models, build scripts,
parameter sweeps, and experiment artifacts here. The repository tooling treats
everything inside `workspace/` as user-owned and will not write to it
automatically.

Conventions:

- `shared/` (one level up) is read-only — reference its formulas and building
  blocks instead of duplicating them.
- One folder per experiment or method works well, e.g. `workspace/fcs_mpc_run1/`.
- Build scripts should inject all parameters via
  `set_param(mdl, 'InitFcn', ...)` so the `.slx` runs from a fresh MATLAB
  session with one click.
- Do not commit large simulation outputs (`.mat`, traces) — keep the
  repository small by listing them in `.gitignore` if needed.
