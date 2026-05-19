# Theory Anchor (Phase 1.5)

Before any build, anchor the method's theoretical formulas. Every formula used downstream must be **signed off** (or AI-self-audited if the user lacks domain expertise).

## Signing SOP

Each formula gets **its own** sign-off (no batch signing). Required defenses:

1. **Source citation**: DOI, paper title, author, year
2. **Abstract / page reference**: page number + equation number from the source
3. **Dependency chain**: which previously-signed formulas it depends on
4. **Sanity check**: dimensional analysis + at least one boundary condition

## Already-signed PMSM formula index

These are in `shared/formulas/pmsm_formulas.md`. Do **not** re-sign or re-derive when starting a new PMSM method — reference them by section:

| Section | Content |
|---|---|
| `pmsm_formulas.md #0–#7` | PMSM dq plant: voltage equations, electromagnetic torque, mechanical equation, electrical/mechanical angle relation |
| `pmsm_formulas.md §A` | PI controller design methods (Pole-Zero Cancellation, Symmetric Optimum, Modulus Optimum, Loop Shaping) |
| `pmsm_formulas.md §B` | DTC: αβ flux estimation, electromagnetic torque, hysteresis comparators, sector identification, switching table |
| `pmsm_formulas.md §C` | SMC: sliding surface, reaching law, Lyapunov analysis, boundary layer |

For a new PMSM method (e.g., sensorless, MTPA, weak-field):
1. Audit coverage: which formulas already exist vs. need to be added vs. not needed
2. Sign **only the new** formulas; existing signed sections are reused as-is

## AI-Self-Audit Mode

Use when the user does not have independent domain expertise to verify formulas (typical for advanced SMC variants, sensorless observers, complex MTPA strategies).

### Switch protocol

1. At Phase 1.5 start, ask: "Do you have the domain expertise to independently verify these formulas?"
2. If "no", switch to AI-self-audit mode and document: header on the formula section reads `AI-self-audit with verifiable sources, user domain knowledge gap declared YYYY-MM-DD`.
3. Each formula must satisfy:
   - **≥ 2 independent cross-check sources** (e.g., one journal paper + one textbook, or two independent arxiv preprints)
   - **≥ 1 link** the user can click to verify
   - **≥ 1 PDF cached locally** for the user's offline inspection

### Fail-safe replacement for user sign-off

When the user cannot sign formulas:
- Use AI-self-audit + multi-source cross-check at Phase 1.5
- Compensate at acceptance time: a final user visual review on the simulated waveforms (does the controller actually do what the theory promises?) acts as the human-in-the-loop check that user formula sign-off would have provided.

## Dimensional analysis (always)

Even with full sign-off, every equation must pass dimensional analysis. Common pitfalls:

- ω_e (rad/s) vs. θ_e (rad): used wrong, dimensions silently look OK if you forget time
- Lq, Ld in henry vs. millihenry: factor 1000 hides easily in numerical results
- ψ_f in V·s vs. Wb: same units (Wb = V·s) but library blocks may expect specific naming

When in doubt, write out units in MATLAB comments:

```matlab
omega_e   = Pn * omega_m;          % rad/s = (1) * rad/s
BEMF_peak = omega_e * psi_f;       % V    = (rad/s) * (V*s)
```
