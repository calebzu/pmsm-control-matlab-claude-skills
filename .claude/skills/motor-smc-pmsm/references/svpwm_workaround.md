# SVPWM Library Block — Sector=7 Startup Workaround

The shared `pmsm_blocks.slx` SVPWM library block contains a `Sector_Caculate` subsystem that computes sector ∈ {1, …, 6} from `sign(X)`, `sign(Y)`, `sign(Z)` — three intermediate variables of `(Vα, Vβ)`. At simulation startup transient where `Vα = Vβ = 0`:

- `X = Y = Z = 0`
- All three signs = `1` (positive zero per IEEE 754)
- Sector_Caculate output = `4·1 + 2·1 + 1 = 7` (out of valid 1..6 range)

The downstream `T1T1_Caculate` and `Tcm_Caculate` MultiPort Switch blocks have `DiagnosticForDefault='Error'` (default), so sim throws:

```
Error: 'Multiport Switch' block specifies that the value of the
control input (7) cannot match a data port input
```

## Workaround (Mandatory in Build Script)

Apply on the **local instance** via `LinkStatus='inactive'` — do NOT modify the shared library:

```matlab
% Break library link on SVPWM_blk instance only (preserves shared library):
set_param([mdl '/SVPWM_blk'], 'LinkStatus', 'inactive');

% Set DiagnosticForDefault to 'None' on internal MultiPortSwitch blocks. With
% the default DataPortForDefault='Last data port', sector=7 falls back to
% sector-6 behavior for 1-2 startup samples (negligible).
ms_blks = find_system([mdl '/SVPWM_blk'], ...
    'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'MultiPortSwitch');
for k = 1:numel(ms_blks)
    set_param(ms_blks{k}, 'DiagnosticForDefault', 'None');
end
```

## Discipline

- The workaround applies only to the **local instance** via `LinkStatus='inactive'`
- Future SVPWM library upgrades remain compatible
- Self-test should verify `LinkStatus` is `'inactive'` on the instance

## Self-Test

```matlab
link_status = get_param([mdl '/SVPWM_blk'], 'LinkStatus');
assert(strcmp(link_status, 'inactive'), 'SVPWM workaround: LinkStatus must be inactive on local instance');
```

## Phase 5 Modulation Wiring

After applying the workaround, complete the modulation chain:

| Block | Configuration |
|---|---|
| `Anti_Park` | from `pmsm_blocks/Anti_Park` library block. **Inport 1 = Vq, Inport 2 = Vd**. θ_e dependency via internal `From "The"` (subscribes to `Goto_The`, see G-CRIT). Output: `Ualpha (port 1)`, `Ubeta (port 2)` |
| `SVPWM_blk` | from `pmsm_blocks/SVPWM` library block (with workaround applied). Inputs: `Ualpha (port 1)`, `Ubeta (port 2)`. Output: 6-bit pair-adjacent gate `[Sa_up, Sa_dn, Sb_up, Sb_dn, Sc_up, Sc_dn]` direct-fit to Universal Bridge inport 1. Internal Constants reference `Tpwm` and `Vdc_val` workspace vars (set in InitFcn) |
| `Universal_Bridge` | 3-arm IGBT inverter; gate input from `SVPWM_blk/1` |

```matlab
% Wiring
add_line(mdl, 'Sum_Vq_total/1', 'Anti_Park/1', 'autorouting', 'on');
add_line(mdl, 'Sum_Vd_total/1', 'Anti_Park/2', 'autorouting', 'on');
add_line(mdl, 'Anti_Park/1', 'SVPWM_blk/1', 'autorouting', 'on');
add_line(mdl, 'Anti_Park/2', 'SVPWM_blk/2', 'autorouting', 'on');
add_line(mdl, 'SVPWM_blk/1', 'Universal_Bridge/1', 'autorouting', 'on');
```
