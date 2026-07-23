# Channel Gate Control

Closed-loop height control of a hydraulically actuated channel gate.

An operator drags a **Target Height** slider in the Doover UI. The app reads the
*actual* gate height published by the [`channel-gate-encoder`](../channel-gate-encoder)
app and drives one of two solenoid valves — **raise** or **lower** — via digital
outputs to close the error, stopping within a deadband. It's the control half of
the encoder/controller pair: the encoder measures, this app acts.

```
  operator slider ──▶ Target Height (mm)
  encoder ──────────▶ Gate Height  (mm)   ┐
                                          ├─▶ error ─▶ RAISE or LOWER solenoid (DO)
                      deadband/hysteresis ┘            (never both — hard interlock)
```

## How it works

- **Bang-bang with deadband + hysteresis.** When the error (target − actual)
  exceeds `deadband + hysteresis`, the app energises the solenoid that moves the
  gate toward the target. It stops once the gate is within `deadband`. The
  hysteresis gap stops the valves chattering when the gate drifts slightly off
  target.
- **Hard interlock.** Both solenoid outputs are written through a single choke
  point as one atomic `set_do` transaction, so the raise and lower solenoids can
  **never** be energised at the same time (which on a double-acting cylinder
  means dead-heading the hydraulics).
- **Fail-safe.** Both solenoids are de-energised on hold, on loss of the height
  signal, on a latched fault, and on app shutdown. The startup routine also
  drives both outputs to their safe (de-energised) state before any control
  runs — important for active-low wiring.

### Safety trips (latch a fault → both outputs off until Reset)

| Trip | Condition |
|------|-----------|
| **Move timeout** | A solenoid has been energised continuously for `move_timeout_s` without reaching target. |
| **Stall / wrong way** | While moving, the gate hasn't progressed at least `stall_min_progress_mm` in the commanded direction within `stall_window_s`. Catches a jam, a dead/frozen encoder, or **reversed solenoid wiring** (gate moving away from target). |

Loss of the height signal (`None`) is **not** latched — it holds the outputs off
and resumes automatically when the signal returns.

## Getting started (bench simulator)

The simulator closes the physical loop with no hardware: a small "gate physics"
app reads this controller's solenoid-drive tags and integrates a virtual gate
height, publishing it back as the `Height` tag the controller reads.

```bash
doover app run        # docker compose up in simulators/
```

Then open the app in the Doover UI:

1. Set **Control Mode** → **Auto**.
2. Drag **Target Height** to a value.
3. Watch **Raise/Lower Solenoid** energise and **Gate Height** converge on the
   target, then the solenoids drop out inside the deadband.

The gate starts at 400 mm with 1000 mm of travel at 80 mm/s (tunable in
`simulators/docker-compose.yml`). Mode defaults to **Hold** so nothing moves
until you deliberately switch to Auto.

## Configuration

| Key | Default | Purpose |
|-----|---------|---------|
| `height_app_key` | — | The channel gate encoder app supplying the height feedback. |
| `height_tag_name` | `Height` | Tag (mm) read from that app as the actual gate height. |
| `height_min_mm` / `height_max_mm` | `0` / `1000` | Travel limits — the target slider span. |
| `raise_do_pin` / `lower_do_pin` | — | Digital outputs for the raise / lower solenoids. |
| `do_active_low` | `false` | Set if solenoids energise on a LOW output. |
| `deadband_mm` | `5` | Stop tolerance around the target. |
| `hysteresis_mm` | `5` | Extra error before a stopped gate re-engages (anti-chatter). |
| `control_period_s` | `0.25` | Control-loop period. |
| `outputs_enabled` | `true` | Master enable; off ⇒ both solenoids held de-energised. |
| `move_timeout_s` | `30` | Max continuous energise time per move before faulting. `0` disables. |
| `stall_window_s` | `4` | Window the gate must show progress within while moving. `0` disables. |
| `stall_min_progress_mm` | `3` | Minimum progress expected per stall window. |

## Tags published (live, for the HMI and peers)

`TargetHeight`, `GateHeight`, `Error`, `Moving` (`raising`/`lowering`/`idle`),
`RaiseOutput`, `LowerOutput`, `Mode`, `Status`, `Fault`, `FaultReason`,
`HeightValid`.

## Regenerating `doover_config.json`

```bash
uv run export-config                                        # config schema
uv run python -c "from channel_gate_control.app_ui import export; export()"  # UI schema
```
