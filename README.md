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

### Top limit prox (over-travel guard + height calibration)

An optional proximity sensor at the top of the gate, wired to `estop_di_pin`.
Normally-open by default (active HIGH); set `estop_active_low` for a
normally-closed prox. It is **not** a trip — it does three things while it reads
active:

- **Blocks raising.** The raise solenoid and the pump are held off, enforced at
  the same output choke point as the interlock, so no code path can drive the
  gate further up.
- **Still allows lowering.** The operator can always close the gate off the
  limit; nothing latches and no Reset is needed. The block lifts by itself on a
  confirmed inactive reading.
- **Re-zeros the gate height.** The prox sits at a known point of travel, so
  arriving there is what calibrates the measurement: the height is offset to read
  `estop_height_mm` — **520 mm**, the prox's real height on this gate. Whatever
  the encoder had drifted to, touching the prox pins it back. This happens **once
  per arrival**, so a gate creeping while parked on the sensor can't keep
  re-zeroing and hide it. The offset is published as `HeightOffset` and restored
  on restart.

⚠️ `estop_height_mm` has to agree with the travel limits: heights are measured
upward from the closed gate, so the datum is the prox's own height (520 mm), not
`0`. A `0` there would make every position below the prox read negative, outside
the target slider's span, and the gate could not then be commanded downward.
`height_max_mm` defaults to the same 520 mm so the slider can't ask for a height
the prox will refuse to raise to — move the two together if the sensor is
repositioned.

Detection is **level-driven**: `main_loop` polls the DI every cycle, before any
control decision, and the pulse-counter edge callback is only a fast path for an
immediate stop. A failed read holds the last state rather than releasing the
block. The edge callback reads the pin level back rather than trusting the
callback's `value`, which pydoover delivers as `False` on every driver.

### Local control (momentary switch at the gate)

A momentary switch at the gate puts **12 V** on an analog input — `manual_raise_ai_pin`
for up, `manual_lower_ai_pin` for down. While it's held the gate jogs that way; on
release it stops. Only **AI0** and **AI1** can do this (they're the pins that
support the voltage-step detection), so both keys are capped at `1`.

**It is armed by default** — AI0 for raise, AI1 for lower, the standard wiring
for this gate. The platform backfills schema defaults into deployments that
don't carry the keys, so on upgrade every install gets local control on AI0/AI1
without a config change. If a site has no switch fitted (or something else wired
to those pins), set the pin to `null` explicitly: anything putting 6 V+ on an
armed pin will move the gate, above the fault latch.

- **It outranks a latched fault**, on purpose. The switch is the on-site recovery
  path: it has to work in exactly the situations Auto refuses to move in — dead
  or unhomed encoder, latched stall or move timeout — and the operator standing
  at the gate watching it is the safety case that stands in for the move-timeout
  and stall detector (both of which need a trusted height anyway). The latch
  itself is untouched: Auto stays off until Reset.
- **It works in either mode.** Hold only means "don't chase the setpoint".
- **`outputs_enabled` still wins.** The master enable is the one gate above it.
- **Both switches held ⇒ nothing moves**, resolved before the output choke point
  so the interlock never has to catch an operator action.
- **The top limit still hard-blocks a manual raise**, and still leaves manual
  lower available — including on the press fast path, which polls the prox itself
  rather than waiting for the first control period to do it.
- On release, Auto simply resumes on the next pass — deadband and hysteresis
  re-engage as normal, no reset and no extra state. The resumed move is always a
  **fresh** move, never a hunting continuation of the one the jog interrupted: a
  jog moves the gate an arbitrary distance, so inheriting the old move clock
  would fault it on a move timeout it never earned.

Press detection is a **voltage-step** pulse counter on the analog input
(`VI+<threshold>`, fired by the 0 → 12 V jump), and release is a **level poll** in
`main_loop`, so the worst-case stop latency is one control period — the same as
every other stop here. It has to be that way round: the firmware holds one
threshold per pin, so a pin streaming presses cannot also stream releases. The
poll is also the backstop for a press the stream drops, and a **failed read
releases** both switches — the opposite of the top limit's hold-last-state,
because an input that *drives* an output must drop out when it can't be read. A
level list that doesn't match the pins asked for counts as a failed read too,
rather than being zipped and misattributed, and clearing the pins at runtime drops
any switch that was reading pressed.

`manual_poll_s` sets the firmware sample rate. Leave it at `0.4` and the legacy
bare edge string is emitted, for older platform interfaces that crash parsing an
explicit rate.

`manual_threshold_v` never falls back to `0`: an unreadable value takes the 6 V
schema default, because a 0 V threshold would read every level — a released
switch included — as held down. A threshold explicitly at or below `0` disables
local control outright (nothing armed, both flags forced released) and says so in
the log at startup.

If the prox is what calibrates this gate, turn `require_homed` **off** —
otherwise the outputs stay held waiting for the encoder to home and the gate can
never be driven up to the prox in the first place.

## Cloud UI

Two tabs, **Control** open by default:

| Tab | Contents |
|-----|----------|
| **Control** | Target Height (slider), Control Mode (Hold / Auto), Gate Height, Height Calibration — what you need to run the gate, and nothing else. |
| **Diagnostics** | Target as the app read it, Error, Movement, Status, Raise / Lower / Pump outputs, the two local switch states, Fault Reason and **Reset Fault**. |

The four warning indicators — no height signal, height not trusted, top limit,
fault — sit **outside** the tabs, so they're visible whichever tab is open.

## Getting started (bench simulator)

The simulator closes the physical loop with no hardware: a small "gate physics"
app reads this controller's solenoid-drive tags and integrates a virtual gate
height, publishing it back as the `Height` tag the controller reads.

```bash
doover app run        # docker compose up in simulators/
```

Then open the app in the Doover UI:

1. On **Control**, set **Control Mode** → **Auto**.
2. Drag **Target Height** to a value, and watch **Gate Height** converge on it.
3. On **Diagnostics**, watch **Raise/Lower Solenoid** energise, then drop out
   once the gate is inside the deadband.

The gate starts at 400 mm with 1000 mm of travel at 80 mm/s (tunable in
`simulators/docker-compose.yml`). Mode defaults to **Hold** so nothing moves
until you deliberately switch to Auto. The local switches are armed on AI0/AI1
(the defaults) so the manual path can be exercised on the bench.

## Configuration

| Key | Default | Purpose |
|-----|---------|---------|
| `height_app_key` | — | The channel gate encoder app supplying the height feedback. |
| `height_tag_name` | `Height` | Tag (mm) read from that app as the actual gate height. |
| `require_homed` | `true` | Hold the outputs off until the encoder reports `Homed`. Turn off when the top limit prox is what calibrates the gate. |
| `heartbeat_timeout_s` | `15` | Treat the height as stale (and hold outputs) if the encoder's `Heartbeat` stops advancing. `0` disables. |
| `height_min_mm` / `height_max_mm` | `0` / `520` | Travel limits — the target slider span. Keep the top in step with `estop_height_mm`. |
| `raise_do_pin` / `lower_do_pin` | — | Digital outputs for the raise / lower solenoids. |
| `pump_do_pin` | — | Digital output for the hydraulic pump; energised whenever either solenoid is. |
| `do_active_low` | `false` | Set if solenoids energise on a LOW output. |
| `deadband_mm` | `5` | Stop tolerance around the target. |
| `hysteresis_mm` | `5` | Extra error before a stopped gate re-engages (anti-chatter). |
| `control_period_s` | `0.25` | Control-loop period. |
| `estop_di_pin` | — | Digital input from the top limit prox. Unset ⇒ not fitted. |
| `estop_active_low` | `false` | Clear for a normally-open prox (active HIGH), set for normally-closed. |
| `estop_height_mm` | `520` | Gate height the prox sits at — the calibration datum the height is re-zeroed to. |
| `manual_raise_ai_pin` / `manual_lower_ai_pin` | `0` / `1` | Analog inputs the local momentary raise / lower switches feed 12 V into. AI0/AI1 only (`0`–`1`). Armed by default; set to `null` if no switch is fitted on that direction. |
| `manual_threshold_v` | `6` | Volts at which a local switch reads as pressed, and the step the press detector is armed with. `0` or below disables local control rather than reading every level as pressed. |
| `manual_poll_s` | `0.1` | Firmware sample rate for the switch inputs. `0.4` emits the legacy bare edge string. |
| `outputs_enabled` | `true` | Master enable; off ⇒ both solenoids held de-energised (the only gate above local control). |
| `move_timeout_s` | `30` | Max continuous energise time per move before faulting. `0` disables. |
| `stall_window_s` | `4` | Window the gate must show progress within while moving. `0` disables. |
| `stall_min_progress_mm` | `3` | Minimum progress expected per stall window. |

## Tags published (live, for the HMI and peers)

`TargetHeight`, `GateHeight` (calibrated), `HeightOffset`, `Error`, `Moving`
(`raising`/`lowering`/`idle`), `RaiseOutput`, `LowerOutput`, `PumpOutput`,
`TopLimitActive`, `ManualRaise`, `ManualLower`, `Mode`, `Status`, `Fault`,
`FaultReason`, `HeightValid`.

## Regenerating `doover_config.json`

```bash
uv run export-config   # config schema
uv run export-ui       # UI schema
```
