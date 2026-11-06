# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Drone-competition rectangle-target tracking system (电赛E题). A **MaixCAM2** board (MaixPy v4,
640×480 touch LCD) performs computer vision **and drives the gimbal directly** — no intermediate
microcontroller. The default actuator (V16) is two **FashionStar FSUS bus servos** on one
half-duplex UART2 bus; the legacy path is two Emm42_V5.0 closed-loop stepper drivers with 42
steppers (`servo.enabled=false`).

- **V16 — dual FSUS bus servos (optional; `config.json` default ON)** — `servo.enabled=true`
  switches the actuator to two FashionStar FSUS servos on one half-duplex **UART2 B0/B1**
  bus (TX 1kΩ series, IDs yaw=0 / pitch=1, 115200). One `cmd25` sync multi-turn frame per
  vision frame; outer loop = error → optional KF → **incremental PID** (`yaw_*/pitch_*`
  `kp/ki/kd/kff`) → absolute target angle, with deadband, `max_step_deg`, soft limits and
  anti-windup. Boot reads the current angle and holds it (no jump); lost → hold →
  bounded search. The stepper path is untouched and selected by `servo.enabled=false`;
  the V15 IMU inner loop is force-disabled in servo mode (650 Hz RPM would saturate the
  115200 bus). Touch `P` page is **not** mapped for servo yet — tune `[servo]` in
  `config.json`. New section `[servo]`; wiring in `接线图.md` §0 /
  `云台/舵机接线图_MaixCAM2.png`; analysis in `云台/双舵机云台改造方案.md`.

- **yaw (left/right)** — UART3, serial **speed mode**: `|dx|` linearly mapped to RPM.
- **pitch (up/down)** — UART2, serial **speed mode**: identical logic to yaw (`|dy|` → RPM).
  (A position-mode path exists in `EmmStepper` — `move`/`clear_position`/`home` — but is NOT
  wired into the control flow: driver ② failed with it on hardware. Both axes are now the
  same speed-mode logic.)
- **V7 features** — joint rect+ring detection (tracking aims the rectangle center; the
  earlier red-dot/bullseye aim was removed as it conflicted with rect detection — ring
  detection feeds circle mode only), ring-circling toggled by the **touchscreen `T/C`
  button** (default; optional fallback: GPIO49/Pin3 jumper when `mode_pin.enabled=true`),
  and boot search (yaw spins single-direction until first acquisition). All params live in
  `k230/config.json` (`[red_target]` / `[search]` / `[circle]` / `[mode_pin]`).
- **Touchscreen center calibration (V7.7) + tuning page (V7.9)** — on-screen green dot + 8
  touch buttons on the calib page (▲▼◀▶ move the aim offset live, T/C toggles track/circle mode,
  ● hides the UI, S saves `center.laser_offset_x/y` back to config.json, P opens the PID/param
  tuning page); a same-button click guard (`center_calib.debounce_ms`, default 300ms) swallows
  touch-jitter repeat edges (different buttons are never blocked); touch failure auto-disables
  without affecting tracking. Params in `[center_calib]`.
- **PID speed control + touch tuning (V7.9)** — optional speed-form PID per axis
  (`pid_enabled`; `u = Kp·e + Ki·∫e·dt + Kd·de/dt` clamped to `[min_rpm, max_rpm]`, anti-windup
  freeze on saturation, PID state cleared on deadband/lost/search frames). Disabled = the
  linear map exactly as before. All PID + per-axis motion params (`kp/ki/kd/max/min/dead/acc/full`)
  are live-tunable on the touch **param page** (`P`: `Y/P` axis, item cycling, `+/-` hold-repeat,
  `S` writes the 18 keys back to config.json text-level with `.bak` + read-back verify). Changes
  apply on the next frame; coast/search keep the original map.
- **V8 — Kalman filter (default off)** — per-axis 2-state (position+velocity) KF smooths the
  vision error, ported from the reference gimbal's `VisionKalmanFilterInit`; `kf_lead_s>0` uses
  the (previously unused) velocity state as a look-ahead feed-forward against frame latency.
  Keys `kf_*` in `[motor]`/`[pitch_motor]`; KF state cleared together with PID state on
  deadband/lost/search frames.
- **V8 — circle memory fallback (default off)** — `circle.fallback_ms>0` keeps drawing for a
  short window after the ring is lost, using the remembered radius + box center
  (`ring_memory_step`, prints `CIRCLE fallback`); then falls back to the original behavior.
- **V8.5 — circle geometry main-track** — `circle.radius_ratio>0` makes
  circle-drawing consume NO ring detection: ring center := box center (confirmed concentric
  prior), radius := k×√(box area) — immune to the laser-sweeping-the-ring self-interference.
  `red_target.enabled=false` skips ring detection entirely. Prints/OSD tag `[geo]` (geometry)
  vs `[ring]` (detection).
- **V8.6 — progressive auto-calibration + circle OSD + speed pack** — `circle.calib_frames: 30`
  (**default on**) feeds valid ring frames to `calib_feed`; **≥6 samples make k live at once**
  (`CIRCLE autocalib ready: k=...`), 30 samples converge (`... 收敛`; paste into `radius_ratio`
  to make permanent). Circle mode draws the circle: orange = synthetic (k path), cyan =
  memory fallback (blue = detected ring, as before); `CIRCLE: no ring/k` hint when circle mode
  degenerates to rect. Speed pack: `acc` 200→250 (ramp (256-acc)×50μs/RPM: 2.8→0.3 ms/RPM),
  yaw `max_rpm` 60→90 / pitch 30→45, `dx/dy_full_px` 200→150. Touch param page shows `C:` state.
- **V8 — Emm42 read commands & stall monitor (default off)** — `read_velocity`(0x35) /
  `read_position`(0x36) / `reset_clog`(0x0E 0x52); boot probe appends `rpm=N` to the `actuator:`
  line; `monitor_ms>0` prints a stall WARN (warn only, control unaffected); `clog_reset_on_boot`
  optional auto-recovery. **Response byte layouts are expected-only — verify on hardware via
  `raw_probe(0x35)/raw_probe(0x36)` before trusting the parsed values.**
- **V15 — attitude stabilisation (onboard IMU, default OFF)** — `[attitude]` section. The
  onboard LSM6DSOW (IIC1/0x6B, acc+gyro, **no magnetometer**) is rigidly on the same board as
  the camera, so the gyro reads `ω_cam = ω_base + m_motor` and **base disturbance cannot be
  separated from the gimbal's own tracking motion** — therefore NO feed-forward "measure the
  disturbance and cancel it". Instead a two-loop structure: the vision outer loop only moves the
  bearing setpoint `θ_ref` (`att_outer`, px→deg via `px_per_deg`), and an inner loop
  (`att_step`, 400–650 Hz between vision frames) holds the camera's **absolute** attitude on
  `θ_ref` using a disturbance observer (`ω_base_hat = ω_cam - 6·rpm_prev`) + angle P.
  `θ_imu` integrates the de-tilted gyro (`att_yaw_rate`); tilt comes from `att_tilt`
  (acc+gyro complementary filter). Zero point = power-on attitude; the target bearing is
  `θ_imu + dx/px_per_deg` at first lock (no magnetometer ⇒ no absolute heading, which is fine
  because only the 22 ms inter-frame window is graded). `px_per_deg = 0` auto-derives the focal
  length from the existing `[distance]`/`[circle]` calibration (≈11.5 px/deg at the current
  numbers ⇒ 0.5° ≈ 5.8 px). Lost target: `lock → hold` (IMU holds the world bearing for
  `hold_ms`) `→ search` (**single-direction, counted in turns via IMU 360°**, `search.turns`
  0 = keep looking). RPM is integer ⇒ 1 RPM = 6°/s granularity, so near-zero corrections are
  dithered under `deadband_dps` + `min_pulse_ms`. Divergence guard: `|err| > fault_deg` for
  300 ms ⇒ auto-disable and fall back to the legacy mapping. Attitude mode uses its own
  `[attitude].acc` (255, sub-tick command build-up) and `min_rpm` 1; `[motor].acc/min_rpm` only
  apply to the legacy path. `enabled=false` ⇒ byte-for-byte V14 behaviour. Implementation detail:
  the three yaw states are bound to either legacy or attitude implementations **at boot**
  (`_att_bind`) so the hot path gets no new `if`.

(The original MaixCam + STM32F407 relay architecture is retired — there is no `stm32/` directory and no `fs_servo.py`; all code is single-file in `k230/main.py`.)

## Architecture

```
MaixCAM2 (MaixPy v4, vision + gimbal control)
    camera 640×480               UART2 B0/B1 → FSUS servo bus → yaw ID0 / pitch ID1   [V16 default]
    LCD 640×480 touch            (legacy) UART2/UART3 → Emm42 drivers → 42 steppers
    mode switch (optional A4)    touchscreen T/C (default) / jumper: 3.3V = circle
```

Visual→actuator interface: `gimbal_send(dx, dy, dist, status)` inside `k230/main.py` (section 四).

> V16 is the default: one UART2 half-duplex bus + two FSUS servos (IDs 0/1). The stepper path is
> legacy/rollback (`servo.enabled=false`). See `云台/双舵机云台改造方案.md`,
> `接线图.md` §0 and `云台/舵机接线图_MaixCAM2.png`.

Camera capture: MaixCAM2 onboard camera — `camera.Camera(SENSOR_W, SENSOR_H,
image.Format.FMT_BGR888, fps=FPS)` with config **640×480**, `cv2` zero-copy BGR frame;
display `disp.show(img)` 1:1 on the 640×480 LCD. The old CanMV `Sensor(id=CSI_ID)` /
1280×720 probe / `crop=True` / `scale_view` chain is retired; software upscaling was
disproven on hardware (garble/hang) and is guarded by `test_camera_init.py`.

Device access (bench): MaixVision over USB-RNDIS or the same WiFi; SSH `ssh root@<IP>` with
password **`root`** (factory default; reset on the bench 2026-11-05). The USB
net address is `10.150.102.1` (stable, preferred); the WiFi address was `10.55.95.113`
(changes with the network) — confirm with `hostname -I` or the MaixVision connect page.
Run scripts remotely with `scp ... root@<IP>:/root/` +
`ssh root@<IP> "python3 /root/x.py"`, never while MaixVision is running another program
(both grab the camera and UART2). The password lives only as a hash in `/etc/shadow`; see
README §1.4.

## Repository structure

```
复刻/
├── k230/
│   ├── main.py          # single-file runtime: config load → servo/stepper driver
│   │                    #   → actuator init + gimbal_send → CV/YOLO detection → main loops
│   └── config.json      # the sole tuning surface ([servo] default; [motor]/[pitch_motor] legacy)
├── 接线图.md             # authoritative wiring + V16 MaixCAM2 servo diagram
├── skill/               # methodology docs (reference only, not runtime)
└── tools/               # host-side regression tests only (not deployed)
    ├── test_emm_protocol.py    # host-side frame test (AST-extracts EmmStepper from main.py)
    ├── test_coast_logic.py     # coast state-machine test (AST-extracts coast_step)
    ├── test_circle_logic.py    # V7 pure-function tests (priority/circle/search/pin)
    ├── test_camera_init.py     # capture guard: id + 720 probe + 16:9 output + scale view (geometry regression)
    └── test_center_calib.py    # touch-calibration: hit test / touch state machine / config save
```

## Critical performance constraints (MicroPython on MaixCAM2)

MicroPython on MaixCAM2 has no JIT. These anti-patterns cause 5-10x slowdowns on the hot path:

1. **Class attribute access** (`self.xxx`) — costs 5-10x more than reading a module-level global.
2. **Hot-path `if` branches** — each branch check per frame adds up.
3. **Dict allocation** — triggers GC, causing periodic frame drops.

The codebase uses a **lambda injection pattern** to eliminate hot-path branching. Optional features (Kalman filter, UART) are resolved at import time:

```python
# At module load — one-time decision:
if KALMAN_ENABLED:
    _kf = _Kalman(KALMAN_ALPHA)
    kf_update = _kf.update       # real EMA
    kf_reset  = _kf.reset
else:
    kf_update = lambda mx, my: (float(mx), float(my))  # pass-through
    kf_reset  = lambda: None                            # no-op

# In the hot loop — no branch:
fdx, fdy = kf_update(dx, dy)  # works identically whether Kalman is on or off
```

**Never add an `if FEATURE_ENABLED:` inside the main `while True` loop.**

## Config system

- `k230/config.json` is the **only tuning surface**. Sections: detection, camera, display, center, center_calib, rectangle, yolo, motor (yaw), pitch_motor (pitch), red_target, search, circle, mode_pin, debug.
  `[motor]`/`[pitch_motor]` also carry the V7.9 PID keys (`pid_enabled` / `pid_kp` / `pid_ki` /
  `pid_kd`); these plus the motion params (`min_rpm` / `max_rpm` / `dx_full_px`|`dy_full_px` /
  `deadband_px` / `acc`) are live-tunable from the touch param page (`P`) and written back by its
  `S` button (text-level replace, all-or-nothing, `.bak` + read-back verify).
  V8 additions: `kf_*` (Kalman, default off) and `monitor_ms` / `clog_reset_on_boot`
  (Emm42 read-based stall monitor, default off) in `[motor]`/`[pitch_motor]`;
  `[circle].fallback_ms` (ring-memory fallback, default 0 = off);
  V8.5/V8.6: `[circle].radius_ratio` (geometry main-track k; 0 = runtime auto-calibrated) /
  `[circle].calib_frames` (progressive calibration, **default 30 = on**: ≥6 valid ring frames
  make k live)
  and `[red_target].enabled` (default true = ring detection on).
  **V15 additions**: the `[attitude]` section (18 keys — `enabled` / `yaw_sign` / `tick_period_us` /
  `tick_budget_ms` / `gyro_dps` / `angle_kp` / `rate_ki` / `deadband_dps` / `min_pulse_ms` /
  `min_rpm` / `px_per_deg` / `outer_gain` / `outer_lead_s` / `hold_ms` / `fault_deg` / `active_dps` /
  `tilt_alpha` / `acc`; `enabled` and `angle_kp` + `deadband_dps` are live-tunable on the touch
  param page as `att` / `AKp` / `ADz`, yaw axis only — `param_next(item, axis)` skips them on
  pitch) and `[search].turns` / `[search].dps`. The touch `S` button now writes **21** keys.
  **V16 additions**: `[servo]` section (`enabled` / bus pins / `yaw_id`/`pitch_id` /
  `yaw_sign`/`pitch_sign` / soft limits / `yaw_*`+`pitch_*` PID / `max_dps` / `max_step_deg` /
  `deadband_px` / `i_zone_px` / `interval_ms` / `power_mw` / `sync` / `hold_ms` / `search_*` /
  `kf_*` / `test_mode` / `zero_auto` / `yaw_zero_deg` / `pitch_zero_deg`). `servo.enabled=true`
  skips the stepper init and rebinds
  `gimbal_send` to `servo_send`; the touch `P` page is not mapped for servo yet — edit
  `[servo]` and restart.
  **Software zero**: internal targets/limits/search live in a "logical" frame where the
  software zero is 0°; `_servo_send_targets` / `_servo_boot_send` add `_s_off_y` / `_s_off_p`
  (raw = logical + offset) and `_servo_init` sets the offsets from `zero_auto` (capture the
  boot pose) or the fixed `yaw_zero_deg`/`pitch_zero_deg`. Nothing is written to the servo
  EEPROM, so a power cycle re-captures the zero; `cmd17` is not used by `main.py`.
- The config loader strips keys starting with `_`, so comments can be embedded as `"_说明": "..."`.
- Values are read at import time into module-level globals; restart `main.py` after edits.

## Emm42 protocol essentials (k230/main.py, section 三)

- Frame: `[addr][func][data...][checksum]`; checksum is the fixed value **0x6B**; all
  multi-byte fields are **big-endian** (opposite of Modbus).
- Used commands:
  - enable `F3 AB <en> <sync>`, stop `FE 98 <sync>`
  - speed mode `F6 <dir> <spd×2> <acc> <sync>` (yaw)
  - position mode `FD <dir> <spd×2> <acc> <pulses×4> <abs> <sync>` (reserved — not wired into control flow; abs=01 absolute, negative targets two's-complement)
  - position clear `0A 6D` (defines current physical position as 0° — pitch boot)
  - home `9A <mode> <sync>` (reserved, not wired into the control flow)
  - read bus voltage `24` (online probe)
- 16 microsteps → 3200 pulses/rev (only relevant to the reserved position-mode path).
- Driver-side presets (checksum 0x6B, 115200, UART_FUN, Response=Receive, ID) — see `接线图.md`.

## Testing

- **Host-side (no hardware)**: nine stdlib-only scripts, all static-check `k230/main.py`
  (some also read `k230/config.json`):
  - `python tools/test_emm_protocol.py` — frame bytes vs manual examples + V8 read-command /
    clog-reset frames & reply parsing (24 cases). Always run after touching protocol code.
  - `python tools/test_coast_logic.py` — coast (lost-target pre-judgment) state machine (10 cases).
  - `python tools/test_circle_logic.py` — V10/V14 geometry (41 cases): circle point/θ
    gating + wait timeout, quad area (shoelace), search & pin-debounce machines,
    `geo_radius_px` (25.6×17 / Ø11.3), distance-by-area + laser mapping. The old
    V8.5 ring-detection tests (`classify_blob` / `ring_memory_step` / `calib_feed`)
    were retired with the V10 red-ring removal.
  - `python tools/test_camera_init.py` — MaixCAM2 capture guard (11 cases):
    exactly one `camera.Camera(SENSOR_W, SENSOR_H, FMT_BGR888, fps=FPS)`, config
    640×480 + center half, display 1:1, exactly one `disp.show`, no legacy
    `Sensor/set_framesize/CSI_ID`, dead-display-path guard.
  - `python tools/test_center_calib.py` — touch calibration (37 cases): 8-button layout (incl. P),
    button hit test, touch state machine (wake edge / repeat / no-bounce on hide / same-button
    debounce guard), config text save (exactly 2 lines).
  - `python tools/test_pid_tune.py` — V7.9 PID + touch tuning (59 cases): pid_step (P/I/D,
    clamp, anti-windup, first-frame/long-frame dt), pid_speed, param step/clamp/toggle,
    section-scoped config text save (motor vs pitch_motor isolation, real-config roundtrip),
    tuning state machine (plus/minus repeat + same-button debounce), static guards (tune vars
    defined; gimbal_send wired to pid_step).
  - `python tools/test_kf_filter.py` — V8 Kalman filter (18 cases): kf_step convergence / noise
    suppression / step response / dt guards / lead feed-forward / clamping / reset, plus static
    guards (KF vars defined; gimbal_send wired to kf_step; kf_enabled defaults false).
  - `python tools/test_attitude_loop.py` — **V15 attitude (45 cases)**: px↔deg focal derivation,
    tilt complementary filter, de-tilt yaw projection, `att_step` (deadband/min/max/fault/sign
    symmetry), `att_gate` min-pulse, `att_outer` gain/deadband/lead clamp, `att_turn_step`
    turn counting — **plus a closed-loop simulation** (base rate + motor rate = camera rate,
    with 1-RPM quantization, transport delay, gyro scale error and noise) asserting the
    measured pixel/angle error stays inside the 0.5° acceptance line and that the wrong
    `yaw_sign` diverges (which is what the fault guard exists for). Static guards: attitude
    vars defined, inner loop wired, yaw dispatch has no inline `if ATT_EN`, `attitude.enabled`
    defaults false, `search.turns` defaults 0.
  - `python tools/test_servo_protocol.py` — **V16 servo (40 cases)**: FSUS frame bytes vs
    protocol samples (ping / cmd8 / cmd13 / cmd25 sync / cmd18 stop), signed i32 + 0.1°
    encoding, response parse, `servo_pid` / `servo_step_target` / `servo_search_step` /
    `servo_boost`
    (deadband / clamp / anti-windup / integral separation / no-jump search / 1-4 screen boost),
    a stub
    closed-loop simulation of `servo_send` (target updates + sync frame contents + hold/search),
    software-zero offset (frame angle = logical target + offset, boot frame offset), plus static
    guards (`[servo]` config incl. zero keys, stepper disabled, `servo_send` wired to PID/KF).
- All nine suites currently pass: **285 cases** total (servo 40, emm 24, coast 10, circle 41,
  camera 11, center_calib 37, pid_tune 59, kf 18, attitude 45).
- **Hardware**: `motor.test_mode` / `pitch_motor.test_mode` in config.json perform power-on
  self-checks (online probe + continuous rotation) without running vision. `main.py` also
  self-reports at boot: the `actuator:` line (both axes + online voltage). V16 adds
  `servo.test_mode` (ping / read angle / ±5° jog / cmd25 sync).

## Detection modes

- **CV** (default): Classic OpenCV pipeline — white mask → contours → black-border/white-center
  verification → scoring. ~45fps, no model needed. Only supports white rectangle targets on dark backgrounds.
- **YOLO**: KPU inference via PipeLine/AIBase/Ai2d framework. Requires `model.kmodel` on SD card. ~25fps.
  `input_size` must equal training `imgsz`; `task` (detect/segment) must match the model.

## OpenCV speed optimization techniques used

| Layer | Strategy |
|-------|----------|
| Input | Detection downsampling (320×240, 1/4 pixels), frame skipping (`detect_every`) |
| Preprocess | `cv2.inRange` color mask (binary image directly, faster than grayscale+Canny) |
| Contour | `RETR_EXTERNAL` (outer contours only), loose `approxPolyDP` (epsilon=0.04) |
| Filter | Area + aspect-ratio boundingRect (O(1)), before expensive contour analysis |
| Temporal | Spatial continuity filter (center jump / area ratio), lost-frame hold |
| Verify | ROI-only black-border and white-center checks (not full-image) |
| Runtime | GC every N frames, print every N frames (both configurable) |

## UART pin map (MaixCAM2)

| Path | UART | TX | RX | Device |
|---|---|---|---|---|
| V16 servo yaw | UART2 | B0 | B1 | FSUS servo ID0 (one half-duplex bus) |
| V16 servo pitch | UART2 | B0 | B1 | FSUS servo ID1 (same bus) |
| legacy yaw stepper | UART2 | B0 | B1 | Emm42 driver ① |
| legacy pitch stepper | UART3 | B2 | B3 | Emm42 driver ② |

V16 uses **one bus** for both servos (IDs distinguish them); the legacy stepper path uses two
UARTs. Never connect/enable both paths at once. Full wiring in `接线图.md`.
