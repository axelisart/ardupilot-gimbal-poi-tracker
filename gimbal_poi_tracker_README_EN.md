# gimbal_poi_tracker.lua

Lua script for ArduPlane 4.7 — Automatic POI locking and tracking with the CADDX gimbal.

**Author:** Alexis Artus (2026)

---

## Hardware Requirements

| Component | ArduPlane Parameter |
|---|---|
| CADDX gimbal on a serial port (e.g. SERIAL4) | `SERIAL4_PROTOCOL = 8`, `SERIAL4_BAUD = 115200` |
| Mount type | `MNT1_TYPE = 13` (CADDX) |
| RC7 — **3-position switch** | Tracking activation + gimbal lock control |
| RC8 — 3-position switch | Gimbal mode (neutral / normal) |

---

## User Parameters (top of script)

| Parameter | Default | Description |
|---|---|---|
| `RC_CHANNEL` | 7 | Tracking activation RC channel (3-position) |
| `RC_HIGH_THRESHOLD` | 1700 µs | Above this → tracking ON + ROI locked on target |
| `RC_LOW_THRESHOLD` | 1300 µs | Below this → tracking OFF (MID zone: 1300–1700 µs) |
| `RC_GIMBAL_MODE` | 8 | Gimbal mode control RC channel |
| `RC_GIMBAL_LOW` | 1300 µs | Below this: gimbal neutral (looks forward) |
| `UPDATE_HZ` | 10 | Loop update rate (Hz) |
| `MAX_TARGET_DIST_M` | 3000 m | Maximum target distance. If the computed range exceeds this value, the target is **clamped** to this distance along the same bearing (no refusal). |
| `MIN_PITCH_DEG` | −5.0° | Minimum gimbal depression angle. Activation is **refused** if the gimbal is shallower than this (e.g. −3° is shallower than −5°). Below −5° the sight-ray is too flat: a ±0.5° angle error causes ~26 m position error at 100 m AGL. |
| `GIMBAL_OFFSET_X` | 0.20 m | Gimbal position forward of CG (body frame) |
| `GIMBAL_OFFSET_Y` | 0.00 m | Gimbal position right of CG (body frame) |
| `GROUND_TEST_MODE` | false | Set to `true` to simulate altitude on the bench |
| `GROUND_TEST_ALT_M` | 100 m | Simulated AGL height when `GROUND_TEST_MODE = true` |
| `POI_ORBIT_RAD_M` | from `WP_LOITER_RAD` | Arrival radius: GCS notification sent once when plane enters this circle |
| `POI_PROGRESS_STEP_M` | 50 m | Progress notification interval: GCS distance update every N meters closer |

---

## RC7 — Three-Position Switch Logic

RC7 is the master control for the entire script.

| RC7 Position | Behavior |
|---|---|
| **LOW** (< 1300 µs) | Tracking OFF — returns to previous flight mode, gimbal free |
| **MID** (1300–1700 µs) | Tracking ON — GUIDED flight, gimbal free (RC_TARGETING, sticks work) |
| **HIGH** (> 1700 µs) | Tracking ON — GUIDED flight + ROI locked on target, no manual gimbal override |

**Both MID and HIGH activate tracking** on a rising edge from LOW. The target GPS is computed once from the current gimbal angles and stored. Subsequent MID↔HIGH transitions only toggle the gimbal lock without recomputing the target.

---

## RC8 — Gimbal Mode Override

RC8 takes priority over the ROI lock.

| RC8 Position | Behavior |
|---|---|
| LOW (< 1300 µs) | Gimbal in NEUTRAL mode — looks straight forward regardless of RC7 |
| MIDDLE or HIGH | Normal behavior — ROI active if RC7 HIGH |

---

## General Operation

```
┌─────────────────────────────────────────────────────┐
│  10 Hz loop                                         │
│                                                     │
│  RC8 < 1300 µs ?  →  gimbal NEUTRAL (mode 1)       │
│                                                     │
│  RC7 LOW → MID  : activate tracking, gimbal free    │
│  RC7 LOW → HIGH : activate tracking, ROI locked     │
│  RC7 LOW        : deactivate tracking               │
│                                                     │
│  If tracking active:                                │
│    RC7 MID → HIGH : ROI locked                      │
│    RC7 HIGH → MID : gimbal free (RC_TARGETING)      │
│    → Resend GUIDED waypoint every cycle             │
│    → Distance progress notifications                │
└─────────────────────────────────────────────────────┘
```

---

## Tracking Activation (RC7 MID or HIGH from LOW)

At activation, the script performs the following operations **once only**:

### 1. Reading Gimbal Angles

```lua
mount:get_attitude_euler(0)  →  (roll_bf, pitch_bf, yaw_bf)
```

The CADDX driver does not report measured gimbal attitude. `get_attitude_euler()` returns the **target angles sent to the gimbal**, which ArduPilot already expresses in **earth-frame** (pitch is relative to the horizon, not the aircraft body). Therefore `pitch_bf` is already a world-frame depression angle — no aircraft attitude correction is applied.

Yaw is reported in body-frame and is converted to an absolute geographic heading:

```
yaw_geo = (aircraft_heading + yaw_body_frame) % 360
```

### 2. Shallow Angle Guard

Activation is refused if the gimbal elevation is above `MIN_PITCH_DEG`:

```
if pitch_world > MIN_PITCH_DEG  →  aborted, GCS message with measured angle
```

At shallow angles the position error grows rapidly:

| AGL | Depression | Computed range | ±0.5° noise → position error |
|---|---|---|---|
| 100 m | −5° | ~1145 m | ±20 m |
| 100 m | −10° | ~567 m | ±5 m |
| 100 m | −20° | ~275 m | ±1.5 m |
| 100 m | −45° | 100 m | ±0.5 m |

### 3. Gimbal Lever-Arm Correction

The AHRS position is the aircraft CG. The gimbal is offset by `(GIMBAL_OFFSET_X, GIMBAL_OFFSET_Y)` in the body frame. This offset is projected horizontally onto the ground plane using the aircraft heading:

```
lever_north = GIMBAL_OFFSET_X · cos(heading) − GIMBAL_OFFSET_Y · sin(heading)
lever_east  = GIMBAL_OFFSET_X · sin(heading) + GIMBAL_OFFSET_Y · cos(heading)
```

The sight-ray origin is shifted by this amount before computing the target GPS.

> **Why no vertical correction?**  
> At a 0.20 m forward offset and 15° aircraft pitch, the vertical error on the
> target is < 10 cm — well within GPS noise (±2–5 m) and gimbal angle noise
> (~26 m at −20°, 100 m AGL). A full rotation matrix was used previously;
> it was removed as it added complexity with no measurable benefit.

### 4. Computing Target GPS Position

#### Side View — Horizontal Distance

```
                 aircraft
                    *
                   /|
                  / |
    gimbal       /  |  AGL height
    sight ray   /   |  = effective_alt_MSL − home_alt_MSL
               /    |
              / θ   |
             /______|___________
           target          ground (home alt)

  θ  = |pitch_world|   (depression angle, always > 0)

  dist_horiz = AGL_height / tan(θ)
```

If `dist_horiz > MAX_TARGET_DIST_M`, it is **clamped** to `MAX_TARGET_DIST_M` and a GCS warning is sent. The activation still proceeds.

#### Top View — North/East Decomposition

```
              North
               ↑
               |
   aircraft    *─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─→ East
               |╲
               |  ╲  yaw_geo = 135° (Southeast)
               |    ╲  dist_horiz = 227 m
   delta_N     |      ╲
   (negative   |        ╲
   = southward)↓          ╲
               P ─ ─ ─ ─ ─ * target
                  delta_E
                  (positive = eastward)

  delta_N = 227 × cos(135°) = −161 m
  delta_E = 227 × sin(135°) = +161 m
```

#### Converting Meters to GPS Degrees

```
  target_lat = gimbal_lat + delta_N / 111320
  target_lon = gimbal_lon + delta_E / (111320 × cos(gimbal_lat))
```

**Target altitude is fixed at home altitude.** This ensures ArduPlane computes the same depression angle as the gimbal for the ROI — no camera jump at activation.

### 5. Creating the Two Location Objects

| Object | Altitude | Purpose |
|---|---|---|
| `roi_loc` | `home_alt` MSL | Gimbal ROI — correct depression angle |
| `target_loc` | effective aircraft MSL altitude | GUIDED waypoint — aircraft holds level |

### 6. Activation Sequence

```
if RC7 HIGH:
  mount:set_roi_target()        → gimbal locks on target immediately
if RC7 MID:
  mount:set_mode(RC_TARGETING)  → gimbal free, responds to sticks
vehicle:set_mode(15)            → switch to GUIDED
vehicle:set_target_location()   → send waypoint
```

### 7. GCS Messages at Activation

```
POI: GROUND TEST MODE — simulated alt=100m AGL   (only if GROUND_TEST_MODE = true)
POI dbg1: plane=XXXm home=XXXm agl=XXXm [TEST]
POI dbg2: pitch_bf=XX.X plane_p=XX.X plane_r=XX.X pitch_w=XX.X yaw_bf=XX.X dist=XXXm
POI dbg3: lev_n=X.XXXm lev_e=X.XXXm
POI dbg4: target=XX.XXXXXXX,XX.XXXXXXX alt_home=XXXm
POI locked dist:XXXm pos:Xh              ← if RC7 HIGH
POI guided (gimbal free) dist:XXXm pos:Xh  ← if RC7 MID

POI: gimbal too shallow (−3.2° > −5.0°), aborted   ← if angle too shallow
POI: shallow angle, target clamped to 3000m          ← if dist clamped
```

The **clock position** indicates the direction of the target relative to the aircraft heading (`12h` = ahead, `6h` = behind, `3h` = right, `9h` = left).

---

## Active Loop (tracking in progress)

Each cycle (100 ms):

### Gimbal Lock Management

```
RC7 MID → HIGH:
  roi_locked = true
  mount:set_roi_target()       → ROI engaged
  GCS: "POI: ROI locked"

RC7 HIGH → MID:
  roi_locked = false
  mount:set_mode(RC_TARGETING) → gimbal responds to sticks
  GCS: "POI: gimbal free (RC7 mid)"

If roi_locked and RC8 not LOW:
  mount:set_roi_target()       → ROI refreshed every cycle
```

### Distance Progress Notifications

Each cycle, while the plane has not yet reached the orbit radius:

```
distance computed → floor to nearest 50 m step

If step decreased since last notification:
  GCS: "POI: XXXm to target"    (e.g. 600m, 550m, 500m, …)

If distance ≤ POI_ORBIT_RAD_M:
  GCS: "POI: orbiting target (dist=XXXm)"   ← sent once only
```

### GUIDED Orbit Maintenance

```lua
vehicle:set_target_location(last_target_loc)
```

The waypoint is resent every cycle. ArduPlane orbits the target point in GUIDED mode stably and precisely.

---

## Deactivation (RC7 LOW)

```
active = false
mount → RC_TARGETING (gimbal responds to sticks)
vehicle → previous mode (saved at activation)
GCS: "POI off, mode X"
```

---

## Technical Notes

### Why the CADDX Pitch Needs No Attitude Correction

The ArduPilot CADDX driver (`AP_Mount_CADDX`) reports **target angles sent to the gimbal**, not measured angles. These are already expressed in earth-frame (pitch relative to the horizon). Applying an aircraft pitch/roll correction on top would double-correct the angle and cause overshoot in flight.

### Why Use Home Altitude as Reference?

Using home altitude ensures that `calc_target_gps()` and ArduPlane use the **same reference** to compute the gimbal's ROI depression angle. If a different reference is used (e.g. terrain), ArduPlane corrects the gimbal angle at activation → visible camera jump.

### Why GUIDED and Not LOITER?

LOITER mode centers the orbit on the **aircraft's current position** at the time it is activated, not on the transmitted waypoint. GUIDED with `set_target_location()` resent continuously keeps the aircraft precisely on the computed target.

---

## State Diagram

```
                    ┌──────────┐
         RC7 LOW    │ INACTIVE │
    ┌───────────────│          │◄──────────────────────┐
    │               └──────────┘                       │
    │          RC7 LOW→MID │ RC7 LOW→HIGH              │ RC7 LOW
    │         gimbal free  │ ROI locked                │
    │                      ▼                           │
    │               ┌───────────────────────────┐      │
    │               │  ACTIVE (GUIDED)          │      │
    │               │                           │      │
    │               │  RC7 HIGH → roi_locked    │      │
    │               │  RC7 MID  → gimbal free   │      │
    │               │                           │      │
    │               │  RC8 LOW overrides all    │──────┘
    │               └───────────────────────────┘
    │
    └── on RC7 LOW: restore prev_mode, gimbal RC_TARGETING
```
