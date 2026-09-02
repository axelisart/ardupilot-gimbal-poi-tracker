-- =============================================================================
-- gimbal_poi_tracker.lua
-- ArduPlane 4.7 — POI tracker using CADDX gimbal
--
-- Author: Alexis Artus (2026)
--
-- Hardware requirements:
--   - CADDX gimbal connected on a serial port (e.g. SERIAL4)
--   - SERIALX_PROTOCOL = 8  (Gimbal)
--   - SERIALX_BAUD    = 115200
--   - MNTX_TYPE       = 13  (CADDX)
--   - CAMX_TYPE       = 4  (Mount)
--   - RC7 configured as a 3-position switch
--   - RC8 configured as a 3-position switch
--   - RC5_OPTION configured as Mount 1 Yaw in arduplane 
--   - RC6_OPTION configured as Mount 1 Pitch in arduplane
--   - RC8_OPTION configured as Mount Roll/Pitch Lock in arduplane (only way to change CADDX Gimbal mode)
--
-- How it works:
--   1. Script runs in a loop at ~10 Hz
--   2. RC8 low → gimbal neutral (looks forward), regardless of RC7
--   3. RC7 LOW → inactive, previous flight mode
--   4. RC7 MID (rising edge from LOW) → tracking activated, GUIDED + gimbal free
--      a. Target GPS computed ONCE from current gimbal angles
--      b. Plane switches to GUIDED toward target
--      c. Gimbal in RC_TARGETING: user controls it freely with sticks
--   5. RC7 HIGH (rising edge from LOW, or transition from MID) →
--      a. If coming from LOW: same activation as MID but ROI immediately locked
--      b. If already active (from MID): ROI locked on target, no manual override
--   6. RC7 HIGH → MID while active: ROI released, gimbal free again
--
-- NOTE:
--   The C20D gimbal does NOT transmit GPS coordinates directly.
--   Script computes them from: plane AGL altitude + gimbal angles (pitch/yaw) corrected with plane attitude and gimbal offset from CG.
--   Target altitude is fixed at home (takeoff) altitude — no terrain data needed.
-- =============================================================================

-- ─── User parameters ─────────────────────────────────────────────────────────

local RC_CHANNEL          = 7       -- Activation RC channel (RC7, 3-position)
local RC_HIGH_THRESHOLD   = 1700    -- Above this → tracking + ROI locked on target
local RC_LOW_THRESHOLD    = 1300    -- Below this → tracking off (MID zone: 1300–1700)

local RC_GIMBAL_MODE      = 8       -- Gimbal mode RC channel (RC8)
local RC_GIMBAL_LOW       = 1300    -- Below this → gimbal neutral (look forward)

local UPDATE_HZ           = 10      -- Loop update rate (Hz)
local UPDATE_MS           = 1000 / UPDATE_HZ

-- Maximum target distance (meters). When gimbal is near horizon and computed distance
-- exceeds this value, target is clamped to this range (no refusal).
local MAX_TARGET_DIST_M   = 3000

-- Minimum gimbal depression angle (degrees, negative = below horizon).
-- Below this absolute value the sight-ray is too shallow: small angle errors cause
-- large position errors. Activation is refused above this threshold.
-- Rule of thumb: at 100 m AGL, -5° → ~1150 m range (reasonable); -2° → ~2860 m (too noisy).
local MIN_PITCH_DEG       = -5.0

-- ─── Gimbal lever-arm offset (body frame, meters) ────────────────────────────
-- Horizontal distance from aircraft CG to gimbal, in body frame:
--   X: forward  (+) / backward (-)
--   Y: rightward (+) / leftward (-)
-- Impact analysis: at 100 m AGL with a 0.20 m offset, the position error on the
-- target is at most 0.20 m — negligible vs GPS noise (±2–5 m) and gimbal angle
-- noise (±0.5° at -20° → ~26 m error). Full rotation matrix corrections were
-- removed: they added complexity without measurable benefit.
-- Set both to 0 to disable.
local GIMBAL_OFFSET_X     =  0.20  -- meters forward of CG
local GIMBAL_OFFSET_Y     =  0.00  -- meters right of CG

-- ─── Ground test mode ────────────────────────────────────────────────────────
-- Set to true to simulate a virtual aircraft altitude for bench testing.
-- The real AHRS altitude is replaced by GROUND_TEST_ALT_M above home.
-- Set back to false before any real flight.
local GROUND_TEST_MODE    = false
local GROUND_TEST_ALT_M   = 100    -- simulated AGL height (meters)

-- Orbit arrival radius (meters). GCS message sent once when plane enters this circle.
-- Read from WP_LOITER_RAD (ArduPlane loiter radius parameter).
local POI_ORBIT_RAD_M     = math.abs(param:get("WP_LOITER_RAD"))

-- Progress notification interval (meters). GCS distance update sent every N meters closer.
local POI_PROGRESS_STEP_M = 50

-- ─── State variables ─────────────────────────────────────────────────────────

local active              = false   -- Tracking mode currently active
local roi_locked          = false   -- true = ROI auto-lock (RC7 HIGH); false = gimbal free (RC7 MID)
local orbit_notified      = false   -- true once GCS orbit message has been sent
local last_progress_step  = nil     -- last 50 m step notified (e.g. 500, 450, 400 …)
local prev_mode           = nil     -- Flight mode saved before activation
local last_target_loc     = nil     -- Target location fixed at activation (lat/lon/alt)
local last_roi_loc        = nil     -- Gimbal ROI location (home alt)

-- ─── Helper functions ────────────────────────────────────────────────────────

--[[
  Computes the GPS position of the ground target from:
    - plane_loc       : Location object (lat/lon in 1e7 degrees, alt in cm MSL)
    - mount0_yaw_geo  : gimbal heading in degrees (0 = North, clockwise)
    - mount0_pitch_geo : gimbal elevation in degrees (negative = below horizon)
    - home_alt_m      : home (takeoff) altitude in meters MSL

  Returns (lat, lon) in decimal degrees, or nil if the calculation is invalid.
--]]
local function calc_target_gps(plane_loc, mount0_yaw_geo, mount0_pitch_geo, home_alt_m)
    -- Reject if gimbal is too close to the horizon (angle noise → huge position error)
    if mount0_pitch_geo > MIN_PITCH_DEG then
        return nil, "shallow"
    end

    -- Height of plane above home altitude
    local plane_height_m = plane_loc:alt() * 0.01 - home_alt_m

    if plane_height_m <= 0 then
        return nil, "alt"
    end

    -- Horizontal distance to target using trigonometry
    -- Clamp pitch to -1° minimum so tan() never reaches zero (belt-and-suspenders guard).
    local safe_pitch      = math.min(mount0_pitch_geo, -1.0)
    local dist_horizontal = plane_height_m / math.tan(math.rad(-safe_pitch))

    -- Clamp to max range instead of refusing: target is placed at max distance along
    -- the same bearing when gimbal is very shallow.
    local clamped = false
    if dist_horizontal > MAX_TARGET_DIST_M then
        dist_horizontal = MAX_TARGET_DIST_M
        clamped = true
    end

    -- Decompose into North/East deltas
    local yaw_rad  = math.rad(mount0_yaw_geo)
    local delta_n  = dist_horizontal * math.cos(yaw_rad)
    local delta_e  = dist_horizontal * math.sin(yaw_rad)

    -- Convert meters to degrees (spherical earth approximation)
    local plane_lat = plane_loc:lat() * 1e-7
    local plane_lon = plane_loc:lng() * 1e-7

    local target_lat = plane_lat + delta_n / 111320.0
    local target_lon = plane_lon + delta_e / (111320.0 * math.cos(math.rad(plane_lat)))

    return target_lat, target_lon, clamped
end

-- ─── Main loop ───────────────────────────────────────────────────────────────

local function update()

    -- 1. Read RC8: gimbal mode override (low = neutral, high/mid = normal/ROI)
    local rc8_pwm = rc:get_pwm(RC_GIMBAL_MODE)
    local gimbal_neutral = rc8_pwm and rc8_pwm < RC_GIMBAL_LOW
    if gimbal_neutral then
        mount:set_mode(0, 1)   -- NEUTRAL: gimbal looks straight forward
    end

    -- 2. Read RC7
    local rc7_pwm = rc:get_pwm(RC_CHANNEL)
    if not rc7_pwm then
        return update, UPDATE_MS
    end

    -- ── Activation function (shared by MID and HIGH rising edges) ────────
    local function activate_tracking(with_roi_lock)

        -- Read plane position and gimbal angles
        local plane_loc    = ahrs:get_location()
        local mount0_roll_bf, mount0_pitch_bf, mount0_yaw_bf = mount:get_attitude_euler(0)

        if not plane_loc or not mount0_roll_bf or not mount0_pitch_bf or not mount0_yaw_bf then
            gcs:send_text(4, "POI: no AHRS/gimbal data, aborted")
            return false
        end

        -- Home altitude as ground reference (target altitude = home altitude)
        local home_alt_cm = ahrs:get_home():alt()   -- cm MSL (native ArduPilot unit)
        local home_alt_m  = home_alt_cm * 0.01

        -- Effective aircraft altitude: real or simulated (ground test mode)
        local effective_alt_cm
        if GROUND_TEST_MODE then
            effective_alt_cm = home_alt_cm + GROUND_TEST_ALT_M * 100
            gcs:send_text(4, string.format(
                "POI: GROUND TEST MODE — simulated alt=%.0fm AGL", GROUND_TEST_ALT_M
            ))
        else
            effective_alt_cm = plane_loc:alt()
        end

        -- Convert gimbal body-frame yaw to absolute geographic heading
        local plane_heading  = math.deg(ahrs:get_yaw_rad())
        local mount0_yaw_geo = (plane_heading + mount0_yaw_bf) % 360

        -- The CADDX driver does not report measured gimbal attitude.
        -- mount:get_attitude_euler() returns the *target* angles sent to the gimbal
        -- (see AP_Mount_CADDX::get_attitude_quaternion), which ArduPilot already
        -- expresses in earth-frame (pitch is relative to the horizon, not body-frame).
        -- Therefore mount0_pitch_bf is already a world-frame depression angle.
        -- No aircraft attitude correction is needed or appropriate.
        local plane_pitch_deg = math.deg(ahrs:get_pitch_rad())
        local plane_roll_deg  = math.deg(ahrs:get_roll_rad())
        local mount0_pitch_world = mount0_pitch_bf   -- already earth-frame

        -- Lever-arm correction: shift the ray origin from CG to gimbal position.
        -- The gimbal body-frame offset (X forward, Y right) is projected onto the
        -- ground plane using the current heading. Vertical correction is omitted:
        -- at 0.20 m offset and up to 15° pitch, lever_down < 5 cm → < 10 cm on target,
        -- which is well within GPS and angle noise margins.
        local yaw_rad     = math.rad(plane_heading)
        local lever_north = GIMBAL_OFFSET_X * math.cos(yaw_rad) - GIMBAL_OFFSET_Y * math.sin(yaw_rad)
        local lever_east  = GIMBAL_OFFSET_X * math.sin(yaw_rad) + GIMBAL_OFFSET_Y * math.cos(yaw_rad)

        -- Shift the ray origin horizontally by the lever arm
        local gimbal_lat_ref = plane_loc:lat() * 1e-7 + lever_north / 111320.0
        local gimbal_lon_ref = plane_loc:lng() * 1e-7
                             + lever_east / (111320.0 * math.cos(math.rad(plane_loc:lat() * 1e-7)))

        gcs:send_text(5, string.format(
            "POI dbg3: lev_n=%.3fm lev_e=%.3fm",
            lever_north, lever_east
        ))

        -- Compute target GPS from gimbal angles and effective AGL height above home
        -- (uses simulated altitude in ground test mode, lever-arm corrected position)
        local test_loc = Location()
        test_loc:lat(math.floor(gimbal_lat_ref * 1e7))
        test_loc:lng(math.floor(gimbal_lon_ref * 1e7))
        test_loc:alt(math.floor(effective_alt_cm))
        test_loc:relative_alt(false)
        test_loc:terrain_alt(false)
        local target_lat, target_lon, dist_clamped = calc_target_gps(test_loc, mount0_yaw_geo, mount0_pitch_world, home_alt_m)

        if not target_lat then
            gcs:send_text(4, string.format(
                "POI: gimbal too shallow (%.1f° > %.1f°), aborted",
                mount0_pitch_world, MIN_PITCH_DEG
            ))
            return false
        end
        if dist_clamped then
            gcs:send_text(4, string.format(
                "POI: shallow angle, target clamped to %.0fm",
                MAX_TARGET_DIST_M
            ))
        end

        -- Pre-compute integer lat/lon once for both Location objects
        local target_lat_1e7 = math.floor(target_lat * 1e7)
        local target_lon_1e7 = math.floor(target_lon * 1e7)

        -- ROI location: target lat/lon at home altitude
        -- ArduPlane will compute the same depression angle as the gimbal
        local roi_loc = Location()
        roi_loc:lat(target_lat_1e7)
        roi_loc:lng(target_lon_1e7)
        roi_loc:alt(home_alt_cm)   -- home alt in cm
        roi_loc:relative_alt(false)
        roi_loc:terrain_alt(false)

        -- GUIDED waypoint: target lat/lon at effective MSL altitude (plane holds level)
        local target_loc = Location()
        target_loc:lat(target_lat_1e7)
        target_loc:lng(target_lon_1e7)
        target_loc:alt(math.floor(effective_alt_cm))   -- effective MSL altitude in cm
        target_loc:relative_alt(false)
        target_loc:terrain_alt(false)

        active             = true
        roi_locked         = with_roi_lock
        orbit_notified     = false
        last_progress_step = nil
        last_target_loc    = target_loc
        last_roi_loc       = roi_loc
        prev_mode          = vehicle:get_mode()

        if with_roi_lock then
            mount:set_roi_target(0, last_roi_loc)
        else
            mount:set_mode(0, 3)   -- RC_TARGETING: gimbal free from the start
        end
        vehicle:set_mode(15)                      -- GUIDED toward target (once)
        vehicle:set_target_location(last_target_loc)

        -- Pre-compute plane lat/lon in degrees (reused for debug and clock position)
        local plane_lat        = plane_loc:lat() * 1e-7
        local plane_lon        = plane_loc:lng() * 1e-7
        local effective_alt_m  = effective_alt_cm * 0.01
        local effective_agl_m  = effective_alt_m - home_alt_m

        -- Debug messages
        local dn0     = (target_lat - plane_lat) * 111320.0
        local de0     = (target_lon - plane_lon) * (111320.0 * math.cos(math.rad(plane_lat)))
        local dbg_dist = math.sqrt(dn0*dn0 + de0*de0)
        local dist0   = math.floor(dbg_dist)
        gcs:send_text(5, string.format(
            "POI dbg1: plane=%.0fm home=%.0fm agl=%.0fm%s",
            effective_alt_m, home_alt_m, effective_agl_m,
            GROUND_TEST_MODE and " [TEST]" or ""
        ))
        gcs:send_text(5, string.format(
            "POI dbg2: pitch_bf=%.1f plane_p=%.1f plane_r=%.1f pitch_w=%.1f yaw_bf=%.1f dist=%.0fm",
            mount0_pitch_bf, plane_pitch_deg, plane_roll_deg, mount0_pitch_world, mount0_yaw_bf, dbg_dist
        ))
        gcs:send_text(5, string.format(
            "POI dbg4: target=%.7f,%.7f alt_home=%.0fm",
            target_lat, target_lon, home_alt_m
        ))

        -- Clock position relative to plane heading

        local rel = mount0_yaw_bf
        if rel >  180 then rel = rel - 360 end
        if rel < -180 then rel = rel + 360 end

        local clock_str
        if math.abs(rel) < 15 then
            clock_str = "12h (ahead)"
        elseif math.abs(rel) > 165 then
            clock_str = "6h (behind)"
        else
            local rel_norm = rel % 360
            local h        = math.floor(rel_norm / 30 + 0.5) % 12
            if h == 0 then h = 12 end
            clock_str = string.format("%dh", h)
        end

        gcs:send_text(5, string.format(
            "POI %s dist:%dm pos:%s",
            with_roi_lock and "locked" or "guided (gimbal free)",
            dist0, clock_str
        ))
        return true
    end

    -- ── RC7 HIGH rising edge: tracking ON + ROI locked ────────────────────
    if rc7_pwm > RC_HIGH_THRESHOLD and not active then
        activate_tracking(true)
    end

    -- ── RC7 MID rising edge: tracking ON + gimbal free ───────────────────
    if rc7_pwm >= RC_LOW_THRESHOLD and rc7_pwm <= RC_HIGH_THRESHOLD and not active then
        activate_tracking(false)
    end

    -- ── Falling edge: tracking off ────────────────────────────────────────
    if rc7_pwm < RC_LOW_THRESHOLD and active then
        active              = false
        roi_locked          = false
        orbit_notified      = false
        last_progress_step  = nil
        last_target_loc = nil
        last_roi_loc    = nil
        mount:set_mode(0, 3)   -- RC_TARGETING: gimbal free, responds to sticks
        if prev_mode then
            gcs:send_text(5, string.format("POI off, mode %d", prev_mode))
            vehicle:set_mode(prev_mode)
            prev_mode = nil
        end
    end

    -- ── Active tracking loop ──────────────────────────────────────────────
    if active then

        -- RC7 HIGH → ROI locked on target (no manual gimbal possible)
        -- RC7 MID  → GUIDED flight maintained, gimbal free (RC_TARGETING)
        local want_lock = rc7_pwm > RC_HIGH_THRESHOLD

        if want_lock and not roi_locked then
            -- MID → HIGH: engage ROI lock
            roi_locked = true
            mount:set_roi_target(0, last_roi_loc)
            gcs:send_text(5, "POI: ROI locked")
        elseif not want_lock and roi_locked then
            -- HIGH → MID: release ROI lock.
            -- Freeze the gimbal at its current attitude before switching to RC_TARGETING
            -- to avoid the jerk caused by the gimbal jumping to the RC stick neutral position.
            local cur_roll, cur_pitch, cur_yaw = mount:get_attitude_euler(0)
            if cur_roll and cur_pitch and cur_yaw then
                mount:set_angle_target(0, cur_roll, cur_pitch, cur_yaw, false)
            end
            roi_locked = false
            mount:set_mode(0, 3)   -- RC_TARGETING: gimbal responds to sticks
            gcs:send_text(5, "POI: gimbal free (RC7 mid)")
        end

        if roi_locked and not gimbal_neutral then
            -- Keep ROI locked every cycle
            mount:set_roi_target(0, last_roi_loc)
        end

        -- Distance tracking: progress notifications + arrival detection
        if not orbit_notified then
            local plane_loc = ahrs:get_location()
            if plane_loc then
                local dlat = (plane_loc:lat() - last_target_loc:lat()) * 1e-7
                local dlon = (plane_loc:lng() - last_target_loc:lng()) * 1e-7
                local plane_lat_rad = plane_loc:lat() * 1e-7 * math.pi / 180
                local dist2d = math.sqrt(
                    (dlat * 111320.0)^2 +
                    (dlon * 111320.0 * math.cos(plane_lat_rad))^2
                )

                if dist2d <= POI_ORBIT_RAD_M then
                    -- Arrival: final notification
                    orbit_notified     = true
                    last_progress_step = nil
                    gcs:send_text(5, string.format(
                        "POI: orbiting target (dist=%.0fm)", dist2d
                    ))
                else
                    -- Progress: notify every POI_PROGRESS_STEP_M meters closer
                    local step = math.floor(dist2d / POI_PROGRESS_STEP_M) * POI_PROGRESS_STEP_M
                    if last_progress_step == nil or step < last_progress_step then
                        last_progress_step = step
                        gcs:send_text(5, string.format(
                            "POI: %.0fm to target", dist2d
                        ))
                    end
                end
            end
        end

        -- Keep sending GUIDED waypoint so plane orbits the target
        vehicle:set_target_location(last_target_loc)
    end

    return update, UPDATE_MS
end

-- ─── Initialisation ──────────────────────────────────────────────────────────

gcs:send_text(5, "POI tracker loaded")
return update()
