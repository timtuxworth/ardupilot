--[[
    The avoidance MECHANISM for planedaa.lua: "where can I safely go?"

    This module decides nothing about what to do with an obstacle.  It gathers the threat
    picture and searches for a heading - the bendy-ruler sweep with its turn lead, the
    closest-point-of-approach assessment, aircraft and altitude-fence detection, and the
    DAAD/DAAS/DAAG logging that records those decisions.  Every judgement about what to SAY
    and what to COMMAND stays in planedaa.lua, so an integrator adding their own avoidance
    action edits that one file.  See planedaa.md.

    detect() returns a report rather than setting shared state, which is what lets the two
    halves live in different files:

        { target_loc, obstacle, aircraft, best_distance_m, distance_to_target_m }

    Everything from the applet is pushed in: new() takes the collaborators and constants,
    configure() the cached parameters, update_state() the per-cycle vehicle state.  They
    land in instance locals rather than fields because the candidate-heading sweep reads
    them well over a hundred times per cycle.
--]]

local DAAcore = {}

function DAAcore.new(deps)
    local self = {}

    -- collaborators and constants, fixed for the life of the instance
    local geometry              = deps.geometry
    local obstacles             = deps.obstacles
    local OBSTACLE_TYPE         = deps.OBSTACLE_TYPE
    local MAV_SEVERITY          = deps.MAV_SEVERITY
    local SCRIPT_NAME_SHORT     = deps.SCRIPT_NAME_SHORT
    local FLT_MAX               = deps.FLT_MAX
    local COARSE_SWEEP_MULT     = deps.COARSE_SWEEP_MULT
    local MIN_STEP2_M           = deps.MIN_STEP2_M
    local MIN_TURN_CHORD_M      = deps.MIN_TURN_CHORD_M
    local LOG_CLEARANCE_MAX_M   = deps.LOG_CLEARANCE_MAX_M
    -- ahrs, gcs, logger, fence and OAScripting are ArduPilot singletons: global in every chunk

    -- the module helpers this file leans on, bound once so the sweep reaches them as
    -- upvalues rather than through a table on every probe
    local wrap_360                  = geometry.wrap_360
    local wrap_180                  = geometry.wrap_180
    local location_project          = geometry.location_project
    local max_turn_rate_dps         = geometry.max_turn_rate_dps
    local effective_groundspeed     = geometry.effective_groundspeed
    local find_closest_obstacle     = obstacles.find_closest_obstacle
    local populate_obstacle         = obstacles.populate_obstacle
    local get_standoff              = obstacles.get_standoff

    -- cached parameters, pushed in by configure()
    local alt_cool_ms, alt_hyst_m, bearing_inc_deg, bendy_angle
    local bendy_ratio, cpa_min_ms, detect_m, margin_alt_m
    local margin_crewed_m, margin_fence_m, margin_vertical_m, plan_m
    local side_hold_s, slew_dps, slew_urg_s, well_clear_xy
    local well_clear_z, wp_loiter_rad_m, lookahead_param_m

    -- per-cycle vehicle state, pushed in by update_state()
    local current_loc, navigation_target_loc, airspeed_ms, groundspeed_ms, ground_course_deg, wind_speed, wind_dir_rad, now_ms

    -- the sweep's own state: nothing outside this module reads any of it
    local obstacle_avoiding         = nil
    local aircraft_avoiding         = nil
    local last_aircraft_obstacle    = nil
    local last_aircraft_ts_ms       = nil
    local last_avoid_bearing_deg    = nil
    local last_cmd_bearing_ms       = nil
    local committed_side_sign       = 0
    local side_flip_pending         = false
    local side_flip_want_ms         = uint32_t(0)
    local now_debug_ms              = millis()
    local current_lookahead         = 0
    local lookahead_set_m           = nil

    local function configure(settings)
        alt_cool_ms            = settings.alt_cool_ms
        alt_hyst_m             = settings.alt_hyst_m
        bearing_inc_deg        = settings.bearing_inc_deg
        bendy_angle            = settings.bendy_angle
        bendy_ratio            = settings.bendy_ratio
        cpa_min_ms             = settings.cpa_min_ms
        detect_m               = settings.detect_m
        margin_alt_m           = settings.margin_alt_m
        margin_crewed_m        = settings.margin_crewed_m
        margin_fence_m         = settings.margin_fence_m
        margin_vertical_m      = settings.margin_vertical_m
        plan_m                 = settings.plan_m
        side_hold_s            = settings.side_hold_s
        slew_dps               = settings.slew_dps
        slew_urg_s             = settings.slew_urg_s
        well_clear_xy          = settings.well_clear_xy
        well_clear_z           = settings.well_clear_z
        wp_loiter_rad_m        = settings.wp_loiter_rad_m
        lookahead_param_m      = settings.lookahead_param_m
        -- the working look-ahead follows the parameter, announcing the change once
        if lookahead_param_m ~= lookahead_set_m then
            lookahead_set_m   = lookahead_param_m
            current_lookahead = lookahead_param_m
        end
    end

    -- Positional, not a table: this runs every cycle the applet is active, and the sweep
    -- alone can call find_closest_obstacle() over a hundred times in one cycle, so a
    -- table literal here would be the single most frequent allocation in the script.
    local function update_state(new_current_loc, new_navigation_target_loc, new_airspeed_ms,
                                 new_groundspeed_ms, new_ground_course_deg,
                                 new_wind_speed, new_wind_dir_rad, new_now_ms)
        current_loc            = new_current_loc
        navigation_target_loc  = new_navigation_target_loc
        airspeed_ms            = new_airspeed_ms
        groundspeed_ms         = new_groundspeed_ms
        ground_course_deg      = new_ground_course_deg
        wind_speed             = new_wind_speed
        wind_dir_rad           = new_wind_dir_rad
        now_ms                 = new_now_ms
    end

    local function log_detect_result(obstacle_found, distance_found_m, best_distance_m, distance_to_target_m, best_bearing_deg, target_loc, obstacle_type)
        if target_loc == nil or distance_found_m == nil or distance_to_target_m == nil or best_bearing_deg == nil then
            -- we can't be avoiding if no target, so no loggin required
            return
        end
        local status, err = pcall(logger.write, logger, "DAAD",
            'Obs,DstF,DstB,DstT,HdgB,Tfnd,TLat,TLng,TAlt,TFra,ObjT',
            'BfffffLLfBI',                  -- Formats (L for Lat/Lng, f for Alt)
            '-mmmmdDUm--',                  -- Units (D=lat deg, U=lng deg, m=meter)
            '------GG---',                  -- Multipliers (G=1e-7 for L types)
            (obstacle_found and 1 or 0),    -- Obs - Obstacle found true/false
            distance_found_m,               -- DstF - clearance of the WORST heading in the sweep
            -- DstB - clearance of the heading we CHOSE (HdgB).  A heading that clears every
            -- obstacle reports FLT_MAX, so clamp it to something a log viewer can scale.
            math.max(math.min(best_distance_m, LOG_CLEARANCE_MAX_M), -LOG_CLEARANCE_MAX_M),
            distance_to_target_m,           -- DstT - Distance to proposed new target to avoid the obstacle
            wrap_360(best_bearing_deg),     -- HdgB - Best bearing found to avoid obstacles (0-360 deg)
            (target_loc ~= nil and 1 or 0), -- TFnd - Target found
            target_loc:lat(),               -- TLat - Latitude of proposed new target in degrees
            target_loc:lng(),               -- TLng - Longitude of proposed new target in degrees
            target_loc:alt() * 0.01,        -- TAlt - Alitude of proposed new target in meters
            target_loc:get_alt_frame(),     -- TFrm - Frame of the ALtitlde: 0: AMSL, 1: Home Relative, 3: Terrain Relative
            obstacle_type)                  -- ObjT - the OBSTACLE_TYPE of the object detected

        if not status then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " log detect:" .. tostring(err) )
        end
    end

    local function log_detect_aircraft(aircraft)
        -- a position-less contact (e.g. bearing-only ADS-B) has no location to log
        if aircraft == nil or aircraft.location == nil then
            return
        end

        local status, err = pcall(logger.write, logger, "DAAG",
            'DstF,TLat,TLng,TAlt,TFra,DstH,DstZ,ICAO',
            'fLLfBffI',                         -- Formats (L for Lat/Lng, f for Alt)
            'mDUm-mmh',                         -- Units (D=lat deg, U=lng deg, m=meter)
            '-GG-----',                         -- Multipliers (G=1e-7 for L types)
            aircraft.distance_m,                -- DstF - Distance to found aircraft in meters
            aircraft.location:lat(),            -- TLat - Latitude of proposed new target in degrees
            aircraft.location:lng(),            -- TLng - Longitude of proposed new target in degrees
            aircraft.location:alt() * 0.01,     -- TAlt - Alitude of proposed new target in meters
            aircraft.location:get_alt_frame(),  -- TFrm - Frame of the ALtitlde: 0: AMSL, 1: Home Relative, 3: Terrain Relative
            aircraft.distance_xy,               -- DstH - Horizontal distance to the aircraft
            aircraft.distance_z,                -- DstZ - Vertical distance to the aircraft (+ve is up)
            aircraft.icao_code                  -- ICAO - the integer value of the ICAO code of the aircraft if available
        )
        if not status then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " log aircraft:" .. tostring(err) )
        end
    end

    local function log_smoothing(direct_deg, raw_deg, resisted_deg, final_deg, side, flip, urgent, motion, obstacle)
        local status, err = pcall(logger.write, logger, "DAAS",
            'HdD,HdR,HdS,HdC,Sid,Flp,Urg,Cls,CPA,TTC,PsB,Dst,Typ',
            'ffffbBBfffbfB',                    -- Formats
            'dddd---nms-m-',                    -- Units (d=deg, n=m/s, m=metre, s=second)
            '-------------',                    -- Multipliers
            wrap_360(direct_deg),               -- HdD - direct bearing to target
            wrap_360(raw_deg),                  -- HdR - raw bendy-ruler bearing
            wrap_360(resisted_deg),             -- HdS - after clearance hysteresis (pre-smoothing)
            wrap_360(final_deg),                -- HdC - final commanded bearing (flown)
            side,                               -- Sid - committed side (-1 left / 0 / +1 right)
            (flip and 1 or 0),                  -- Flp - side-flip debounce pending
            (urgent and 1 or 0),                -- Urg - slew limit bypassed (urgent)
            motion.closing_speed,               -- Cls - closing speed
            motion.cpa_miss,                    -- CPA - predicted horizontal miss distance
            math.min(motion.ttc, 999.0),        -- TTC - time until the keep-out boundary is crossed (capped)
            motion.pass_behind,                 -- PsB - side that passes behind the obstacle
            obstacle.distance_m,                -- Dst - range to the obstacle
            obstacle.type)                      -- Typ - OBSTACLE_TYPE
        if not status then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " log smoothing:" .. tostring(err))
        end
    end

    local function calc_avoidance_distance(avoid_step1_m, target_distance)
        -- test for flying past the waypoint, so if we are close, we have room to dodge after the waypoint
        return math.min(avoid_step1_m, target_distance + math.min(margin_fence_m / 2, 100))
    end

    -- Forward-declared: resist_bearing_change() (defined here, ahead of probe_bearing in
    -- the file) needs to call it, and needs the SAME turn-lead-aware measurement probe_bearing
    -- gives every other candidate in the sweep.  Assigned without "local" at its usual
    -- location further down; this is the upvalue resist_bearing_change closes over.
    local probe_bearing

    --[[
    This function is called when BendyRuler has found a bearing which is obstacles free at at least lookahead_step1_dist and  then lookahead_step2_dist from the present location
    In many situations, this new bearing can be either left or right of the obstacle, and BendyRuler can have a tough time deciding between the two.
    It has the tendency to move the vehicle back and forth, if the margin obtained is even slightly better in the newer iteration.
    Therefore, this method attempts to avoid changing direction of the vehicle by more than _bendy_angle degrees,
    unless the new margin is atleast _bendy_ratio times better than the margin with previously calculated bearing.
    We return true if we have resisted the change and will follow the last calculated bearing.

    Returns (bearing, distance): the distance is the clearance of WHICHEVER bearing is
    returned, not of the candidate that was proposed.  Getting this right matters for
    logging: DAAD.DstB used to be left as the proposed candidate's distance even on the
    "stay the course" path, so it could read as fully clear while the bearing actually
    being flown was not - see project_planedaa_standoff_not_achieved in memory.
    --]]
    local function resist_bearing_change(bearing_orig_deg, bearing_deg, distance_found_m, target_loc)
        if bearing_orig_deg == nil then
            -- no prior commitment, accept the proposed bearing
            return bearing_deg, distance_found_m
        end
        if distance_found_m == 0 then
            -- obstacle is immediate, must manoeuvre regardless
            return bearing_deg, distance_found_m
        end
        if math.abs(wrap_180(bearing_orig_deg - bearing_deg)) < bendy_angle then
            -- proposed change is small enough, no resistance needed
            return bearing_deg, distance_found_m
        end
        if current_loc == nil then
            -- no current position to measure against, accept the proposed bearing
            return bearing_deg, distance_found_m
        end
        -- Measure clearance in the previously committed direction using the SAME
        -- turn-lead-aware probe every other candidate in the sweep goes through, not a
        -- bare straight-line offset.  A straight-line retest here compared an
        -- already-turn-corrected new candidate against an uncorrected retest of the old
        -- one - biasing this decision toward "stay the course" whenever the committed
        -- bearing itself needed the turn lead to read accurately, which is exactly the
        -- geometry of continuing to track an avoidance already under way.
        --
        -- Pass FLT_MAX, not the real distance to the navigation target, as probe_bearing's
        -- full_distance.  That argument exists so a NEW candidate step does not overshoot a
        -- nearby target; it has nothing to do with this retest, whose only question is
        -- whether the OLD committed bearing is still safe over the full look-ahead.  Passing
        -- the real distance here made the retest see only current_lookahead's near end - a
        -- fence sitting just past that bounded window went undetected until 1-2 cycles
        -- before the aircraft reached it.  Measured on a live SITL flight: the equivalent
        -- straight-line probe (what shipped before this file's turn-lead work) would have
        -- flagged the same bearing unsafe 8 seconds earlier, well before the eventual breach.
        local distance_previous_m = probe_bearing(bearing_orig_deg, bearing_orig_deg,
                                                   FLT_MAX, target_loc, false)
        -- Only switch sides if the new direction is significantly better: POSITIVE clearance
        -- and bendy_ratio times clearer than continuing.  The positive-clearance requirement is
        -- what makes this negative-aware: when hugging a boundary both clearances read near-zero
        -- or negative and a plain ratio test flip-flops every cycle (the fence-skirt oscillation),
        -- so we hold the committed escape direction through the hug.  It still switches away from
        -- a committed side that is itself breaching (distance_previous_m < 0) towards a side that
        -- actually clears (distance_found_m > 0), so containment is preserved.
        if distance_found_m > 0 and distance_found_m >= bendy_ratio * distance_previous_m then
            return bearing_deg, distance_found_m
        end
        -- new direction is not significantly better — stay the course
        return bearing_orig_deg, distance_previous_m
    end

    --[[
    Velocity-aware assessment of a (possibly moving) obstacle. Uses the obstacle's
    ADS-B velocity plus our own velocity to reason about the encounter over time
    rather than from its instantaneous position (which is what makes bendy ruler
    wiggle against a moving target). Returns:
      is_conflict   - false when a moving obstacle is opening range and its predicted
                      closest approach stays beyond the well-clear distance (it is leaving)
      pass_behind   - +1/-1 the side of the direct bearing that passes behind the
                      obstacle's track (0 when it is not usefully moving)
      ttc_s         - estimated time to closest approach, for the slew-rate urgency test
    Static obstacles (fences, ~zero velocity) return (true, 0, closing-based ttc) so
    their behaviour is unchanged.
    --]]
    local function assess_obstacle_motion(obstacle)
        if obstacle == nil or current_loc == nil or obstacle.location == nil then
            -- no geometry to assess: treat as a conflict (the safe default is to avoid)
            return { is_conflict = true, closing_speed = 0.0, cpa_miss = 0.0, ttc = FLT_MAX, pass_behind = 0 }
        end
        local rel = current_loc:get_distance_NED(obstacle.location)  -- N,E,D metres to the obstacle
        local rn, re = rel:x(), rel:y()
        local range_h = math.sqrt(rn * rn + re * re)

        local ov = obstacle.vel_NED_ms
        local own = ahrs:get_velocity_NED()
        local ovn = (ov ~= nil) and ov:x() or 0.0
        local ove = (ov ~= nil) and ov:y() or 0.0
        local rvn = ovn - ((own ~= nil) and own:x() or 0.0)   -- relative velocity (obstacle - own), North
        local rve = ove - ((own ~= nil) and own:y() or 0.0)   -- East

        local rel_dot_rv = rn * rvn + re * rve                -- < 0 => range decreasing (closing)
        local rv2 = rvn * rvn + rve * rve
        local closing_speed = (range_h > 0.1) and (-rel_dot_rv / range_h) or 0.0

        -- horizontal closest point of approach
        local t_cpa = (rv2 > 1e-4) and math.max(0.0, -rel_dot_rv / rv2) or 0.0
        local miss_n = rn + rvn * t_cpa
        local miss_e = re + rve * t_cpa
        local cpa_miss_h = math.sqrt(miss_n * miss_n + miss_e * miss_e)

        -- Type-aware keep-out radius: the miss distance below which this obstacle is a conflict.
        -- The range check uses the same value, so any obstacle already inside the keep-out radius
        -- is unconditionally a conflict (the conservative floor); only one that will miss beyond it
        -- AND is not closing AND is already beyond it is treated as leaving (no manoeuvre needed).
        local standoff_m = get_standoff(obstacle.type)

        -- Time until the obstacle crosses that keep-out boundary: the earlier root of
        -- |rel + rv*t| = standoff_m.  This was range_h / closing_speed, which is the time to
        -- ZERO range - a point crossing traffic never reaches, so it read later than the
        -- encounter really was and left the slew limiter damping turns that had no time left.
        local ttc_s = FLT_MAX
        if rv2 > 1e-4 then
            local c_term = range_h * range_h - standoff_m * standoff_m
            if c_term <= 0.0 then
                ttc_s = 0.0                     -- already inside the keep-out radius
            else
                local disc = rel_dot_rv * rel_dot_rv - rv2 * c_term
                if disc >= 0.0 then
                    local t_enter = (-rel_dot_rv - math.sqrt(disc)) / rv2
                    if t_enter >= 0.0 then
                        ttc_s = t_enter         -- negative => boundary behind us, receding
                    end
                end
            end
        end
        local is_conflict = true
        if cpa_miss_h > standoff_m and closing_speed < cpa_min_ms and range_h > standoff_m then
            is_conflict = false
        end

        -- side of the direct bearing that passes behind the obstacle's track
        local pass_behind = 0
        if ov ~= nil and (math.abs(ovn) + math.abs(ove)) > 0.5 then
            local cross = rn * ove - re * ovn                 -- (rel x obstacle_vel) vertical component
            if cross > 0.0 then pass_behind = -1 elseif cross < 0.0 then pass_behind = 1 end
        end

        return {
            is_conflict   = is_conflict,
            closing_speed = closing_speed,
            cpa_miss      = cpa_miss_h,
            ttc           = ttc_s,
            pass_behind   = pass_behind,
        }
    end

    --[[
    Post-process the raw bendy-ruler heading into a smooth command for a MOVING
    obstacle, WITHOUT ever overriding a turn the sweep needs to clear an obstacle.

    resist_bearing_change() already produces a bearing that clears ALL obstacles
    (fences included) and only makes a large change when the new side is clearly
    clearer. So:
      * An urgent encounter, or a large change there is no time to damp, is a
        genuine avoidance turn -> obey it exactly, no smoothing.  (This is the
        safety fix: previously the side-commit could mirror such a turn onto the
        committed side and the slew limit could throttle it, flying the aircraft
        into the very fence the sweep was turning away from.)
      * "No time to damp" is a question about TIME, not about size.  It used to be
        size alone - any change over DAA_BR_ANGLE bypassed the smoothing - which
        meant the one case the smoothing was built for was the one case that
        reached the servos completely unfiltered: a ~180 degree side flip is always
        over DAA_BR_ANGLE.  With two obstacles in range the single-obstacle sweep
        alternates between them, and every alternation looked like a "necessary"
        reversal.  Flight log_87 (2026-08-27) commanded 34 changes over 45 degrees
        in one sortie, the worst 180 degrees in 0.26 s, none of them urgent.
        Asking instead whether the slew limit can deliver the change before the
        conflict leaves only the 3 that really were out of time.
      * Only SMALL residual changes (the left/right jitter) are damped, via a side
        commitment and a heading slew-rate limit. When holding a committed side we
        keep the LAST FLOWN bearing (known clear) rather than a mirrored one that
        was never clearance-checked.
    --]]
    -- (2) Side commitment: once a side is chosen, hold it until the sweep has wanted the
    -- other one continuously for DAA_SIDE_HOLD_S.  Returns the bearing to fly.
    local function apply_side_commitment(bearing, side, pass_behind)
        if side_hold_s <= 0 then
            return bearing
        end
        if committed_side_sign == 0 then
            -- fresh episode: commit; prefer passing behind a moving obstacle
            committed_side_sign = (pass_behind ~= 0) and pass_behind or side
            side_flip_pending = false
            return bearing
        end
        if side == 0 or side == committed_side_sign then
            side_flip_pending = false
            return bearing
        end
        -- sweep wants the opposite side: only honour it once it has persisted; meanwhile
        -- hold the last flown (clearance-proven) bearing, do NOT mirror
        if not side_flip_pending then
            side_flip_pending = true
            side_flip_want_ms = now_ms
        end
        if (now_ms - side_flip_want_ms) < (side_hold_s * 1000) then
            return last_avoid_bearing_deg
        end
        committed_side_sign = side
        side_flip_pending = false
        return bearing
    end

    -- (3) Heading slew-rate limit, for small changes only - large turns bypass this.
    local function apply_slew_limit(bearing)
        if slew_dps <= 0 or last_avoid_bearing_deg == nil then
            return bearing
        end
        local dt = (now_ms - last_cmd_bearing_ms):tofloat() / 1000.0
        local max_step = slew_dps * dt
        if max_step <= 0 then
            return bearing
        end
        local d = wrap_180(bearing - last_avoid_bearing_deg)
        if d > max_step then d = max_step elseif d < -max_step then d = -max_step end
        return wrap_360(last_avoid_bearing_deg + d)
    end

    local function refine_avoidance_bearing(direct_bearing_deg, raw_bearing_deg, raw_distance_m,
                                            motion, obstacle, target_loc)
        local pass_behind = motion.pass_behind
        local ttc_s = motion.ttc

        -- (1) clearance hysteresis: the safe, multi-obstacle, anti-flip baseline.  The
        -- returned distance is only accurate for `resisted` itself - apply_side_commitment
        -- and apply_slew_limit below can move the bearing further without a matching
        -- re-measurement, so a bearing damped by either is logged against this value as
        -- the best available figure, not a guaranteed-exact one.
        local resisted, resisted_distance_m =
                resist_bearing_change(last_avoid_bearing_deg, raw_bearing_deg, raw_distance_m, target_loc)
        local bearing = resisted

        local urgent = (ttc_s ~= nil) and (ttc_s < slew_urg_s)
        local change = (last_avoid_bearing_deg ~= nil) and math.abs(wrap_180(resisted - last_avoid_bearing_deg)) or 999.0

        -- which side of the direct bearing the sweep wants this cycle
        local off = wrap_180(resisted - direct_bearing_deg)
        local side = 0
        if off > 1.0 then side = 1 elseif off < -1.0 then side = -1 end

        -- Obey a large change exactly ONLY when damping it would make us late: if
        -- slewing at DAA_SLEW_DPS cannot cover the change before the conflict, there
        -- is no time left to be smooth.  ttc_s is FLT_MAX when nothing is closing, so
        -- a manoeuvre with time in hand always damps.
        local no_time_to_damp = change > bendy_angle and slew_dps > 0
                                and (ttc_s * slew_dps) < change

        if last_avoid_bearing_deg == nil or urgent or no_time_to_damp then
            -- first cycle, or a turn there is no time to damp: obey the sweep exactly.
            -- Record the side we are going round, so that a later reversal is seen as
            -- a flip to be debounced rather than as another "necessary" turn; leave it
            -- uncommitted on the very first cycle so pass_behind can choose below.
            committed_side_sign = (last_avoid_bearing_deg == nil) and 0 or side
            side_flip_pending = false
        else
            -- small adjustment only: damp the jitter ('side' was computed above from this
            -- same bearing)
            bearing = apply_side_commitment(bearing, side, pass_behind)
            bearing = apply_slew_limit(bearing)
        end
        last_cmd_bearing_ms = now_ms

        log_smoothing(direct_bearing_deg, raw_bearing_deg, resisted, bearing,
                      committed_side_sign, side_flip_pending, urgent, motion, obstacle)
        return bearing, resisted_distance_m
    end

    -- Second step of the bendy ruler test: having flown one step out to loc_test, look a
    -- further avoid_step2_m ahead to check this heading does not lead into a dead end.
    -- Mirrors AP_OABendyRuler::search_xy_path(): the three probes fan +/-45 degrees around
    -- the bearing from loc_test to the real destination (not around the step-1 heading),
    -- the leg is capped by the distance still to run, and the segment tested starts AT
    -- loc_test so it is contiguous with step 1, which covered current_loc -> loc_test.
    -- Returns the clearance found and whether the probe that produced it was the straight
    -- one (delta == 0); the caller needs that to decide "no avoidance required" without
    -- comparing recomputed bearings for equality.
    -- Reuse these offsets: test_step2() may run for many candidate headings in one
    -- update, so building this table in the function adds repeated heap churn.
    local test_bearings = { 0, 45, -45 }
    local function test_step2(loc_test, avoid_step2_m, destination_loc)
        local bearing_to_dest_deg   = math.deg(loc_test:get_bearing(destination_loc))
        local distance2_m           = math.max(math.min(avoid_step2_m,
                                                       loc_test:get_distance(destination_loc)), MIN_STEP2_M)

        local closest_distance_m    = FLT_MAX
        local closest_obstacle      = nil
        local straight              = false

        for _, delta in ipairs(test_bearings) do
            local bearing_test  = wrap_180(bearing_to_dest_deg + delta)
            local loc_test2     = location_project(loc_test, bearing_test, distance2_m, destination_loc)

            local distance_m, obstacle = find_closest_obstacle(loc_test, loc_test2, detect_m, wind_speed)

            if distance_m > detect_m then
                -- return immediately - no obstacles in this direction
                return distance_m, (delta == 0), nil
            end
            if distance_m < closest_distance_m then
                -- remember the worst blocker we saw, and whether the probe that hit it
                -- was the straight-at-the-destination one
                closest_distance_m  = distance_m
                closest_obstacle    = obstacle
                straight            = (delta == 0)
            end
        end

        return closest_distance_m, straight, closest_obstacle
    end

    -- Where the vehicle will actually BE once it has turned onto course_deg.  A candidate
    -- course is not flown from where we are now: getting onto it costs a turn, and the arc
    -- of that turn carries the vehicle up to 2R towards whatever lies on the inside of it.
    -- At 25 m/s and 60 deg of roll R is ~35 m, so a reversal displaces ~70 m - more than a
    -- fence standoff - and a heading judged from the present position can read as clear and
    -- still fly the vehicle through the fence.
    --
    -- Displacement around a circular arc of turn angle th is R*sin(th) along the current
    -- ground course plus R*(1 - cos(th)) to the side of the turn.  R is derived from the arc
    -- length the vehicle will actually cover (ground speed x turn time) rather than from
    -- airspeed, so the wind-corrected ground speed carries through and a near-zero turn
    -- degrades to "carry straight on".
    local function location_after_course_change(from_loc, course_deg, to_loc)
        local course_change_deg = wrap_180(course_deg - ground_course_deg)
        local ground_speed_ms   = effective_groundspeed(airspeed_ms, course_deg, wind_dir_rad, wind_speed)
        local rate_of_turn_dps  = max_turn_rate_dps(airspeed_ms)

        if rate_of_turn_dps <= 0 or ground_speed_ms <= 0 then
            return from_loc                 -- no usable speed to turn at
        end
        local arc_length_m  = ground_speed_ms * (math.abs(course_change_deg) / rate_of_turn_dps)
        local turn_rad      = math.rad(math.abs(course_change_deg))
        if turn_rad < 1e-3 then
            return location_project(from_loc, ground_course_deg, arc_length_m, to_loc)
        end
        local radius_m      = arc_length_m / turn_rad
        -- Which way we turn: a reversal has no short way round, so the side is whatever
        -- wrap_180 gives it.  The sweep tries both sides of the target bearing, so the
        -- opposite reversal is still costed - as its own candidate, with its own bulge.
        local side_deg      = (course_change_deg >= 0) and (ground_course_deg + 90) or (ground_course_deg - 90)

        local projected_loc = location_project(from_loc, ground_course_deg, radius_m * math.sin(turn_rad), to_loc)
        return location_project(projected_loc, side_deg, radius_m * (1.0 - math.cos(turn_rad)), to_loc)
    end

    -- The straight leg above starts where the turn ENDS, so on its own it never looks at the
    -- ground the turn itself covers.  One extra probe along the chord from here to that point
    -- closes the gap: against log 161's exclusion circles it caught every case that sampling
    -- the arc in eight segments caught, and none were missed.  Only worth its cost once the
    -- turn is big enough to bow away from the leg - below DAA_BR_ANGLE the chord lies along
    -- it - which also skips it for the small-deflection candidates the sweep usually wins on.
    local function probe_turn_arc(adjusted_loc, bearing_test_deg, distance_found_m, obstacle_found)
        if math.abs(wrap_180(bearing_test_deg - ground_course_deg)) <= bendy_angle then
            return distance_found_m, obstacle_found
        end
        if current_loc:get_distance(adjusted_loc) < MIN_TURN_CHORD_M then
            return distance_found_m, obstacle_found  -- too short to have a meaningful bearing
        end
        local turn_distance_m, turn_obstacle =
                find_closest_obstacle(current_loc, adjusted_loc, detect_m, wind_speed)
        if turn_distance_m ~= nil and turn_distance_m < distance_found_m then
            return turn_distance_m, turn_obstacle
        end
        return distance_found_m, obstacle_found
    end

    -- Core clearance probe for an explicit candidate course bearing_test_deg. Returns
    -- (distance_found_m, bearing_test_deg, obstacle_found); a clear course returns
    -- FLT_MAX with obstacle_found == nil. allow_straight lets the unobstructed
    -- straight-ahead path short-circuit (only meaningful for the i == 0 candidate).
    -- (forward-declared above, for resist_bearing_change's benefit)
    probe_bearing = function(bearing_test_deg, bearing_deg, full_distance, target_loc, allow_straight)
        local avoid_step1_m     = current_lookahead
        local avoid_step2_m     = current_lookahead * 2.0

        -- Start the look-ahead from where we will actually be after turning onto this
        -- candidate course.  This used to be applied only in wind, as a way of leading the
        -- carrot downwind; it is really a TURN lead and calm air needs it just as much.
        -- Without it the probe assumes the vehicle is already on the candidate course, so a
        -- heading that needs a reversal is judged against a path the turn never flies.
        local adjusted_loc          = location_after_course_change(current_loc, bearing_test_deg, target_loc)

        -- Position after one step from where we think we will be after turning to bearing_test_deg
        local avoidance_distance_m  = calc_avoidance_distance(avoid_step1_m, full_distance)
        local test_loc              = location_project(adjusted_loc, bearing_test_deg, avoidance_distance_m, target_loc)

        local distance_found_m, obstacle_found = find_closest_obstacle(adjusted_loc, test_loc, detect_m, wind_speed)
        if distance_found_m == nil then
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. "closest returned NIL ")
            return FLT_MAX, bearing_deg, nil -- no avoidance required
        end
        distance_found_m, obstacle_found = probe_turn_arc(adjusted_loc, bearing_test_deg,
                                                         distance_found_m, obstacle_found)
        if distance_found_m > detect_m then
            -- This direction avoids all obstacles for one step. Check if it leads to a clear path for a longer distance.
            local distance2_m, straight2, obstacle2 = test_step2(test_loc, avoid_step2_m, target_loc)
            if distance2_m >= detect_m then
                if allow_straight and straight2 then
                    -- means we have a direct unobstructed path for step1 and step2
                    return FLT_MAX, bearing_deg, nil -- no avoidance required
                end
                -- we've found at least one direction where there is no obstacle at least for 2 steps out
                distance_found_m = distance_found_m + distance2_m
            elseif obstacle2 ~= nil then
                -- All three second-leg probes are blocked: this heading is clear for one
                -- step but leads into a dead end.  Report the blocker so self.detect() keeps
                -- sweeping (it ends the sweep on a nil obstacle, so without this the step-2
                -- result was discarded and the heading was accepted as clear).
                return distance2_m, bearing_test_deg, obstacle2
            end
        end

        return distance_found_m, bearing_test_deg, obstacle_found
    end

    -- This method checks whether we will collide with any obstacle if we fly at a given bearing bearing_deg + i * inc_deg
    -- inc_deg is the sweep step: the coarse pass of self.detect() passes a multiple of
    -- DAA_HEADING_INC, and the refine pass probes explicit bearings via probe_bearing().
    local function test_step1(full_distance, bearing_deg, i, target_loc, inc_deg)
        local bearing_delta_deg = i * inc_deg / 2.0
        if i % 2 == 1 then
            -- Alternate between left and right of the target
            bearing_delta_deg = -bearing_delta_deg
        end
        local bearing_test_deg = wrap_180(bearing_deg + bearing_delta_deg)
        return probe_bearing(bearing_test_deg, bearing_deg, full_distance, target_loc, i == 0)
    end

    -- if the plane is currently pointing far away from the target, then assume that we
    -- will be turning sharply, so we don't look too far ahead for obstacles
    local function limit_distance(from_loc, to_loc, bearing_deg)
        local distance_to_target_m = from_loc:get_distance(to_loc)

        if (math.abs(wrap_180(bearing_deg - ground_course_deg)) > bendy_angle * 2) then
            distance_to_target_m = wp_loiter_rad_m * 3
        end

        return distance_to_target_m
    end

    -- AC_FENCE_TYPE bits (see AC_Fence.h) for the altitude fences we handle here
    local FENCE_TYPE_ALT_MAX = 1    -- FENCE_TYPE bit 0
    local FENCE_TYPE_ALT_MIN = 8    -- FENCE_TYPE bit 3

    -- Clamp a target location's altitude into the safe altitude-fence band, leaving a DAA_MARGIN_ALT buffer
    -- inside the fence's own safe limits. This is the "continue" half of clamp-and-continue: the horizontal
    -- path is untouched, only the commanded altitude is corrected. Applied to every target we command via
    -- update_target_location(), so it also enforces the band while avoiding a horizontal obstacle.
    local function clamp_alt_to_fence(loc)
        if loc == nil or fence == nil then
            return
        end
        local enabled = fence:get_enabled_fences()
        if (enabled & FENCE_TYPE_ALT_MAX) ~= 0 then
            local safe_max_alt_m, max_alt_frame = fence:get_safe_alt_max()
            local ceiling_m = safe_max_alt_m - margin_alt_m
            local current_alt_m = loc:get_alt_m(max_alt_frame)
            if current_alt_m ~= nil and current_alt_m > ceiling_m then
                loc:set_alt_m(ceiling_m, max_alt_frame)
            end
        end
        if (enabled & FENCE_TYPE_ALT_MIN) ~= 0 then
            local safe_min_alt_m, min_alt_frame = fence:get_safe_alt_min()
            local floor_alt_m = safe_min_alt_m + margin_alt_m
            local current_alt_m = loc:get_alt_m(min_alt_frame)
            if current_alt_m ~= nil and current_alt_m < floor_alt_m then
                loc:set_alt_m(floor_alt_m, min_alt_frame)
            end
        end
    end

    -- altitude fences have no horizontal location, so build a lightweight obstacle for alerting/telemetry only.
    -- headroom_m is the (positive) distance from the current altitude to the safe fence limit, reported in the alert.
    local function make_alt_fence_obstacle(otype, label_str, headroom_m)
        local obstacle = {}
        obstacle.distance_m  = headroom_m
        obstacle.sysid       = 0
        obstacle.icao_code   = 0
        obstacle.type        = otype
        obstacle.label       = label_str
        obstacle.location    = nil
        obstacle.pos_NED_m   = nil
        obstacle.vel_NED_ms  = nil
        obstacle.distance_xy = headroom_m
        obstacle.distance_z  = headroom_m
        return obstacle
    end

    -- latch + hysteresis for the altitude-fence trigger. The proactive projection crosses the limit
    -- intermittently as the plane climbs toward then levels off at the clamp altitude; without a latch
    -- the trigger toggles and re-alerts every few seconds. Once engaged we stay engaged (so the alert
    -- de-dupes to one message and the clamp holds steady) until the plane is clearly back in safe air.
    local alt_fence_active  = false
    local alt_fence_near    = false
    local last_alt_alert_ms = uint32_t(0)

    -- Proactively detect that we are approaching (or projected to cross) an altitude fence.
    -- Only kicks in for the fences enabled in FENCE_TYPE: bit 0 (ALT_MAX) and/or bit 3 (ALT_MIN).
    -- The vertical position is projected forward using the current climb rate over the time it takes to fly
    -- the lookahead distance (capped to a sane vertical horizon) so we level off before the band is reached.
    -- Returns a synthetic obstacle while corrective action is needed (latched), otherwise nil.
    local function detect_altitude_fence()
        if fence == nil or current_loc == nil then
            alt_fence_active    = false
            alt_fence_near      = false
            return nil
        end
        local enabled = fence:get_enabled_fences()
        local alt_max_on = (enabled & FENCE_TYPE_ALT_MAX) ~= 0
        local alt_min_on = (enabled & FENCE_TYPE_ALT_MIN) ~= 0
        if not alt_max_on and not alt_min_on then
            alt_fence_active    = false
            alt_fence_near = false
            return nil
        end

        -- climb rate (m/s, positive up) for proactive projection
        local climb_rate_ms = 0.0
        local vel_ned = ahrs:get_velocity_NED()
        if vel_ned ~= nil then
            climb_rate_ms = -vel_ned:z()
        end
        -- project over the time to fly the lookahead distance, capped to a sensible vertical horizon
        local horizon_s = detect_m / math.max(groundspeed_ms, 1.0)
        horizon_s = math.min(math.max(horizon_s, 1.0), 20.0)

        -- pick whichever enabled altitude fence currently needs (or is already taking) action.
        -- enter when current or projected altitude is past the clamp limit; while latched, only release
        -- once we are DAA_ALT_HYST_M clear of the limit on both current and projected altitude.
        local otype, label_str, headroom_m
        if alt_max_on then
            local safe_max_alt_m, max_alt_frame = fence:get_safe_alt_max()
            local current_alt_m = current_loc:get_alt_m(max_alt_frame)
            if current_alt_m ~= nil then
                local ceiling_m = safe_max_alt_m - margin_alt_m
                local projected_alt_m = current_alt_m + climb_rate_ms * horizon_s
                local enter = current_alt_m > ceiling_m or projected_alt_m > ceiling_m
                local clear = current_alt_m < ceiling_m - alt_hyst_m and projected_alt_m < ceiling_m - alt_hyst_m
                if enter or (alt_fence_active and not clear) then
                    otype, label_str = OBSTACLE_TYPE.FENCE_ALT_MAX, "Alt Max Fence"
                    headroom_m = safe_max_alt_m - current_alt_m     -- metres below the safe ceiling
                end
            end
        end
        if label_str == nil and alt_min_on then
            local safe_min_alt_m, min_alt_frame = fence:get_safe_alt_min()
            local current_alt_m = current_loc:get_alt_m(min_alt_frame)
            if current_alt_m ~= nil then
                local floor_alt_m = safe_min_alt_m + margin_alt_m
                local projected_alt_m = current_alt_m + climb_rate_ms * horizon_s
                local enter = current_alt_m < floor_alt_m or projected_alt_m < floor_alt_m
                local clear = current_alt_m > floor_alt_m + alt_hyst_m and projected_alt_m > floor_alt_m + alt_hyst_m
                if enter or (alt_fence_active and not clear) then
                    otype, label_str = OBSTACLE_TYPE.FENCE_ALT_MIN, "Alt Min Fence"
                    headroom_m = current_alt_m - safe_min_alt_m     -- metres above the safe floor
                end
            end
        end

        local now_active = label_str ~= nil

        -- Announce once when we actually level off near the limit (within the clamp band), not while
        -- merely projecting a distant crossing. The "near" latch + cooldown collapses the brief
        -- trigger drop-outs during a long climb/descent into a single notice; the steady clamp is silent.
        -- The reported distance is the steady-state clearance the plane settles at (DAA_MARGIN_ALT),
        -- not the trigger headroom.
        local near = now_active and headroom_m ~= nil and headroom_m <= (margin_alt_m + alt_hyst_m)
        if near and not alt_fence_near and (now_ms - last_alt_alert_ms) > alt_cool_ms then
            gcs:send_named_string("DAA-ALERT", "alt-fence")
            gcs:send_named_string("DAA-OBSTCL", label_str)
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(" levelling off %.0fm from %s",
                                margin_alt_m, label_str))
            gcs:send_named_float("DAA-DISTZ", margin_alt_m)
            last_alt_alert_ms = now_ms
        end

        alt_fence_near      = near
        alt_fence_active    = now_active
        if not now_active then
            return nil
        end
        return make_alt_fence_obstacle(otype, label_str, math.max(headroom_m, 0.0))
    end

    -- crewed aircraft are a special case. We do specific things if there is an aircraft nearby so we need to know the nearest one
    local function detect_aircraft()
        if current_loc == nil then
            aircraft_avoiding = nil
            last_aircraft_obstacle = nil
            last_aircraft_ts_ms = nil
            return
        end

        -- search out to the well clear distance (plus the crewed-aircraft margin), matching the
        -- treatment in the bendy ruler path, so aircraft are detected and logged at a
        -- useful range rather than only once they are within DAA_MARGIN_CA of us
        -- pass the full gate distance for each axis (computed here, applied in C++): the
        -- horizontal gate is well_clear_xy + margin_crewed_m, the vertical gate is
        -- well_clear_z + margin_vertical_m
        local distance_m, aircraft_obstacle = OAScripting:find_aircraft(current_loc, well_clear_xy + margin_crewed_m, well_clear_z + margin_vertical_m)

        if distance_m == nil or aircraft_obstacle == nil then
            aircraft_avoiding       = nil
            last_aircraft_obstacle  = nil
            last_aircraft_ts_ms     = nil
            return
        end

        -- De-bounce the oversampled ADS-B feed: AP_Avoidance re-reports the same fix many
        -- times between genuine updates (~63% of DAAG records were duplicate lat/lng in
        -- log_102). Act only on a fresh fix (new timestamp_ms); on a repeat, reuse the last
        -- obstacle so the loiter latch holds without re-populating or re-logging every
        -- cycle. The timestamp change is the true (~1 Hz) fix rate.
        local ts_ms = aircraft_obstacle:timestamp_ms()
        if last_aircraft_obstacle ~= nil and ts_ms == last_aircraft_ts_ms then
            aircraft_avoiding = last_aircraft_obstacle
            return
        end

        local obstacle = populate_obstacle(distance_m, aircraft_obstacle)

        aircraft_avoiding       = obstacle
        last_aircraft_obstacle  = obstacle
        last_aircraft_ts_ms     = ts_ms

        log_detect_aircraft(aircraft_avoiding)
    end


    -- Coarse pass of the sweep: step at coarse_inc_deg around the full circle (test_step1
    -- alternates left/right) until a clear heading is found or every candidate is exhausted.
    -- Returns clear_delta_deg (the signed deflection of the first clear heading, or nil if
    -- boxed in - nothing cleared) and obstacle_distance_m (the worst blocker seen, needed by
    -- the boxed-in refine below), plus the possibly-improved best_distance_m/best_bearing_deg
    -- (every probe can improve these, clear or not).  A separate function rather than a
    -- goto-out-of-loop: returning on the first clear heading is the same short-circuit,
    -- without a jump target.
    local function coarse_sweep(bearing_deg, distance_to_target_m, target_loc, coarse_inc_deg,
                                best_distance_m, best_bearing_deg)
        local obstacle_distance_m = FLT_MAX
        for i = 0, math.floor(360 / coarse_inc_deg) do
            local distance_found_m, bearing_found_deg, obstacle_found =
                    test_step1(distance_to_target_m, bearing_deg, i, target_loc, coarse_inc_deg)
            if distance_found_m > best_distance_m then
                best_distance_m     = distance_found_m
                best_bearing_deg    = bearing_found_deg
            end
            if obstacle_found == nil then -- found a path with no obstacles - done!
                return wrap_180(bearing_found_deg - bearing_deg), obstacle_distance_m,
                       best_distance_m, best_bearing_deg
            end
            if distance_found_m < obstacle_distance_m then
                obstacle_avoiding   = obstacle_found
                obstacle_distance_m = distance_found_m
            end
        end
        return nil, obstacle_distance_m, best_distance_m, best_bearing_deg
    end

    -- Probe one refine candidate at centre_deg + delta_deg, shared by both refine loops in
    -- sweep_for_heading() below.  They differ in what centre and delta sequence they walk
    -- (see the comments at each call site) but do the same thing with each candidate: adopt
    -- it if it clears, otherwise fold it into the running worst-blocker distance so the
    -- boxed-in case still has a best-available fallback.  Returns cleared (true/false) and,
    -- only when cleared, the distance/bearing to adopt; obstacle_distance_m is always
    -- returned since the caller's running total must carry across candidates that don't clear.
    local function probe_refine_candidate(centre_deg, delta_deg, direct_bearing_deg,
                                          distance_to_target_m, target_loc, obstacle_distance_m)
        local distance_found_m, bearing_found_deg, obstacle_found =
                probe_bearing(wrap_180(centre_deg + delta_deg), direct_bearing_deg,
                             distance_to_target_m, target_loc, false)
        if obstacle_found == nil then
            return true, distance_found_m, bearing_found_deg, obstacle_distance_m
        end
        if distance_found_m < obstacle_distance_m then
            obstacle_avoiding   = obstacle_found
            obstacle_distance_m = distance_found_m
        end
        return false, nil, nil, obstacle_distance_m
    end

    -- Sweep for the heading that best clears the obstacles between here and target_loc.
    -- Returns the updated best_distance_m and best_bearing_deg; obstacle_avoiding is an
    -- upvalue and is updated in place as closer obstacles are found.
    local function sweep_for_heading(bearing_deg, distance_to_target_m, target_loc,
                                     best_distance_m, best_bearing_deg)
        -- Under 20 m to the target there is nothing useful to sweep for.  Only the sweep
        -- declines: detect_aircraft() and detect_altitude_fence() are independent of it, and
        -- returning from self.detect() here - as this check used to - cleared aircraft_avoiding
        -- and then suppressed traffic alerts, NMAC, the trapped failsafe, the aircraft loiter
        -- and the altitude clamp for as long as the target stayed close.
        if distance_to_target_m < 20 then
            return best_distance_m, best_bearing_deg
        end
        -- Try increments around a circle, alternating left and right. The first heading
        -- that clears all obstacles for two look-ahead steps wins (a bounded downwind
        -- preference is applied afterwards, once we know we are avoiding).
        --
        -- The sweep is coarse-to-fine. A full-resolution sweep is 360/DAA_HEADING_INC
        -- candidates (241 at the 1.5 deg default) and it only exits early when a heading
        -- clears, so the boxed-in case - no heading clears at all - runs every candidate
        -- and each one costs an obstacle probe. That worst case can exceed SCR_VM_I_COUNT,
        -- which does not merely skip a cycle: the VM kills the script outright, mid
        -- avoidance, and it stays dead for the rest of the flight. Sweeping at
        -- COARSE_SWEEP_MULT x the increment and refining only around the winner keeps the
        -- final angular resolution while cutting the worst case by ~COARSE_SWEEP_MULT.
        local coarse_inc_deg  = bearing_inc_deg * COARSE_SWEEP_MULT
        local clear_delta_deg, obstacle_distance_m
        clear_delta_deg, obstacle_distance_m, best_distance_m, best_bearing_deg =
                coarse_sweep(bearing_deg, distance_to_target_m, target_loc, coarse_inc_deg,
                            best_distance_m, best_bearing_deg)

        -- Refine. The clear coarse heading sits one coarse step beyond the last blocked one,
        -- so the smallest deflection that actually clears lies inside that window. Walk the
        -- window at the full DAA_HEADING_INC resolution, nearest-to-target first, and take the
        -- first heading that still clears - which restores the "least deflection that works"
        -- result of the original fine sweep. Nothing to refine when the direct bearing was
        -- already clear, or when we are boxed in and no heading cleared: skipping the refine in
        -- the boxed-in case is exactly what keeps that (most expensive) case cheap.
        if clear_delta_deg ~= nil and clear_delta_deg ~= 0 then
            local sign     = (clear_delta_deg > 0) and 1 or -1
            local clear_mag = math.abs(clear_delta_deg)
            local prev_mag  = math.max(clear_mag - coarse_inc_deg, 0)   -- last blocked candidate this side
            for k = 1, COARSE_SWEEP_MULT do
                local test_mag = prev_mag + k * bearing_inc_deg
                if test_mag >= clear_mag then
                    break   -- reached the known-clear coarse heading; keep it
                end
                local cleared, distance_found_m, bearing_found_deg
                cleared, distance_found_m, bearing_found_deg, obstacle_distance_m =
                        probe_refine_candidate(bearing_deg, sign * test_mag, bearing_deg,
                                               distance_to_target_m, target_loc, obstacle_distance_m)
                if cleared then
                    -- a smaller deflection also clears, so prefer it (closer to the direct path)
                    best_distance_m  = distance_found_m
                    best_bearing_deg = bearing_found_deg
                    break
                end
            end
        elseif obstacle_avoiding ~= nil then
            -- Boxed in: no coarse heading cleared. A gap narrower than the coarse step can be
            -- stepped straight over, so probe outwards from the most open coarse candidate at
            -- the full resolution before accepting that we are trapped. Bounded at
            -- 2 * (COARSE_SWEEP_MULT - 1) extra probes, and it also restores full-resolution
            -- steering for the boxed-in case itself, which the coarse pass alone would leave
            -- on the coarse grid.
            local centre_deg = best_bearing_deg
            for j = 1, 2 * (COARSE_SWEEP_MULT - 1) do
                -- alternate either side of the most open heading: +1, -1, +2, -2, ... steps
                local step_n = math.floor((j + 1) / 2)
                local sign   = (j % 2 == 1) and 1 or -1
                local cleared, distance_found_m, bearing_found_deg
                cleared, distance_found_m, bearing_found_deg, obstacle_distance_m =
                        probe_refine_candidate(centre_deg, sign * step_n * bearing_inc_deg, bearing_deg,
                                               distance_to_target_m, target_loc, obstacle_distance_m)
                if cleared then
                    -- there was a gap after all; steer for it (still avoiding, so
                    -- obstacle_avoiding stays set, exactly as the full sweep would leave it)
                    best_distance_m  = distance_found_m
                    best_bearing_deg = bearing_found_deg
                    break
                end
            end
        end
        return best_distance_m, best_bearing_deg
    end

    -- detect flying objects or fences when flying towards navigation_target_loc
    local function detect_impl()
        -- TODO be smarter about re-populating this
        obstacle_avoiding = nil
        aircraft_avoiding = nil

        -- we want to calculate avoidance towards the current NAVIGATION TARGET (navigation_target_loc) - coping to target_loc to avoid changing the copy/pasted code
        if navigation_target_loc == nil or current_loc == nil then
            gcs:send_text(MAV_SEVERITY.ERROR, " AVOIDING: NO TARGET ")
            return
        end
        local target_loc = navigation_target_loc:copy()

        local bearing_deg       = math.deg(current_loc:get_bearing(target_loc))
        local best_bearing_deg  = bearing_deg
        local best_distance_m   = -FLT_MAX

        local distance_to_target_m = limit_distance(current_loc, target_loc, bearing_deg)
        best_distance_m, best_bearing_deg =
                sweep_for_heading(bearing_deg, distance_to_target_m, target_loc,
                                  best_distance_m, best_bearing_deg)

        -- we need to independently detect aircraft because even if an aircraft may not be the closest obstacle found by bendy ruler, we may still need to deal with it
        -- in other words, sometimes aircraft have higher priority than any other obstacles
        detect_aircraft()

        -- proactively check the altitude fences (vertical clamp-and-continue)
        local alt_obstacle = detect_altitude_fence()

        if obstacle_avoiding == nil then
            last_avoid_bearing_deg = nil
            committed_side_sign = 0
            side_flip_pending = false
            if alt_obstacle ~= nil then
                -- no horizontal threat, but we are approaching an altitude fence: keep heading to the
                -- waypoint and let update_target_location() clamp the commanded altitude into the safe band
                obstacle_avoiding = alt_obstacle
                local alt_target_loc = navigation_target_loc:copy()
                clamp_alt_to_fence(alt_target_loc)
                return alt_target_loc
            end
            return nil -- no avoidance required
        end

        if (now_ms - now_debug_ms) > 2000 then
            now_debug_ms = now_ms
        end

        local obstacle_type = obstacle_avoiding.type
        local is_fence = obstacle_type ~= nil and (
            (obstacle_type >= OBSTACLE_TYPE.FENCE_HOME and obstacle_type <= OBSTACLE_TYPE.FENCE_LUA)
            or obstacle_type == OBSTACLE_TYPE.FENCE_ALT_MAX
            or obstacle_type == OBSTACLE_TYPE.FENCE_ALT_MIN)

        if is_fence then
            -- Fences are fixed and containment is safety-critical: a heading slew limit or a
            -- committed side could delay/deflect the turn at a hard boundary and breach it.
            -- Keep the responsive bendy-ruler behaviour (clearance hysteresis only).
            -- best_distance_m is reassigned here too, to the clearance of whichever bearing
            -- comes back - not the discarded candidate's - so DAAD.DstB reflects what is
            -- actually flown, including on the "stay the course" path.
            best_bearing_deg, best_distance_m = resist_bearing_change(
                last_avoid_bearing_deg, best_bearing_deg, best_distance_m, target_loc)
            last_avoid_bearing_deg  = best_bearing_deg
            committed_side_sign     = 0
            side_flip_pending       = false
        else
            -- Non-fixed obstacles (aircraft, drones, birds, AIS, ...): velocity-aware smoothing.
            -- First decide whether the obstacle is actually a conflict: one that is opening range
            -- and whose predicted closest approach stays beyond well-clear is leaving, resume nav.
            local motion = assess_obstacle_motion(obstacle_avoiding)
            if not motion.is_conflict then
                -- the obstacle is leaving (opening range, predicted miss beyond its keep-out
                -- radius): drop it so avoid_obstacle() does not steer or announce for it. Any
                -- avoidance already in progress reverts cleanly (avoid_obstacle(nil)). This is
                -- re-decided every cycle from current geometry (no hold) so a manoeuvring obstacle
                -- is always tracked on fresh data; near a marginal crossing that can cost a few
                -- extra (slew-limited) heading reversals, which is the safe trade.
                obstacle_avoiding       = nil
                last_avoid_bearing_deg  = nil
                committed_side_sign     = 0
                side_flip_pending       = false
                return nil
            end
            -- Otherwise commit a side and slew-limit the heading so we track a smooth path
            -- instead of wiggling as the obstacle (and the instantaneous geometry) moves.
            -- refine_avoidance_bearing() also logs the DAAS smoothing trace each cycle.
            best_bearing_deg, best_distance_m = refine_avoidance_bearing(
                bearing_deg, best_bearing_deg, best_distance_m, motion, obstacle_avoiding, target_loc)
            last_avoid_bearing_deg  = best_bearing_deg
        end

        -- Where to put the commanded target along the bearing we picked - DAA_PLAN_M, which
        -- until 4.8.0-080 was DAA_LKAHD and so could not be set independently of how far the
        -- sweep probed.  Do not shorten it casually.  This location REPLACES next_WP_loc, and
        -- ArduPlane draws its past-the-waypoint finish line THROUGH next_WP_loc: a distant
        -- target puts that line out of reach, while a near one puts it alongside the aircraft,
        -- and the moment the avoidance bearing has any component back towards the previous
        -- waypoint the mission completes and moves on.  Shortening it to
        -- max(WP_LOITER_RAD, 2 x WP_RADIUS) was tried and measurably worse -
        -- PlaneDAAHungTrapFires skipped its waypoint at 106 m and finished 7 m off the fence
        -- instead of clearing it.
        local proj_distance = math.max(distance_to_target_m, plan_m)
        local new_target_loc = location_project(current_loc, best_bearing_deg, proj_distance, target_loc)
        log_detect_result(true, obstacle_avoiding.distance_m, best_distance_m, distance_to_target_m,
                          best_bearing_deg, new_target_loc, obstacle_avoiding.type)
        return new_target_loc
    end
    -- Multiple return values, not a report table: this is what lets mechanism and policy
    -- live in separate files, since detect_impl() still returns just the suggested target
    -- and the two contacts it settled on are handed back alongside it rather than left in
    -- shared state for the applet to read - but only actively avoiding builds this at all,
    -- so a table here would be one more allocation on the already-heavier active cycles.
    function self.detect()
        local target_loc = detect_impl()
        return target_loc, obstacle_avoiding, aircraft_avoiding
    end

    self.configure          = configure
    self.update_state       = update_state
    self.clamp_alt_to_fence = clamp_alt_to_fence
    -- the aircraft-loiter policy asks whether a contact is actually converging before it
    -- commits to a loiter, so the CPA assessment is part of the mechanism's public face
    self.assess_obstacle_motion = assess_obstacle_motion

    return self
end

return DAAcore
