--[[
    Geometry and airframe-capability helpers for planedaa.lua.

    Pure mechanism: nothing here decides what to DO about an obstacle, it only answers
    questions about angles, locations, and what turn the aircraft is actually capable of.
    Kept out of the applet so the applet stays the one file an integrator edits to change
    avoidance policy - see planedaa.md.

    DAAgeometry.new() returns an instance whose methods close over its own state, so each
    instance is independent and every call stays an upvalue call.  Reading the roll limit
    through the instance table instead would cost a table index on every probe, and the
    candidate-heading sweep calls turn_radius_m() and max_turn_rate_dps() well over a
    hundred times per cycle.
--]]

local DAAgeometry = {}

DAAgeometry.SCRIPT_VERSION = "4.8.0-001"
DAAgeometry.SCRIPT_NAME = "DAA geometry"
DAAgeometry.SCRIPT_NAME_SHORT = "DAAgeo"

-- Load-banner severity only - this module does no other logging, so no full severity
-- table is needed; MAV_SEVERITY.INFO is a fixed MAVLink wire value (6).
local BANNER_SEVERITY = 6

local GRAVITY_MSS = 9.80665

function DAAgeometry.new()
    local self = {}

    -- pushed in by configure(); a local rather than a field so the hot paths below reach
    -- it as an upvalue
    local roll_limit_deg = 0.0

    -- Push the cached parameter values in.  Called from the applet's parameter refresh.
    local function configure(settings)
        roll_limit_deg = settings.roll_limit_deg or 0.0
    end

    local function wrap_360(angle)
        local res = math.fmod(angle, 360.0)
         if res < 0 then
             res = res + 360.0
         end
         return res
    end
    
    local function wrap_180(angle)
        local res = wrap_360(angle)
        if res > 180 then
           res = res - 360
        end
        return res
    end
    
    --[[
        return true if two locations are identical
    --]]
    local function locations_equal(loc1, loc2)
        if loc1 == nil and loc2 == nil then
            return true
        end
        if (loc1 == nil and loc2 ~= nil) or (loc1 ~= nil and loc2 == nil) then
            return false
        end
        return (loc1:lat() == loc2:lat()) and (loc1:lng() == loc2:lng())
                and (loc1:alt() == loc2:alt())
                and (loc1:get_alt_frame() == loc2:get_alt_frame())
    end
    
    -- copy the altitude (value and frame) from src into dest. A small Lua helper on top of
    -- the existing get_alt_m/set_alt_m bindings, so we don't carry a Location:copy_alt_from()
    -- binding just for this (no C++ flash cost). Reading in src's own frame needs no conversion.
    local function copy_alt_from(dest, src)
        local frame = src:get_alt_frame()
        -- get_alt_m returns the altitude in `frame` (or nil if it can't convert, e.g. no terrain).
        -- Reading in src's own frame needs no conversion, so this normally succeeds.
        local alt_m = src:get_alt_m(frame)
        if alt_m ~= nil then
            dest:set_alt_m(alt_m, frame)
        end
    end
    
    -- Project forward from loc1 to a newlocation in the direction bearing_deg and distance m
    -- the altitude of the new projected location should be based on alt_target_loc, including frame
    local function location_project(loc1, bearing_deg, distance, alt_target_loc)
        -- Create a copy of the location projected distance meters in bearing_deg direction
        -- the projection should be in the frame project_in_frame
        local loc2 = loc1:copy()
        loc2:offset_bearing(bearing_deg, distance)
        copy_alt_from(loc2, alt_target_loc)
    
        return loc2
    end
    
    -- Maximum achievable rate of turn (deg/s) in a level banked turn at the configured
    -- roll limit: omega = g * tan(phi) / V.  Both the startup sanity checks and the
    -- course-change projection call this, so the two can never disagree.  Returns 0 when
    -- there is no usable speed, and the caller decides what that means.
    local function max_turn_rate_dps(speed_ms)
        if speed_ms == nil or speed_ms < 1.0 or roll_limit_deg <= 0 then
            return 0.0
        end
        return math.deg(GRAVITY_MSS * math.tan(math.rad(roll_limit_deg)) / speed_ms)
    end
    
    -- Radius of the tightest level turn available at the configured roll limit:
    -- R = V^2 / (g * tan(phi)).  This is what the aircraft can actually fly, and so what
    -- decides whether it can turn away from something in time.  WP_LOITER_RAD is a commanded
    -- loiter setting, not a capability: ArduPlane's own documentation notes that the achieved
    -- loiter radius is determined by ROLL_LIMIT_DEG when WP_LOITER_RAD is small.  Returns 0
    -- when there is no usable speed, and the caller decides what that means.
    local function turn_radius_m(speed_ms)
        if speed_ms == nil or speed_ms < 1.0 or roll_limit_deg <= 0 then
            return 0.0
        end
        return (speed_ms * speed_ms) / (GRAVITY_MSS * math.tan(math.rad(roll_limit_deg)))
    end
    
    --[[
        calculate what our ground speed would be in a given direction, using wind estimate
    --]]
    local function effective_groundspeed(airspeed, bearing_deg, wind_dir_rad, wind_speed)
        -- Ensure airspeed is at least 1.0
        airspeed = math.max(airspeed, 1.0)
        -- Convert bearing to radians
        local bearing_rad = math.rad(bearing_deg)
        -- Calculate the angle between wind direction and bearing
        local temp = math.pi - (wind_dir_rad - bearing_rad)
        local dangle = wind_speed * math.sin(temp) / airspeed
        -- If dangle is out of valid range, return 0
        if dangle > 1.0 or dangle < -1.0 then
            return 0
        end
        -- Calculate the angle alpha using arcsine
        local alpha = math.asin(dangle)
        -- Calculate yaw
        local yaw = bearing_rad - alpha
        -- Calculate beta, the angle between wind direction and yaw
        local beta = math.pi - (wind_dir_rad - yaw)
        -- Calculate ground speed squared (gs2)
        local gs2 = airspeed^2 + wind_speed^2 - 2 * airspeed * wind_speed * math.cos(beta)
        -- If gs2 is negative or zero, return 0
        if gs2 <= 0 then
            return 0
        end
        -- Calculate the final ground speed
        local gs = math.sqrt(gs2)
        return gs
    end

    self.configure              = configure
    self.wrap_360               = wrap_360
    self.wrap_180               = wrap_180
    self.locations_equal        = locations_equal
    self.location_project       = location_project
    self.max_turn_rate_dps      = max_turn_rate_dps
    self.turn_radius_m          = turn_radius_m
    self.effective_groundspeed  = effective_groundspeed

    return self
end

gcs:send_text(BANNER_SEVERITY, string.format("%s %s module loaded", DAAgeometry.SCRIPT_NAME, DAAgeometry.SCRIPT_VERSION))

return DAAgeometry
