--[[

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

 Detect, Alert, Avoid for Plane

 This code implements a DAA function for Plane. 
 Detect - relies on MAVLink input from GLOBAL_POSITION_INT, FOLLOW_TARGET or ADSB_VEHICLE
            to provide the location of other aircraft in the vecility
        - This other input can come from a variety of sources including an onboard
            companion computer using vision based AI to generate ADSB_VEHICLE messages
            for detected obstacles in teh vicinity. 
        - Also uses ArduPilot GeoFences which can be inclusive or exclusive and can used
            to describe no fly zones such as restricted airspace, or towers, buildings or
            other structures that might not show up in terrain.
Alert - displays GCS text messages which describe threatening vehicles or obstacles
            also sends GCS_THREAT messages which can be displayed by a suitably enabled
            GCS such as Mission Planner or MavProxy
Avoid - implements bendy ruler based heuristic avoidance for most obstacles
            for GA (General Aviation) or Crude Aircraft also implements more
            flexible avoidance manoeuvre such as a Standard Right Turn to Altitude.
            The intention is to come up with a standard library of GA avoidance 
            manoeuvres, will also allowing users to implement there own due to this 
            being implemented as Lua.            
--]]

SCRIPT_NAME = "Plane DAA"
SCRIPT_NAME_SHORT = "PlaneDAA"
SCRIPT_VERSION = "4.7.0-001"

STARTUP_DELAY = 25  -- wait this many seconds for the FC to come up before starting the script

FLIGHT_MODE = {AUTO=10, RTL=11, LOITER=12, GUIDED=15, QHOVER=18, QLOITER=19, QRTL=21}
ALT_FRAME = { GLOBAL = 0, RELATIVE = 1, TERRAIN = 3}


MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
MAV_CMD_INT = { DO_SET_MODE = 176, DO_CHANGE_SPEED = 178, DO_REPOSITION = 192, 
                  GUIDED_CHANGE_SPEED = 43000, GUIDED_CHANGE_ALTITUDE = 43001, GUIDED_CHANGE_HEADING = 43002 }
MAV_SPEED_TYPE = { AIRSPEED = 0, GROUNDSPEED = 1, CLIMB_SPEED = 2, DESCENT_SPEED = 3 }
MAV_HEADING_TYPE = { COG = 0, HEADING = 1} -- COG = Course over Ground, i.e. where you want to go, HEADING = which way the vehicle points 

-- MAV_COLLISION_THREAT_LEVEL
MAV_COLLISION_THREAT_LEVEL = {
    NONE = 0,         -- Not a threat
    LOW = 1,          -- Mild concern about this threat
    HIGH = 2,         -- Craft is panicking and may take action to avoid
    ENUM_END = 3      -- End of enum
}
-- MAV_COLLISION_SRC
MAV_COLLISION_SRC = {
    ADSB = 0,                         -- Source is ADSB_VEHICLE packets
    MAVLINK_GPS_GLOBAL_INT = 1,       -- Source is MAVLink GPS_GLOBAL_INT
    ENUM_END = 2                      -- End of enum
}

MAV_COLLISION_ACTION = {
    NONE                        = 0, -- Ignore any potential collisions 
    REPORT                      = 1, -- Report potential collision 
    ASCEND_OR_DESCEND           = 2, -- Ascend or Descend to avoid threat 
    MOVE_HORIZONTALLY           = 3, -- Move horizontally to avoid threat 
    MOVE_PERPENDICULAR          = 4, -- Aircraft to move perpendicular to the collision's velocity vector 
    RTL                         = 5, -- Aircraft to fly directly back to its launch point 
    HOVER                       = 6, -- Aircraft to stop in place 
    LOITERTURN                  = 7, -- Aircraft to do a loiter turn left or right to lose altitude
}

OBSTACLE_TYPE = {
    GENERAL                     = 0,    -- generic obstacle, we don't really know what it is
    MAV_SYSID                   = 1,    -- another MAVLINK drone with a MAV_SYSID
    GENERAL_AVIATION            = 2,    -- crude aircraft, usually with an ICAO ADSB identifier
    WEATHER                     = 3,    
    BIRD_MIGRATORY              = 4,    -- typically one or more Canada Geese
    BIRD_OF_PREY                = 5,    -- a bird that might attack the vehicle
    FENCE_HOME                  = 6,    -- all fixed/unmovable fences
    FENCE_CIRCLE_INCLUSION      = 7,
    FENCE_CIRCLE_EXCLUSION      = 8,
    FENCE_POLYGON_INCLUSION     = 9,
    FENCE_POLYGON_EXCLUSION     = 10,
    FENCE_LUA                   = 11,
    PROXIMITY                   = 12,   -- detected by a proximty sensor, typically quite close
    AIS                         = 13,   -- Automatic Identification System for ship (maritime) vehicles
}

local DAA_active = true;

local PARAM_TABLE_KEY = 106
local PARAM_TABLE_PREFIX = "DAA_"

-- bind a parameter to a variable
function bind_param(name)
    return Parameter(name)
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), SCRIPT_NAME_SHORT .. string.format(' could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup follow mode specific parameters need 2wo tables because there are > 10 parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 20), SCRIPT_NAME_SHORT .. ' could not add param table: ' .. PARAM_TABLE_PREFIX)

--[[
    // @Param: DAA_ACT_FN
    // @DisplayName: DAA Activation
    // @Description: Disable the DAA capability or turn it off (defaults to on)
    // @Range: 300 307
--]]
DAA_ACT_FN = bind_add_param("ACT_FN", 1, 306)

--[[
  // @Param: DAA_MARGIN_FENCE
  // @DisplayName: fence margin 
  // @Description: Avoidance margin for fence
  // @Units: m
--]]
DAA_MARGIN_FENCE = bind_add_param('MARGIN_FENCE', 2, 50)

--[[
  // @Param: DAA_MARGIN_DYN
  // @DisplayName: dynamic margin
  // @Description: Avoidance margin for dynamic objects
  // @Units: m
--]]
DAA_MARGIN_DYN   = bind_add_param('MARGIN_DYN', 3, 20)

--[[
  // @Param: DAA_MARGIN_EXCL
  // @DisplayName: exclusion zone margin
  // @Description: Avoidance margin for exclusion zones
  // @Units: m
--]]
DAA_MARGIN_EXCL   = bind_add_param('MARGIN_EXCL', 4, 20)

--[[
  // @Param: DAA_MARGIN_WIDE
  // @DisplayName: wide avoidance margin
  // @Description: Avoidance margin for wide avoidance
  // @Units: m
--]]
DAA_MARGIN_WIDE   = bind_add_param('MARGIN_WIDE', 5, 30)

--[[
  // @Param: BR_MARGIN_HGT 
  // @DisplayName: height avoidance margin
  // @Description: Avoidance margin for height avoidance 
  // @Units: m
--]]
DAA_MARGIN_HGT   = bind_add_param('MARGIN_HGT', 6, 60)

--[[
  // @Param: DAA_LKAHD
  // @DisplayName: avoidance lookahead distance
  // @Description: Avoidance lookahead distance
  // @Units: m
--]]
DAA_LKAHD  = bind_add_param('LKAHD', 7, 500)

--[[
  // @Param: DAA_UPDATE_RATE
  // @DisplayName: rate to process avoidance
  // @Description: Avoidance processing rate
  // @Units: hz
--]]
DAA_UPDATE_RATE  = bind_add_param('UPDATE_RATE', 8, 10.0)

--[[
  // @Param: DAA_HEIGHT_USE
  // @DisplayName: Include height differences
  // @Description: Avoidance will consider height differences when calculating collisions
  // @Values: 0:Use Height,1:Ignore Height
--]]
DAA_HEIGHT_USE  = bind_add_param('HEIGHT_USE', 9, 0)

--[[
  // @Param: DAA_MARGIN_GA
  // @DisplayName: Margin for General Aviation
  // @Description: Avoidance margin for Fixed Wing aircraft/General Aviation (Helicopters? eVTOL?)
  // @Units: m
--]]
DAA_MARGIN_GA  = bind_add_param('MARGIN_GA', 10, 300)

--[[
  // @Param: DAA_MARGIN_WTHR
  // @DisplayName: Radius for Weather
  // @Description: Avoidance radius for Weather/Clouds/Rain 
  // @Units: m
--]]
DAA_MARGIN_WTHR  = bind_add_param('MARGIN_WTH', 11, 173)

--[[
  // @Param: DAA_MARGIN_BIRD
  // @DisplayName: Margin for Birds
  // @Description: Avoidance margin for Migratory Birds 
  // @Units: m
--]]
DAA_MARGIN_BIRD  = bind_add_param('MARGIN_BIRD', 12, 100)

--[[
  // @Param: DAA_MARGIN_PREY
  // @DisplayName: Radius for Birds of Prey
  // @Description: Avoidance radius for Birds of Prey
  // @Units: m
--]]
DAA_MARGIN_PREY  = bind_add_param('MARGIN_PREY', 13, 200)

--[[
  // @Param: DAA_MARGIN_UAV
  // @DisplayName: Margin for UAVs/Drones
  // @Description: Avoidance radius for UAV/drone (MAVLink sourced)
  // @Units: m
--]]
DAA_MARGIN_UAV  = bind_add_param('MARGIN_UAV', 14, 50)

--[[
// @Param: BR_RATIO
    // @DisplayName: DAA margin ratio for BendyRuler to change bearing significantly 
    // @Description:  DAA BendyRuler will avoid changing bearing unless ratio of previous margin from obstacle (or fence) to present calculated margin is at least this much.
    // @Range: 1.1 2
    // @Increment: 0.1
    // @User: Standard
--]]
DAA_BR_RATIO = bind_add_param('BR_RATIO', 15, 1.5)

--[[
    // @Param: BR_ANGLE
    // @DisplayName: BendyRuler's bearing change resistance threshold angle   
    // @Description:  DAA BendyRuler will resist changing current bearing if the change in bearing is over this angle
    // @Range: 20 180
    // @Increment: 5
    // @User: Standard
--]]
DAA_BR_ANGLE = bind_add_param('BR_ANGLE', 16, 75)

WARN_DIST_XY  = bind_param("AVD_W_DIST_XY")
WARN_ACTION  = bind_param("AVD_W_ACTION")
AVD_ENABLE  = bind_param("AVD_ENABLE")
ROLL_LIMIT_DEG = bind_param("ROLL_LIMIT_DEG")
WP_LOITER_RAD = bind_param("WP_LOITER_RAD")

--local warn_act = WARN_ACTION:get()
local roll_limit_deg = ROLL_LIMIT_DEG:get()
local lookahead_param = DAA_LKAHD:get()
local margin_fence = DAA_MARGIN_FENCE:get()
local refresh_rate = 1000.0 / DAA_UPDATE_RATE:get()
local bendy_ratio = DAA_BR_RATIO:get()
local bendy_angle = DAA_BR_ANGLE:get()
local wp_loiter_rad = WP_LOITER_RAD:get()

GRAVITY_MSS = 9.80665
LOCATION_SCALING_FACTOR_INV = 89.83204953368922

-- TODO should be a parameter (also convert to degrees)
local bearing_inc_cd = 1500
local bearing_inc_deg = 1.5

COLLISION_DETECTED = false

FLT_MAX = 3.402823466e+38

local now_ms            = millis()
local now_params_ms     = now_ms
local now_debug_ms      = now_ms
local now_avoiding_ms   = now_ms

-------------------------------------------------------------------------------
--- Utility methods
-------------------------------------------------------------------------------
---
---

-- keep local copies of parameter values that the user might change so update ever 5 seconds
local function refresh_parameters()
    if (now_ms - now_params_ms) > 5000 then
        --warn_act = WARN_ACTION:get()
        roll_limit_deg      = ROLL_LIMIT_DEG:get()
        lookahead_param     = DAA_LKAHD:get()
        margin_fence        = DAA_MARGIN_FENCE:get()
        refresh_rate        = 1000.0 / DAA_UPDATE_RATE:get()
        bendy_ratio         = DAA_BR_RATIO:get()
        bendy_angle         = DAA_BR_ANGLE:get()
        wp_loiter_rad       = WP_LOITER_RAD:get()

        now_params_ms       = now_ms
    end
end

-----Auxiliary functions
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

local function length_squared(w)
    return (w:x() * w:x()) + (w:y() * w:y())
end

local function dot_product_2vector(p,w)
    return (w:x() * p:x()) + (w:y() * p:y())
end

local function closest_point(p, w)
    local l2 = length_squared(w)
    if l2 < 1e-6 then
        return w  -- case v == w
    end
    local t = dot_product_2vector(p,w) / l2
    if t <= 0 then
        local Vector2=Vector2f()
        Vector2:x(0)
        Vector2:y(0)
        return Vector2
    elseif t >= 1 then
        return w
    else
        w:x(w:x()*t)
        w:y(w:y()*t)
        return w
    end
end

local function longitude_scale(lat)
    local DEG_TO_RAD = math.pi / 180
    local scale = math.cos(lat * (1.0e-7 * DEG_TO_RAD))
    return math.max(scale, 0.01)
end

local function limit_latitude(lat)
    if lat > 900000000 then
        lat = 1800000000 - lat
    elseif lat < -900000000 then
        lat = -(1800000000 + lat)
    end
    return lat
end

local function wrap_longitude(lon)
    if lon > 1800000000 then
        lon = lon - 3600000000
    elseif lon < -1800000000 then
        lon = lon + 3600000000
    end
    return lon
end

 -- Extrapolate latitude/longitude given bearing and distance
local function offset_bearing(lat, lng, bearing_deg, distance)
    --local radians = math.rad
    local ofs_north = math.cos(math.rad(bearing_deg)) * distance
    local ofs_east  = math.sin(math.rad(bearing_deg)) * distance
    local dlat = ofs_north * LOCATION_SCALING_FACTOR_INV
    local dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(lat + dlat / 2.0)
    lat = lat + dlat
    lat = limit_latitude(lat)
    lng = wrap_longitude(dlng + lng)
    return lat, lng
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

-- Project forward from loc1 to a newlocation in the direction bearing_deg and distance m
-- the altitude of the new projected location should be based on alt_target_loc, including frame
local function location_project(loc1, bearing_deg, distance, alt_target_loc)
    -- Create a copy of the location projected distance meters in bearing_deg direction
    -- the projection should be in the frame project_in_frame
    --local loc2 = Location()
    --loc2=loc1
    --loc2:offset_bearing(bearing_deg, distance)
    -- local loc2 = alt_target_loc:copy()
    -- can now do this with Location
    --local lat, lon = offset_bearing(loc1:lat(), loc1:lng(), bearing_deg, distance)
    --loc2:lat(lat)
    --loc2:lng(lon)

    local loc2 = loc1:copy()
    loc2:offset_bearing(bearing_deg, distance)
    loc2:copy_alt_from(alt_target_loc)

--    void offset_bearing(ftype bearing_deg, ftype distance);

    -- use the altitude/frame copied from loc1
    --loc2:alt(loc1:alt())
    --gcs:send_text(0, string.format("IN: Lat: %.2f, Lon: %.2f", loc1:lat(),loc1:lng()))
    --gcs:send_text(0, string.format("Dist: %.2f, Bear: %.2f", distance,bearing_deg)) 
    --gcs:send_text(0, string.format("Out: Lat: %.2f, Lon: %.2f", loc2:lat(),loc2:lng()))  
    --gcs:send_text(0, string.format("Frame: IN: %d, OUT: %d", loc1:get_alt_frame(),loc2:get_alt_frame()))
    return loc2
end

local function calc_avoidance_margin(loc1, loc2, lookahead_m)
    -- By projecting 1m along the line we avoid a problem with the
    -- exclusion avoidance being happy to skirt along a line parallel
    -- to an exclusion zone
    local bearing_deg = math.deg(loc1:get_bearing(loc2))
    local loc1_shifted = location_project(loc1, bearing_deg, 1, loc2)
    -- local obs_margin, obstacle = obstacle_avoidance_margin(loc1_shifted, our_velocity, avoid_sec)

    local distance_min_m
    local obstacle = {}
    local loc3

    if OAScripting == nil then
    	gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " OAScripting object is nil!")
        return FLT_MAX, nil -- Exit the script or handle the error
    end

    --distance_min_m = OAScripting:distance_obstacle_test2(lookahead_m)

    --distance_min_m, loc3 = OAScripting:distance_obstacle_test(loc1_shifted, loc2, lookahead_m)

    --[[local stashed = OAScripting:find_closest_obstacle_stash(loc1_shifted, loc2)
    distance_min_m,
        obstacle.type,
        obstacle.label,
        obstacle.sysid,
        obstacle.location,
        obstacle.post_NED_m,
        obstacle.velocity = OAScripting:find_closest_obstacle()
    
    --distance_min_m = OAScripting:distance_obstacle_test(lookahead_m)
--]]    
    distance_min_m,
    obstacle.type,
    obstacle.label,
    obstacle.sysid,
    obstacle.location,
    obstacle.post_NED_m,
    obstacle.velocity = OAScripting:find_closest_obstacle(loc1_shifted, loc2, lookahead_m)

    if distance_min_m ~= nil then
        obstacle.distance_m = distance_min_m
        --src = obstacle.sysid * 1.0
        --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. "dist: " .. distance_min_m .. " src: " .. obstacle.label)
    	--gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(" obs dist: %.1f m src: %.0f %s", distance_min_m, obstacle.sysid, obstacle.label))
    else
        distance_min_m = FLT_MAX
        obstacle = nil
    end

--[[
           bool distance_to_obstacle(const Location &start_loc, const Location &end_loc, const float lookahead_m, 
                                                // Return values
                                                float &distance_min_m, 
                                                uint16_t &type, 
                                                char *&label, 
                                                uint32_t &sys_id,
                                                Location &location, 
                                                Vector3f &pos_NED_m,
                                                Vector3f &velocity_NED_ms
--]]
        -- this is the old code
        --obstacle.timestamp_ms=avoid:get_obstacle_timeout(i)
        --obstacle.velocity=avoid:get_obstacle_vel(i)
        --obstacle.location=avoid:get_obstacle_loc(i)
        --obstacle.src_id=avoid:get_obstacle_id(i)  

        
    return distance_min_m, obstacle
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
    --gcs:send_text(0, string.format("as:%.1f bear:%.1f wind_dir:%.1f ws:%.1f -> gs:%.1f", airspeed, bearing_deg, math.deg(wind_dir_rad), wind_speed, gs)) 
    return gs
end

-------------------------------------------------------------------------------
-- LOITER ALTITUDE - Loiter right or left to (usually) lose altitude to avoid an obstacle (usually a crude aircraft)
-------------------------------------------------------------------------------
local loiteralt = {
    active = false,
    target_alt_amsl_m = nil
}
(function ()
    local pre_loiteralt_heading_deg = -1.0
    local current_location = ahrs:get_position()
    local current_heading_deg = math.deg(ahrs:get_yaw_rad())

    -- 
    function loiteralt.start(target_alt_m, direction_right, groundspeed_ms)
        local direction = ""
        if loiteralt.active then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": loiteralt ALREADY ACTIVE: " .. loiteralt.target_alt_amsl_m)
            return nil
        end
        if current_location == nil then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": loiteralt no current_location")
            return nil
        end
        pre_loiteralt_heading_deg = math.deg(ahrs:get_yaw_rad())
        --    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt start hdg pre %.0f now %.0f dif %0.1f",
        --                    pre_loiteralt_heading_deg, current_heading_deg
        --                    , math.abs(pre_loiteralt_heading_deg - current_heading_deg)) )

        loiteralt.target_alt_agl_m = target_alt_m
        local radius_m = (60.0 * groundspeed_ms) / math.pi
        local new_target_loc = current_location:copy()
        new_target_loc:set_alt_m(target_alt_m, ALT_FRAME.TERRAIN)
        if direction_right then
            new_target_loc:offset_bearing(wrap_360(current_heading_deg + 90), radius_m)
            direction = "right"
        else
            new_target_loc:offset_bearing(wrap_360(current_heading_deg - 90), radius_m)
            direction = "left"
        end
        --local new_target_loc, direction = start_loiter(loiteralt.target_alt_agl_m, direction_right, groundspeed_ms)
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiter %s to %.0f alt",
                direction,
                loiteralt.target_alt_agl_m ))
        loiteralt.active = true

        return new_target_loc
    end

    function loiteralt.stop()
        if current_location ~= nil then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt Done alt: %.0f", current_altitude_m) )
        end
        loiteralt.active = false
        loiteralt.target_alt_amsl_m = nil
    end

    -- should be called regularly if loiteralt is active
    function loiteralt.update()
        current_location = ahrs:get_position()
        current_heading_deg = math.deg(ahrs:get_yaw_rad())

        if current_location == nil then
            return
        end
        local current_location_amsl = current_location:copy()
        current_location_agl:change_alt_frame(mavlink.ALT_FRAME.ABOVE_TERRAIN)
        -- if we've reached altitude and are pointing approximately where we were before we started loiteralt
        if now - now_debug > 2 and false then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt check hdg pre %.0f now %.0f dif %0.1f",
                        pre_loiteralt_heading_deg, current_heading_deg,
                        math.abs(pre_loiteralt_heading_deg - current_heading_deg)) )
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt check alt curr %.0f trg %.0f max %0.1f",
                        current_altitude_m, loiteralt.target_alt_amsl_m,
                        altitude_max) )
            now_debug = now
        end
        -- if we have achieved the target altitude exit immediately and we are pointing to the next WP
        local current_agl_m = current_location_agl:alt() * 0.01

        if (math.abs(current_agl_m - loiteralt.target_alt_agl_m)) < 3 and 
                (math.abs(pre_loiteralt_heading_deg - current_heading_deg) < 45.0) then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt STOP alt curr %.0f trg %.0f max %0.1f",
                        current_altitude_m, current_amsl_m, altitude_max) )
            loiteralt.stop()
        else
            --airspeed_desired = airspeed_cruise
            --mavlink.set_vehicle_speed({speed=airspeed_desired})
        end
    end
end)()

-------------------------------------------------------------------------------
--- DAA management class
-------------------------------------------------------------------------------
DAA = {
   enabled = false,
}
(function ()
    local active = true;
    local navigating = false;
    local current_loc = ahrs:get_position()
    --local current_target_loc = vehicle:get_target_location()
    local update_target_location_save_loc                           -- this is the saved current_target for use by update_target_location ONLY
    local navigation_target_loc                                     -- this is where the vehicle is trying to get to (i.e. next waypoint if no avoidance)
    local daa_target_loc                                            -- this is where the DAA is currently trying to in order to avoid obstacles (nil if not avoiding)
    local groundspeed_ms = ahrs:groundspeed_vector():length()
    local airspeed_ms = ahrs:airspeed_EAS() or groundspeed_ms
    local wind_dir_rad = 0.0
    local wind_speed = 0.0
    local ground_course_deg
    local obstacle_avoiding
    local previous_label = ""
    local STATE = {monitoring = 1, avoiding = 2, loitering =3}
    local current_state = STATE.monitoring
    local now_daa_params_ms = now_ms

    -- the distance we look ahead is adjusted dynamically based on avoidance results
    local current_lookahead = lookahead_param

    local function calculate_windspeed()
                -- Get wind estimate and convert to 2D
        local wind_3d = ahrs:wind_estimate()
        local wind_2d = Vector2f()
        wind_2d:x(wind_3d:x())
        wind_2d:y(wind_3d:y())

        return  wind_2d:length(), wind_2d:angle()
    end 

    function DAA.disable()
        DAA.enabled = false
    	gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " disabled")
    end 
    function DAA.enable()
        DAA.enabled = true
    	gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " enabled")
    end

    --return true if we are in a state where DAA can apply
    function DAA.active()
        return DAA.enabled and active and arming:is_armed()
    end

    -- populate some local values with a static/consistent picture of the vehicle state
    function DAA.get_vehicle_state()
        now_ms = millis()

        active = true;
        current_loc = ahrs:get_position()
        local current_target_loc = vehicle:get_target_location()

        if current_loc == nil or current_target_loc == nil then
            -- no position or not navigating
            navigation_target_loc = nil
            daa_target_loc = nil
            if navigating then
    	        gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. "NOT NAVIGATING")
                navigating = false
            end
            active = false
            return
        end
        update_target_location_save_loc = current_target_loc:copy()

        -- if the navigation target has changed to some other target not the DAA target, it must be vehicle navigation 
        if navigation_target_loc == nil or
            (not locations_equal(navigation_target_loc, current_target_loc) and
                not locations_equal(daa_target_loc, current_target_loc)) then
            -- the vehicle navigation code has changed it's target
            if navigation_target_loc == nil then
        	    --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. "Set Navigation Target NAV")
            elseif daa_target_loc == nil then
        	    --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. "Set Navigation Target DAA")
            elseif not locations_equal(daa_target_loc, current_target_loc) then
        	    --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. "Set Navigation Target diff")
            end
            navigation_target_loc = current_target_loc:copy()
        end
        groundspeed_ms = ahrs:groundspeed_vector():length()

        -- Calculate wind direction and speed
        wind_speed, wind_dir_rad = calculate_windspeed()

        ground_course_deg = wrap_180(math.deg(ahrs:groundspeed_vector():angle()))

    end

    local function calc_avoidance_distance(avoid_step1_m, target_distance)
        -- test for flying past the waypoint, so if we are close, we have room to dodge after the waypoint
        local avoid_max = math.min(avoid_step1_m, target_distance + math.min(margin_fence / 2, 100))
        local avoid_sec1 = avoid_max / airspeed_ms
        return airspeed_ms * avoid_sec1
    end

    --[[
    This function is called when BendyRuler has found a bearing which is obstacles free at at least lookahead_step1_dist and  then lookahead_step2_dist from the present location
    In many situations, this new bearing can be either left or right of the obstacle, and BendyRuler can have a tough time deciding between the two.
    It has the tendency to move the vehicle back and forth, if the margin obtained is even slightly better in the newer iteration.
    Therefore, this method attempts to avoid changing direction of the vehicle by more than _bendy_angle degrees, 
    unless the new margin is atleast _bendy_ratio times better than the margin with previously calculated bearing.
    We return true if we have resisted the change and will follow the last calculated bearing. 
    --]]
    local function resist_bearing_change(bearing_orig, avoid_step1_m, bearing_test, distance_found)

       if math.abs(wrap_180(bearing_orig - bearing_test)) > bendy_angle then
            -- check margin in last bearing's direction
            local test_loc_previous_bearing = current_loc:copy()
            test_loc_previous_bearing:offset_bearing(wrap_180(bearing_orig), avoid_step1_m)
            
            local distance_previous_m, _ = calc_avoidance_margin(current_loc, test_loc_previous_bearing, avoid_step1_m)

            if (distance_previous_m < (bendy_ratio * distance_found)) then
                -- don't change direction abruptly. If margin difference is not significant, follow the last direction

            	--gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. " RESIST: was " .. bearing_test .. " now " .. bearing_orig)
                bearing_test = bearing_orig
                distance_found  = distance_previous_m
            end
        end

        return distance_found, bearing_test
    end

    -- calculates the second step of the bendy ruler test - look foward a 2nd "full_distance" to see if we can still avoid obstacles
    local function test_step2(loc_test, target_bearing, avoid_step2_m, full_distance)
        local test_bearings = { 0, 45, -45 }
        local target_loc = loc_test:copy()
        target_loc:offset_bearing(target_bearing, full_distance)

        -- local obstacle_found = {}
        local bearing_found = target_bearing
        local margin_found = FLT_MAX

        for _, delta in ipairs(test_bearings) do
            local bearing_test = target_bearing + delta

            local target_distance = loc_test:get_distance(target_loc)
            local distance = calc_avoidance_distance(avoid_step2_m, target_distance)
            local loc_test2 = location_project(loc_test, bearing_test, distance, target_loc)

            local margin2, _  = calc_avoidance_margin(loc_test2, target_loc, current_lookahead)

            if margin2 > current_lookahead then
                -- Project the new target in the chosen direction by the full distance
                local new_loc = location_project(loc_test2, bearing_test, full_distance, target_loc)
                -- current_lookahead = math.min(lookahead_param, current_lookahead * 1.1)

                -- return immediately - no obstacles in this direction
                return bearing_test, margin2
            end
            if margin2 < margin_found then
                -- return the bearing to the nearest obstacle
                bearing_found = bearing_test
                margin_found = margin2
            end
        end

        return bearing_found, margin_found
    end

    -- This method calculates the projected location in the desired direction taking account of airspeed, windspeed and the time it takes to turn
    local function location_after_course_change(from_loc, course_deg, to_loc)
        local course_change_deg = wrap_180(course_deg - ground_course_deg)
        local ground_speed_ms = effective_groundspeed(airspeed_ms, course_deg, wind_dir_rad, wind_speed) -- ground speed based on the new bearing (accounting for wind)
        local rate_of_turn_dps = math.deg(GRAVITY_MSS * math.tan(math.rad(roll_limit_deg * 0.6)) / (airspeed_ms + 0.1))

        if math.abs(course_change_deg) > 170 then
            -- Skip 180-degree turns as we can't predict the turn direction
            return from_loc
        end
        -- Calculate how long it will take to change course
        local turn_time_s = math.abs(course_change_deg / rate_of_turn_dps)
        -- Approximate turn by flying forward for half of the turn time
        local projected_loc = location_project(from_loc, ground_course_deg, ground_speed_ms * turn_time_s * 0.5, to_loc)
        -- If turning more than 90 degrees, add sideways movement
        if math.abs(course_change_deg) > 90 then
            local direction = course_change_deg > 0 and (ground_course_deg + 90) or (ground_course_deg - 90)
            local proportion = math.sin(math.rad(math.abs(course_change_deg) - 90))
            projected_loc = location_project(projected_loc, direction, ground_speed_ms * proportion * turn_time_s * 0.5, to_loc)
        end

        return projected_loc
    end

    -- This method checks whether we will collide with any obstacle if we fly at a given bearing bearing_deg + i * bearing_inc_deg
    local function test_step1(full_distance, bearing_deg, i, target_loc)

        -- need to get rid of the legacy _cd
        -- local bearing_cd = bearing_deg * 100.0

        -- gcs:send_text(0, string.format("test bearing: %.1f deg", bearing_cd / 100.0))

        --local bearing_delta_cd = i * bearing_inc_cd / 2
        local bearing_delta_deg = i * bearing_inc_deg / 2.0

        --gcs:send_text(0, string.format("i: %d", i)) 
        if i % 2 == 1 then
            -- Alternate between left and right of the target
            --bearing_delta_cd = -bearing_delta_cd
            bearing_delta_deg = -bearing_delta_deg
        end

        local avoid_step1_m = current_lookahead
        local avoid_step2_m = current_lookahead * 2.0

        -- Test bearing used to look ahead for obstacles
        -- local bearing_test = wrap_180((bearing_cd*0.01) + (bearing_delta_cd*0.01))
        local bearing_test_deg = wrap_180(bearing_deg + bearing_delta_deg)
        local adjusted_loc = location_after_course_change(current_loc, bearing_test_deg, target_loc)
        -- fudge to ignore the course change
        adjusted_loc = current_loc

        -- Position after one step from where we think we will be after turning to bearing_test_deg
        local avoidance_distance_m = calc_avoidance_distance(avoid_step1_m, full_distance)
        local test_loc = location_project(adjusted_loc, bearing_test_deg, avoidance_distance_m, target_loc)

        local distance_found_m, obstacle_found = calc_avoidance_margin(adjusted_loc, test_loc, current_lookahead)
        ---@cast margin number
        if distance_found_m > current_lookahead then
            -- This direction avoids all obstacles for one step. Check if it leads to a clear path for a longer distance.
            local bearing2, margin2 = test_step2(test_loc, bearing_test_deg, avoid_step2_m, full_distance)
            if margin2 >= current_lookahead then
                -- Project the new target in the chosen direction by the full distance
                --local new_loc = location_project(projected_loc, bearing2, full_distance, target_loc)

                if i == 0 and bearing2 == bearing_test_deg then
                    -- means we have a direct unobstructed path for step1 (i == 0) and step2
            	    -- gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. "UNOBSTRUCTED  bearing: " .. bearing2)
                    return FLT_MAX, bearing_deg, nil -- no avoidance required
                end
                distance_found_m = margin2
            end
            distance_found_m, bearing_test_deg = resist_bearing_change(bearing_deg, avoid_step1_m, bearing_test_deg, distance_found_m)
        end

        return distance_found_m, bearing_test_deg, obstacle_found
    end

    -- if the plane is currently pointing far away from the target, then assume that we 
    -- will be turning sharply, so we don't look too far ahead for obstacles
    local function normalize_distance(from_loc, to_loc, bearing_deg)
        local distance_to_target_m = from_loc:get_distance(to_loc)

        if (math.abs(wrap_180(bearing_deg - ground_course_deg)) > bendy_angle * 2) then
            distance_to_target_m = wp_loiter_rad * 2
        end

        return distance_to_target_m
    end

    -- detect flying objects or fences when flying towards navigation_target_loc
    function DAA.detect()
        -- TODO be smarter about re-populating this
        local obstacle_distance_m = FLT_MAX
        obstacle_avoiding = nil

        -- we want to calculate avoidance towards the current NAVIGATION TARGET (navigation_target_loc) - coping to target_loc to avoid changing the copy/pasted code
        if navigation_target_loc == nil then
            gcs:send_text(MAV_SEVERITY.ERROR, " AVOIDING: NO TARGET ")
            return
        end
        local target_loc = navigation_target_loc:copy()

        --gcs:send_text(0, "got current lookahead")

        -- local bearing_cd = math.deg(current_loc:get_bearing(target_loc))*100
        local bearing_deg = math.deg(current_loc:get_bearing(target_loc))
        -- get the current ground course
        -- gcs:send_text(0, string.format("ground_course_deg : %.2f",ground_course_deg ))  

        --local distance_to_target_m = current_loc:get_distance(target_loc)
        local distance_to_target_m = normalize_distance(current_loc, target_loc, bearing_deg)
        -- If the full distance is less than 20m, no avoidance is needed
        if distance_to_target_m < 20 then
            return nil
        end
        --gcs:send_text(0, string.format("full_distance: %.1f m", distance_to_target_m))

        local best_bearing_deg = bearing_deg
        local best_distance_m = -FLT_MAX
        local worst_bearing_deg = bearing_deg
        local worst_distance_m = FLT_MAX

        -- Try 5 degree increments around a circle, alternating left and right. Check each one to see if flying in that direction would avoid all obstacles.
        for i = 0, (360 / bearing_inc_deg) do
            local distance_found_m, bearing_found_deg, obstacle_found = test_step1(distance_to_target_m, bearing_deg, i, target_loc)
            if distance_found_m > best_distance_m then
                best_distance_m = distance_found_m
                best_bearing_deg = bearing_found_deg
            end
            if obstacle_found == nil then -- found a path with no obstacles - done!
                goto continue
            end
            if distance_found_m < obstacle_distance_m then
                obstacle_avoiding = obstacle_found
                obstacle_distance_m = distance_found_m
            end
        end
        ::continue::

        if obstacle_avoiding == nil then
            -- gcs:send_text(MAV_SEVERITY.ERROR, "NO OBSTACLE")
            return nil -- no avoidance required
        else
            -- gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. "step1 dist: " .. obstacle_avoiding.distance_m .. " src: " .. obstacle_avoiding.label)
        end

        if (now_ms - now_debug_ms) > 2000 then
            gcs:send_text(MAV_SEVERITY.ERROR, "DETECTED: " ..  obstacle_avoiding.label .. " distance: " .. obstacle_avoiding.distance_m .. " m")
            now_debug_ms = now_ms
        end

        -- calculate the new target location based on the best bearing we found
        return location_project(current_loc, best_bearing_deg, current_loc:get_distance(target_loc), target_loc)
    end

    -- alert the pilot about any obstacles found
    function DAA.alert(alert_target_loc, obstacle)
        -- check for collisions, yes we don't actually do anything with this, just report it (if warn_action == 1)
        -- have_collided(current_loc)
        if alert_target_loc ~= nil and obstacle ~= nil then
            if obstacle.label ~= previous_label then
                if (obstacle.distance_m or 0) > 9 then
                    gcs:send_text(MAV_SEVERITY.WARNING, " ALERT: " .. obstacle.label .. " distance: " .. obstacle.distance_m .. " m ")
                else
                    gcs:send_text(MAV_SEVERITY.WARNING, " ALERT: " .. obstacle.label)
                end
                previous_label = obstacle_avoiding.label
            end
        else
            previous_label = ""
        end
    end

    -- a wrapper around vehicle:update_target_location()
    local function update_target_location(new_target_loc)
        if new_target_loc == nil or update_target_location_save_loc == nil then
            return false
        end
        new_target_loc:change_alt_frame(update_target_location_save_loc:get_alt_frame())
        local updated_location = vehicle:update_target_location(update_target_location_save_loc, new_target_loc)
        if updated_location then
            return true
        end
        gcs:send_text(MAV_SEVERITY.ERROR, "AVOID: UPDATE FAILED")
        return false
    end

    local function set_avoid_location(new_target_loc)
        if new_target_loc == nil then
            if update_target_location(navigation_target_loc) then
                daa_target_loc = nil
                --gcs:send_text(MAV_SEVERITY.WARNING, "AVOID: REVERT target to navigation target")
            end
            return false
        end
        if not locations_equal(daa_target_loc, new_target_loc) then
            if update_target_location(new_target_loc) then
                daa_target_loc = new_target_loc:copy()
                -- gcs:send_text(MAV_SEVERITY.WARNING, "AVOID: AVOID set new target")
                return true
            end
        end
        return false
    end

    local function avoid_obstacle(new_target_loc, obstacle)
        if obstacle == nil then -- no obstacle, so clear any specific avoidance we might have been doing
            if current_state == STATE. loitering then
                loiteralt.stop()
            end
            current_state = STATE.monitoring
            -- reset the target back to the original target
            new_target_loc = nil
        elseif obstacle.type == OBSTACLE_TYPE.GENERAL_AVIATION and false then
        -- depending on the obstacle we might do different things. Specifically if the obstacle is a crude aircraft
        -- in Canada we want to do a "Right 2" circuit descending to XXX altitude
        
            -- TODO make the target altitude a parameter
            new_target_loc = loiteralt.start(50.0, true, groundspeed_ms)
            if set_avoid_location(new_target_loc) then
                -- TODO SET LOITER RADIUS
            end
            current_state = STATE.loitering
        end
        -- if we have a new target - update it if it's different from our current target otherwise revert to the original target
        if set_avoid_location(new_target_loc) then
            local avoid_dist = navigation_target_loc:get_distance(new_target_loc)
            gcs:send_named_float("AVOIDING - DIST", avoid_dist)
            if (now_ms - now_avoiding_ms) < 5000  and obstacle ~= nil then
                gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " AVOIDING: " .. obstacle.label .. " dist: " .. avoid_dist .. " alt: " .. new_target_loc:alt() / 100.0 .. ":".. new_target_loc:get_alt_frame())
                now_avoiding_ms = now_ms
            end
        end
    end

    -- execute avoidance manoevers depending on the nature of the obstacle
    function DAA.avoid(new_target_loc)
        --[[if new_target_loc == nil  and not locations_equal(daa_target_loc, navigation_target_loc) then
            -- if avoidance doesn't require a new target - revert to the original target
            new_target_loc = navigation_target_loc
            if new_target_loc ~= nil then
                gcs:send_text(MAV_SEVERITY.ERROR, "AVOID: REVERT")
            end
        end
        --]]
        --if new_target_loc ~= nil then
            avoid_obstacle(new_target_loc, obstacle_avoiding)
            --logger.write("AVDM", 'Res,BCh,M1,M2','ffff',gcs_action, wrap_180(best_bearing_deg - bearing_deg), best_margin, -1)

        --end
    end
end) () -- DAA management class

-------------------------------------------------------------------------------
--- Main script execution and update loop including RC on/off management
-------------------------------------------------------------------------------
local last_switch_state = 0
local no_DAA_displayed = false

local function update()
    refresh_parameters()

    local switch_function = DAA_ACT_FN:get()
    if switch_function == nil then
        if ~no_DAA_displayed then
    	    gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " no DAA function")
            no_DAA_displayed = true
        end
        return
    end
    local switch_state = rc:get_aux_cached(switch_function) or -1
    if (switch_state ~= last_switch_state) then
	    -- gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " switch:"..switch_state)
        if switch_state == 0 then -- switch Low to disarm - so defaults to on
            DAA.enable()
        elseif switch_state == 2 then -- switch High to turn off
            DAA.disable()
        end
        last_switch_state = switch_state
    end

    DAA.get_vehicle_state()
    if DAA.active() then
        local suggested_target_loc = DAA.detect()
        DAA.alert(suggested_target_loc)
        DAA.avoid(suggested_target_loc)
    end
end

-- wrapper around update(). This calls update() at REFRESHRATE Hz, i.e. every 1000/REFRESH_RATE milliseconds
-- and if update faults then an error is displayed, but the script is not
-- stopped
function Protected_Wrapper()
    local success, err = pcall(update)
    if not success then
       gcs:send_text(0, SCRIPT_NAME_SHORT .. ": Error: " .. err)
       -- when we fault we run the update function again after 1s, slowing it
       -- down a bit so we don't flood the console with errors
       return Protected_Wrapper, 1000
    end
    return Protected_Wrapper, 1000 / refresh_rate
end

function Delayed_Startup()
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
    -- DAA defaults to on but can be disabled using a scripting aux function
    DAA.enable()
    return Protected_Wrapper()
end

-- wait a bit for AP to come up then start running update loop, unless armed 
if arming:is_armed() then
    return Delayed_Startup()
else
    return Delayed_Startup, 1000 * STARTUP_DELAY
end
