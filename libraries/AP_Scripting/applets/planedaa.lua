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

SCRIPT_NAME         = "Plane DAA"
SCRIPT_NAME_SHORT   = "PlaneDAA"
SCRIPT_VERSION      = "4.8.0-016"

STARTUP_DELAY = 25  -- wait this many seconds for the FC to come up before starting the script

PLANE_MODE          = {CIRCLE = 1, STABILIZE = 2, TRAINING = 3, ACRO = 4, FBWA = 4, FBWB = 6, CRUISE = 7, 
                        AUTOTUNE = 8, AUTO=10, RTL=11, LOITER=12, TAKEOFF = 13, AVOID_ADSB = 14, GUIDED=15, 
                        QSTABILIZE = 17,  QHOVER=18, QLOITER=19, QLAND = 20, QRTL=21, QAUTOTUNE = 22, QACRO = 23, 
                        THERMAL = 24, LOITER_ALT_QLAND = 25, AUTOLAND = 26}

ALT_FRAME           = {GLOBAL = 0, RELATIVE = 1, ORIGIN = 2, TERRAIN = 3}

MAV_SEVERITY        = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
MAV_CMD_INT         = {DO_SET_MODE = 176, DO_CHANGE_SPEED = 178, DO_REPOSITION = 192, 
                        GUIDED_CHANGE_SPEED = 43000, GUIDED_CHANGE_ALTITUDE = 43001, GUIDED_CHANGE_HEADING = 43002}
MAV_SPEED_TYPE      = {AIRSPEED = 0, GROUNDSPEED = 1, CLIMB_SPEED = 2, DESCENT_SPEED = 3}
MAV_HEADING_TYPE    = {COG = 0, HEADING = 1} -- COG = Course over Ground, i.e. where you want to go, HEADING = which way the vehicle points 

MAV_VTOL_STATE      = {UNDEFINED = 0, TRANSITION_TO_FW = 1, TRANSITION_TO_MC = 2, MC = 3, FW = 4 }

-- MAV_COLLISION_THREAT_LEVEL
MAV_COLLISION_THREAT_LEVEL = {
    NONE                        = 0,    -- Not a threat
    LOW                         = 1,    -- Mild concern about this threat
    HIGH                        = 2,    -- Craft is panicking and may take action to avoid
    ENUM_END                    = 3     -- End of enum
}
-- MAV_COLLISION_SRC
MAV_COLLISION_SRC = {
    ADSB                        = 0,    -- Source is ADSB_VEHICLE packets
    MAVLINK_GPS_GLOBAL_INT      = 1,    -- Source is MAVLink GPS_GLOBAL_INT
    ENUM_END                    = 2     -- End of enum
}

MAV_COLLISION_ACTION = {
    NONE                        = 0,    -- Ignore any potential collisions 
    REPORT                      = 1,    -- Report potential collision 
    ASCEND_OR_DESCEND           = 2,    -- Ascend or Descend to avoid threat 
    MOVE_HORIZONTALLY           = 3,    -- Move horizontally to avoid threat 
    MOVE_PERPENDICULAR          = 4,    -- Aircraft to move perpendicular to the collision's velocity vector 
    RTL                         = 5,    -- Aircraft to fly directly back to its launch point 
    HOVER                       = 6,    -- Aircraft to stop in place 
    LOITERTURN                  = 7,    -- Aircraft to do a loiter turn left or right to lose altitude
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

-- ADSB Emitter types
ADSB_EMITTER = {
    NO_INFO                     = 0,
    LIGHT                       = 1,
    SMALL                       = 2,
    LARGE                       = 3,
    HIGH_VORTEX_LARGE           = 4,
    HEAVY                       = 5,
    HIGHLY_MANUV                = 6,
    ROTOCRAFT                   = 7,    -- this is Helicopter
    -- 8 Unassigned
    GLIDER                      = 9,
    LIGHTER_AIR                 = 10,
    PARACHUTE                   = 11,
    ULTRA_LIGHT                 = 12,
    AIRCRAFT_HIGH               = 13,
    UAV                         = 14,   -- this is drones
    SPACE                       = 15,   -- this is rockets
    --16 Unassigned

    -- Surface types
    EMERGENCY_SURFACE           = 17,
    SERVICE_SURFACE             = 18,

    -- Obstacle types
    POINT_OBSTACLE              = 19,
    CLUSTER_OBSTACLE            = 20,
    LINE_OBSTACLE               = 21,
    -- 22 - 39 Reserved

}

local DAA_active = true;

local PARAM_TABLE_KEY = 125
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

-- setup follow mode specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 30), SCRIPT_NAME_SHORT .. ' could not add param table: ' .. PARAM_TABLE_PREFIX .. " key: " .. PARAM_TABLE_KEY)

--[[
    // @Param: DAA_ACT_FN
    // @DisplayName: DAA Activation
    // @Description: Disable the DAA capability or turn it off (defaults to on)
    // @Range: 300 307
--]]
DAA_ACT_FN = bind_add_param("ACT_FN", 1, 308)

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
  // @Description: Avoidance margin for Fixed Wing aircraft/General Aviation (Helicopters? eVTOL?) over and above the Well Clear margin AVD_WCLR_XY
ß  // @Units: m
--]]
DAA_MARGIN_GA  = bind_add_param('MARGIN_GA', 10, 50)

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
  // @Param: DAA_MARGIN_AIS
  // @DisplayName: Margin for AIS (ships)
  // @Description: Avoidance radius for AIS (MAVLink sourced)
  // @Units: m
--]]
DAA_MARGIN_AIS  = bind_add_param('MARGIN_AIS', 15, 50)

--[[
  // @Param: DAA_MARGIN_PRX
  // @DisplayName: Margin for proximity 
  // @Description: Avoidance radius for obstacles detected by proximity sensors. Typically pretty close
  // @Units: m
--]]
DAA_MARGIN_PRX  = bind_add_param('MARGIN_PRX', 16, 50)

--[[
// @Param: BR_RATIO
    // @DisplayName: DAA margin ratio for BendyRuler to change bearing significantly 
    // @Description:  DAA BendyRuler will avoid changing bearing unless ratio of previous margin from obstacle (or fence) to present calculated margin is at least this much.
    // @Range: 1.1 2
    // @Increment: 0.1
    // @User: Standard
--]]
DAA_BR_RATIO = bind_add_param('BR_RATIO', 17, 1.5)

--[[
    // @Param: BR_ANGLE
    // @DisplayName: BendyRuler's bearing change resistance threshold angle   
    // @Description:  DAA BendyRuler will resist changing current bearing if the change in bearing is over this angle
    // @Range: 20 180
    // @Increment: 5
    // @User: Standard
--]]
DAA_BR_ANGLE = bind_add_param('BR_ANGLE', 18, 75)

--[[
    // @Param: AVD_ALT
    // @DisplayName: The altitude to loiter to when avoiding a crude aircraft   
    // @Description:  DAA will loiter and descent to this altitude if a crude aircraft is detected within DAA_MARGIN_GA of the vehicle. Ignored if zero (0).
    // @Range: 20 5000
    // @Increment: 5
    // @User: Standard
--]]
DAA_AVD_ALT = bind_add_param('AVD_ALT', 19, 50)

--[[
    // @Param: AVD_ALT_TP
    // @DisplayName: The frame of the DAA_AVD_ALT  
    // @Description:  DAA will loiter and descent to DAA_AVD_ALT in this frame. 0: Absolute, 1: Above Home, 2: Above Origin, 3: Above Terrain (default)
    // @Range: 20 5000
    // @Increment: 5
    // @User: Standard
--]]
DAA_AVD_ALT_TP = bind_add_param('AVD_ALT_TP', 20, 3)

--[[
    // @Param: AVD_ALERT
    // @DisplayName: Alert for DAA Avoidance 
    // @Description: Alert or not Alert 
    // @Values: 0: None, 1: Alert
    // @User: Standard
--]]
DAA_AVD_ALERT = bind_add_param('AVD_ALERT', 21, 1)

--[[
    // @Param: AVD_ACTION
    // @DisplayName: Action for DAA Avoidance 
    // @Description: Action for DAA Avoidance
    // @Values: 0: None, 1: Avoid
    // @User: Standard
--]]
DAA_AVD_ACTION = bind_add_param('AVD_ACTION', 22, 1)

WARN_DIST_XY                = bind_param("AVD_W_DIST_XY")
WARN_ACTION                 = bind_param("AVD_W_ACTION")
AVD_ENABLE                  = bind_param("AVD_ENABLE")
AVD_WCLR_XY                 = bind_param("AVD_WCLR_XY")
AVD_WCLR_Z                  = bind_param("AVD_WCLR_Z")
AVD_NMAC_XY                 = bind_param("AVD_NMAC_XY")
AVD_NMAC_Z                  = bind_param("AVD_NMAC_Z")
ROLL_LIMIT_DEG              = bind_param("ROLL_LIMIT_DEG")
WP_LOITER_RAD               = bind_param("WP_LOITER_RAD")

local roll_limit_deg        = ROLL_LIMIT_DEG:get()
local lookahead_param       = DAA_LKAHD:get()
local margin_fence          = DAA_MARGIN_FENCE:get()
local margin_aircraft       = DAA_MARGIN_GA:get()
local margin_bird           = DAA_MARGIN_BIRD:get()
local margin_prey           = DAA_MARGIN_PREY:get()
local margin_uav            = DAA_MARGIN_UAV:get()
local margin_weather        = DAA_MARGIN_WTHR:get()
local margin_ais            = DAA_MARGIN_AIS:get()
local margin_proximity      = DAA_MARGIN_PRX:get()
local refresh_rate          = 1000.0 / DAA_UPDATE_RATE:get()
local bendy_ratio           = DAA_BR_RATIO:get()
local bendy_angle           = DAA_BR_ANGLE:get()
local wp_loiter_rad         = WP_LOITER_RAD:get()
local ga_avoid_alt          = DAA_AVD_ALT:get()
local ga_avoid_alt_frame    = DAA_AVD_ALT_TP:get()
local daa_alert             = DAA_AVD_ALERT:get()
local daa_action            = DAA_AVD_ACTION:get()
local well_clear_xy         = AVD_WCLR_XY:get()
local well_clear_z          = AVD_WCLR_Z:get()
local near_miss_xy          = AVD_NMAC_XY:get()
local near_miss_z           = AVD_NMAC_Z:get()

GRAVITY_MSS = 9.80665
LOCATION_SCALING_FACTOR_INV = 89.83204953368922

-- TODO should be a parameter (also convert to degrees)
local bearing_inc_cd = 1500
local bearing_inc_deg = 1.5

COLLISION_DETECTED = false

FLT_MAX = 3.402823466e+38

-------------------------------------------------------------------------------
--- Vehicle State stored in local variables to reduce api calls
-------------------------------------------------------------------------------

local current_loc           = ahrs:get_position()
local current_heading_deg   = math.deg(ahrs:get_yaw_rad())
local current_mode          = vehicle:get_mode()
local vtol_state            = MAV_VTOL_STATE.UNDEFINED

local now_ms                = millis()
local now_params_ms         = now_ms
local now_debug_ms          = now_ms
local now_avoiding_ms       = now_ms
local now_obstacle_ms       = now_ms
local now_aircraft_ms       = now_ms
local now_loitering_ms      = now_ms
local aircraft_seen_now_ms  = now_ms

-------------------------------------------------------------------------------
--- Lua Modules
-------------------------------------------------------------------------------
---
---
local mavlink_wrappers = require("mavlink_wrappers")

-------------------------------------------------------------------------------
--- Utility methods
-------------------------------------------------------------------------------
---
---

local function get_mode_string(mode)
    if mode == PLANE_MODE.AUTO then
        return "Auto"
    elseif mode == PLANE_MODE.RTL then
        return "RTL" 
    elseif mode == PLANE_MODE.LOITER then
        return "Loiter"
    elseif mode == PLANE_MODE.GUIDED then
        return "Guided"
    elseif mode == PLANE_MODE.QSTABILIZE then 
        return "Q Stabilize"
    elseif mode == PLANE_MODE.QHOVER then 
        return "Q Hover"
    elseif mode == PLANE_MODE.QLOITER then
        return "Q Loiter"
    elseif mode == PLANE_MODE.QLAND then
        return "Q Land"
    elseif mode == PLANE_MODE.QAUTOTUNE then
        return "Q Autotune"
    elseif mode == PLANE_MODE.FBWA then
        return "FBWA"
    elseif mode == PLANE_MODE.FBWB then
        return "FBWB"
    elseif mode == PLANE_MODE.MANUAL then
        return "Manual"
    elseif mode == PLANE_MODE.CRUISE then
        return "Cruise"
    elseif mode == PLANE_MODE.AUTOTUNE then
        return "Autotune"
    elseif mode == PLANE_MODE.TAKEOFF then
        return "Takeoff"
    elseif mode == PLANE_MODE.AVOID_ADSB then
        return "Avoid ADSB"
    elseif mode == PLANE_MODE.THERMAL then
        return "Thermal"
    elseif mode == PLANE_MODE.LOITER_ALT_QLAND then
        return "Loiter Alt Q Land"
    elseif mode == PLANE_MODE.AUTOLAND then
        return "Autoland"
    end

    return string.format("mode: %d", mode)
end

-- keep local copies of parameter values that the user might change so update ever 5 seconds
local function get_vehicle_state()

    current_loc         = ahrs:get_position()
    current_heading_deg = math.deg(ahrs:get_yaw_rad())
    current_mode        = vehicle:get_mode()
    if quadplane then
        vtol_state      = quadplane:get_mav_vtol_state()
    else
        vtol_state          = MAV_VTOL_STATE.UNDEFINED
    end

    now_ms = millis()

    -- refresh parameters every 5 seconds, its not that urgent we know about changs
    if (now_ms - now_params_ms) > 5000 then
        --warn_act = WARN_ACTION:get()
        roll_limit_deg      = ROLL_LIMIT_DEG:get()
        lookahead_param     = DAA_LKAHD:get()
        margin_fence        = DAA_MARGIN_FENCE:get()
        margin_aircraft     = DAA_MARGIN_GA:get()
        margin_bird         = DAA_MARGIN_BIRD:get()
        margin_prey         = DAA_MARGIN_PREY:get()
        margin_uav          = DAA_MARGIN_UAV:get()
        margin_weather      = DAA_MARGIN_WTHR:get()
        margin_ais          = DAA_MARGIN_AIS:get()
        margin_proximity    = DAA_MARGIN_PRX:get()
        refresh_rate        = 1000.0 / DAA_UPDATE_RATE:get()
        bendy_ratio         = DAA_BR_RATIO:get()
        bendy_angle         = DAA_BR_ANGLE:get()
        wp_loiter_rad       = WP_LOITER_RAD:get()
        ga_avoid_alt        = DAA_AVD_ALT:get()
        ga_avoid_alt_frame  = DAA_AVD_ALT_TP:get()
        daa_alert           = DAA_AVD_ALERT:get()
        daa_action          = DAA_AVD_ACTION:get()

        well_clear_xy        = AVD_WCLR_XY:get()
        well_clear_z         = AVD_WCLR_Z:get()
        near_miss_xy         = AVD_NMAC_XY:get()
        near_miss_z          = AVD_NMAC_Z:get()

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

-- make obstacle labels a bit more meaningful for user especially for crude aircraft and MAVLink vehicles
local function pretty_label(script_obstacle)
    local obstacle_type = script_obstacle:obstacle_type()
    local emitter_type  = script_obstacle:emitter_type()

    -- this will typically be an GLOBAL_POSITION_INT (or FOLLOW_TARGET?) message
    if script_obstacle:is_drone() == true or emitter_type == ADSB_EMITTER.UAV then
        return string.format("SYSID:%d", script_obstacle:src_id())

    -- this will have arrived as an ADSB_VEHICLE
    elseif script_obstacle:is_aircraft() == true or emitter_type == 100
            or (emitter_type >= ADSB_EMITTER.LIGHT and emitter_type <= ADSB_EMITTER.AIRCRAFT_HIGH) then
        return string.format("%06X", script_obstacle:icao_code())

    -- fake generated obstacles from mavproxy_genobstacles have these special case "emitters" for SITL/testing
    elseif emitter_type == 99 then
        return "Obstacle"
    elseif emitter_type == 101 then
        return "Drone"
    elseif emitter_type == 102 then
        return "Weather"
    elseif emitter_type == 103 then
        return "Migratory Bird"
    elseif emitter_type == 104 then
        return "Bird of Prey"

    -- these obstacle types are returned by AP_OAScripting for fences
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION then
        return "Excl. Circle"
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION then
        return "Incl. Circle"
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_POLYGON_EXCLUSION then
        return "Excl. Polygon"
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_POLYGON_INCLUSION then
        return "Incl. Polygon"
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_HOME then
        return "Tin Can"
    elseif obstacle_type == OBSTACLE_TYPE.FENCE_LUA then
        return "Lua Fence"
    end
    -- gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": UNKNOWN: " .. script_obstacle:icao_code() .. " drone? " .. script_obstacle:is_drone() .. " aircraft:" .. script_obstacle:is_aircraft() .. " type: " .. obstacle_type)
    return "Unknown"
end

local function populate_obstacle(distance_m, any_obstacle)
    local obstacle = {}
    obstacle.distance_m   = distance_m                      -- this is the Projected distance based on lookahead
    obstacle.sysid        = any_obstacle:src_id()
    obstacle.icao_code    = any_obstacle:icao_code()
    obstacle.type         = any_obstacle:obstacle_type()
    obstacle.label        = pretty_label(any_obstacle)
    obstacle.location     = any_obstacle:location()
    obstacle.pos_NED_m    = any_obstacle:position_NED_m()
    obstacle.vel_NED_ms   = any_obstacle:velocity_NED_ms()
    -- these are the actual distances based on current location with no lookahead
    -- note that for polygons there is no "location", so it's not easy to find the simple distance, so we just use the OA distance
    if obstacle.location == nil then
        obstacle.distance_xy = obstacle.distance_m
        obstacle.distance_z  = 0
    else
        obstacle.distance_xy  = obstacle.location:get_distance(current_loc)
        obstacle.distance_z   = math.abs(obstacle.location:get_distance_NED(current_loc):z())
        if obstacle.type == OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION then
            obstacle.distance_xy = obstacle.distance_xy - any_obstacle:radius_m() - any_obstacle:margin_m()
        elseif obstacle.type == OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION then
            -- not too sure about the math here
            obstacle.distance_xy = any_obstacle:radius_m() - obstacle.distance_xy - any_obstacle:margin_m()
        end
    end

    return obstacle
end

local function find_closest_obstacle(loc1, loc2, lookahead_m)
    -- By projecting 1m along the line we avoid a problem with the
    -- exclusion avoidance being happy to skirt along a line parallel
    -- to an exclusion zone
    local bearing_deg   = math.deg(loc1:get_bearing(loc2))
    local loc1_shifted  = location_project(loc1, bearing_deg, 1, loc2)
    local obstacle = {}

--[[
    obstacle.distance_m,
    obstacle.type,
    obstacle.label,
    obstacle.sysid,
    obstacle.location,
    obstacle.post_NED_m,
    obstacle.velocity       = OAScripting:find_closest_obstacle(loc1_shifted, loc2, lookahead_m)*/
--]]
    local distance_m, any_obstacle, _, _, _  =
            OAScripting:find_threats(loc1_shifted, loc2, lookahead_m)

    if distance_m == nil then
        return FLT_MAX, nil
    end

    if any_obstacle == nil then
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": threat Nil ")
    end

    local margin_fence          = DAA_MARGIN_FENCE:get()

    local obstacle_margin = 0;
    --[[
    These are currently handled inside find_threats, it would be better if they could be parameterized
    if any_obstacle.obstacle_type == OBSTACLE_TYPE.GENERAL_AVIATION then
        obstacle_margin = margin_aircraft
    elseif any_obstacle.obstacle_type == OBSTACLE_TYPE.MAV_SYSID then
        obstacle_margin = margin_uav
    elseif any_obstacle.obstacle_type == OBSTACLE_TYPE.BIRD_MIGRATORY then
        obstacle_margin = margin_bird
    elseif any_obstacle.obstacle_type == OBSTACLE_TYPE.BIRD_OF_PREY then
        obstacle_margin = margin_prey
    elseif any_obstacle.obstacle_type == OBSTACLE_TYPE.WEATHER then
        obstacle_margin = margin_weather
    else
    --]]
    if any_obstacle.obstacle_type == OBSTACLE_TYPE.GENERAL_AVIATION then
        obstacle_marget = well_clear_xy + margin_aircraft
    elseif any_obstacle.obstacle_type == OBSTACLE_TYPE.AIS then
        obstacle_margin = well_clear_xy + margin_ais
    elseif any_obstacle.obstacle_type == OBSTACLE_TYPE.PROXIMITY then
        obstacle_margin = margin_proximity
    elseif any_obstacle.obstacle_type == OBSTACLE_TYPE.FENCE_CIRCLE_EXCLUSION
        or any_obstacle.obstacle_type == OBSTACLE_TYPE.FENCE_CIRCLE_INCLUSION
        or any_obstacle.obstacle_type == OBSTACLE_TYPE.FENCE_POLYGON_INCLUSION
        or any_obstacle.obstacle_type == OBSTACLE_TYPE.FENCE_POLYGON_INCLUSION
        or any_obstacle.obstacle_type == OBSTACLE_TYPE.FENCE_HOME
        or any_obstacle.obstacle_type == OBSTACLE_TYPE.FENCE_LUA
        then
        obstacle_margin = margin_fence
    end

    -- gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": threat: " .. any_obstacle:label() .. " : " .. distance_m)
    if distance_m > obstacle_margin then
        -- we are further away from the obstacle than we care about
        return FLT_MAX, nil
    end

    obstacle = populate_obstacle(distance_m, any_obstacle)

    --gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": threat: " .. any_obstacle:icao_code() .. " drone? " .. any_obstacle:is_drone() .. " aircraft:" .. any_obstacle:is_aircraft())

    --gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": threat: " .. any_obstacle.: .. " :" .. obstacle.label)

    --gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": threat: " .. any_obstacle.obstacle_type() .. " :" .. obstacle.label)

    return distance_m, obstacle
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
}
(function ()
    local pre_loiteralt_heading_deg = -1.0
    local previous_mode = -1
    local target_alt_m = nil
    local target_alt_frame = ALT_FRAME.GLOBAL

    function loiteralt.start(new_alt_m, new_alt_frame, direction_right, speed_ms)
        local direction = ""

        if loiteralt.active then
            -- gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": loiteralt ALREADY ACTIVE: ")
            return nil
        end

        -- gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": loiteralt starting: " .. target_alt_m)

        if current_loc == nil then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": loiteralt no current_location")
            return nil
        end
        pre_loiteralt_heading_deg   = math.deg(ahrs:get_yaw_rad())
        target_alt_frame            = new_alt_frame
        target_alt_m                = new_alt_m

        --gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt start hdg pre %.0f now %.0f dif %0.1f alt %.0f",
        --                    pre_loiteralt_heading_deg, current_heading_deg
        --                    , math.abs(pre_loiteralt_heading_deg - current_heading_deg), target_alt_m) )

        -- this gives us a radius of an approximation of a "standard turn" based on groundspeed (why groundspeed?)
        local radius_m = (60.0 * speed_ms) / math.pi
        radius_m = wp_loiter_rad
        local loiteralt_loc = current_loc:copy()
        if direction_right then
            direction = "right"
            loiteralt_loc:offset_bearing(wrap_360(pre_loiteralt_heading_deg + 90), radius_m)
        else
            direction = "left"
            loiteralt_loc:offset_bearing(wrap_360(pre_loiteralt_heading_deg - 90), radius_m)
        end
        --local new_target_loc, direction = start_loiter(loiteralt.target_alt_agl_m, direction_right, groundspeed_ms)
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": LOITER %s to %.0f/%.0f(%.0f) alt radius %.0f m",
                direction, target_alt_m, target_alt_frame, mavlink_wrappers.alt_frame_to_mavlink(target_alt_frame), radius_m ))
        
        previous_mode = vehicle:get_mode()
        vehicle:set_mode(PLANE_MODE.GUIDED)

        if mavlink_wrappers.set_vehicle_target_location({lat    = loiteralt_loc:lat(),
                                                        lng     = loiteralt_loc:lng(),
                                                        alt     = target_alt_m,
                                                        frame   = target_alt_frame,
                                                        radius  = radius_m,
                                                        yaw     = 0 }) then
            loiteralt.active = true
        else
                gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt.stop set_vehicle FAILED" ))
            loiteralt.stop()
        end

        return nil
    end

    function loiteralt.aircraft_seen()
        aircraft_seen_now_ms = now_ms
    end

    function loiteralt.stop(force_stop)
        if not force_stop then
            -- we wait for 10 seconds to make sure that we really want to stop
            if (now_ms - aircraft_seen_now_ms) < 10000 then
                return false
            end
        end
        if not force_stop and false then
            -- if we are pointing in the wrong direction for the requried next waypoint, keep going until we are 
            -- pointing more or less in the right direction (but not if force_stop is true)
            local current_alt_m = current_loc:get_alt_m(target_alt_frame)
            if (math.abs(current_alt_m - target_alt_m)) > 10 or
                    (math.abs(wrap_180(pre_loiteralt_heading_deg - current_heading_deg)) < 45.0) then
                gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt STOP? alt  %.0f trg %.0f hdg: %.0f prv: %.0f",
                            current_alt_m, target_alt_m, current_heading_deg, pre_loiteralt_heading_deg) )
                -- not ready to stop yet
                return false
            end
        end

        if previous_mode > 0 and previous_mode ~= PLANE_MODE.GUIDED then
            vehicle:set_mode(previous_mode)
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Loiter Done set mode: %s", get_mode_string(previous_mode) ))
            gcs:send_named_string("DAA-AVOID", "")
            gcs:send_named_float("DAA-LOITER", 0.0)
        end
        previous_mode = -1
        loiteralt.active = false
        return true
    end

    -- should be called regularly if loiteralt is active
    function loiteralt.update()
        if active and current_mode ~= PLANE_MODE.GUIDED then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Pilot changed from GUIDED to: %.0f", current_mode ))
            previous_mode = -1
            loiteralt.stop(true)
            return
        end
        if not active or current_loc == nil then
            return
        end

        local current_location_agl = current_loc:copy()
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
        --local current_agl_m = current_location_agl:alt() * 0.01

        --[[if (math.abs(current_agl_m - loiteralt.target_alt_agl_m)) < 10 and 
                (math.abs(pre_loiteralt_heading_deg - current_heading_deg) < 45.0) then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt STOP alt curr %.0f trg %.0f max %0.1f",
                        current_altitude_m, current_amsl_m, altitude_max) )
            loiteralt.stop()
        else
            --airspeed_desired = airspeed_cruise
            --mavlink.set_vehicle_speed({speed=airspeed_desired})
        end
        --]]
    end
end)()

-------------------------------------------------------------------------------
--- DAA (Detect, Alert, Avoid) management class
-------------------------------------------------------------------------------
DAA = {
   enabled = false,
}
(function ()
    local active            = true;
    local navigating        = false;
    local current_loc       = ahrs:get_position()
    local groundspeed_ms    = ahrs:groundspeed_vector():length()
    local airspeed_ms       = ahrs:airspeed_EAS() or groundspeed_ms
    local ground_course_deg = wrap_180(math.deg(ahrs:groundspeed_vector():angle()))
    local wind_dir_rad      = 0.0
    local wind_speed        = 0.0
    local obstacle_avoiding = nil
    local aircraft_avoiding = nil
    local previous_label    = ""
    local avoiding_label    = ""
    local previous_aircraft = ""
    local STATE             = {monitoring = 1, avoiding = 2, loitering = 3, hovering = 4, landing  = 5}
    local current_state     = STATE.monitoring
    local LoWC_active       = false
    local LoWC_label        = ""
    local NMAC_active       = false
    local NMAC_label        = ""

    local update_target_location_save_loc = nil                 -- this is the saved current_target for use by update_target_location ONLY
    local navigation_target_loc = nil                           -- this is where the vehicle is trying to get to (i.e. next waypoint if no avoidance)
    local daa_target_loc = nil                                  -- this is where the DAA is currently trying to go in order to avoid obstacles (nil if not avoiding)

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

    -- methods to log DAA results DAAD = Detect, DAAA = Alert, DAAV = aVoid
    local function log_detect_result(obstacle_found, distance_found_m, distance_to_target_m, best_bearing_deg, target_loc)
        --    log_detect_result(false, distance_found_m, distance_to_target_m, best_bearing_deg, target_loc)
        if target_loc == nil or distance_found_m == nil or distance_to_target_m == nil or best_bearing_deg == nil then
            -- we can't be avoiding if no target, so no loggin required
            return
        end
        --print(distance_found_m)
        --print(distance_to_target_m)
        --print(target_loc:lat())
        --print(target_loc:lng())
        --print(target_loc:alt())

        --logger:write('DAAD',                -- D for Detect
        local status, err = pcall(logger.write, logger, "DAAD",
            'Obs,DstF,DstT,HdgB,Tfnd,TLat,TLng,TAlt,TFra',
            'BffffLLfB',                    -- Formats (L for Lat/Lng, f for Alt)
            '-mmmdDUm-',                    -- Units (D=lat deg, U=lng deg, m=meter)
            '-----GG--',                    -- Multipliers (G=1e-7 for L types)
            (obstacle_found and 1 or 0),    -- Obs - Obstacle found true/false
            distance_found_m,               -- DstF - Distance to found obstacle in meters
            distance_to_target_m,           -- DstT - Distance to proposed new target to avoid the obstacle
            best_bearing_deg,               -- HdgB - Best bearing found to avoid obstacles
            (has_target and 1 or 0),        -- TFnd - Target found
            target_loc:lat(),               -- TLat - Latitude of proposed new target in degrees
            target_loc:lng(),               -- TLng - Longitude of proposed new target in degrees
            target_loc:alt() * 0.01,        -- TAlt - Alitude of proposed new target in meters
            target_loc:get_alt_frame())     -- TFrm - Frame of the ALtitlde: 0: AMSL, 1: Home Relative, 3: Terrain Relative

        if not status then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " log detect:" .. tostring(err) )
        end
    end

    local function log_detect_aircraft(aircraft)
        if aircraft == nil then
            return
        end

        --logger:write('DAAG',                    -- G for GA = General Aviation
        local status, err = pcall(logger.write, logger, "DAAG",
            'DstF,TLat,TLng,TAlt,TFra,DstH,DstZ,ICAO',
            'fLLfBffI',                          -- Formats (L for Lat/Lng, f for Alt)
            'mDUm-mmh',                          -- Units (D=lat deg, U=lng deg, m=meter)
            '-GG-----',                          -- Multipliers (G=1e-7 for L types)
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

    local function log_alert()
    end

    local function log_avoid(obstacle, target_loc)
        if target_loc == nil then
            return
        end
        local status, err = pcall(logger.write, logger, "DAAV",
        --logger:write('DAAV',                        -- V for aVoid
            'DstO,TLat,TLng,TAlt,TFra,DstH,DstZ,TypO',
            'fLLfBffB',                             -- Formats (L for Lat/Lng, f for Alt)
            'mDUm-mm-',                             -- Units (D=lat deg, U=lng deg, m=meter)
            '-GG-----',                             -- Multipliers (G=1e-7 for L types)
            obstacle.distance_m,                    -- DstO - Distance to found obstacle in meters
            target_loc:lat(),                       -- TLat - Latitude of DAA target in degrees
            target_loc:lng(),                       -- TLng - Longitude of DAA target in degrees
            target_loc:alt() * 0.01,       -- TAlt - Alitude of proposed new target in meters
            target_loc:get_alt_frame(),    -- TFrm - Frame of the ALtitlde: 0: AMSL, 1: Home Relative, 3: Terrain Relative
            obstacle.distance_xy,                   -- DstH - Horizontal distance to the obstacle
            obstacle.distance_z,                    -- DstZ - Vertical distance to the aircraft (+ve is up),
            obstacle.type                           -- ObsT - the type of the obstacle as an OBSTACLE_TYPE
        )
        if not status then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " log avoid:" .. tostring(err) )
        end
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
    function DAA.isactive()
        return DAA.enabled and active and arming:is_armed()
    end

    -- populate some local values with a static/consistent picture of the vehicle state
    function DAA.get_vehicle_state()
        local current_target_loc = vehicle:get_target_location()

        active      = true;
        current_loc = ahrs:get_position()

        if OAScripting == nil then
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " OAScripting object is nil!")
            active = false
            return 
        end

        if current_loc == nil or current_target_loc == nil then
            -- no position or not navigating
            navigation_target_loc   = nil
            daa_target_loc          = nil
            if navigating then
    	        gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. "NOT NAVIGATING")
                navigating = false
            end
            active = false
            return
        end

        -- if we got here we have a current location (AHRS active) and a current navigation target
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

        groundspeed_ms              = ahrs:groundspeed_vector():length()
        airspeed_ms                 = ahrs:airspeed_EAS() or groundspeed_ms
        -- Calculate wind direction and speed
        wind_speed, wind_dir_rad    = calculate_windspeed()
        ground_course_deg           = wrap_180(math.deg(ahrs:groundspeed_vector():angle()))
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
    local function resist_bearing_change(bearing_orig_deg, avoid_step1_m, bearing_deg, distance_found_m)
        if distance_found_m ==  0 then
            return distance_found_m, bearing_deg
        end
	    if math.abs(wrap_180(bearing_orig_deg - bearing_deg)) < bendy_angle then
            return distance_found_m, bearing_deg
        end
        -- gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. 
        --    " RESIST: bearing orig: " .. bearing_orig_deg .. " test:" .. bearing_deg .. " dist: " .. avoid_step1_m .. " found: " .. distance_found_m)
        -- check margin in last bearing's direction
        local test_loc_previous_bearing = current_loc:copy()
        test_loc_previous_bearing:offset_bearing(wrap_180(bearing_orig_deg), avoid_step1_m)

        local distance_previous_m, _ = find_closest_obstacle(current_loc, test_loc_previous_bearing, avoid_step1_m)
        if (math.abs(distance_previous_m) < math.abs(bendy_ratio * distance_found_m)) then
            -- don't change direction abruptly. If margin difference is not significant, follow the last direction
            -- gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. " RESIST: was " .. bearing_test .. " now " .. bearing_orig .. " found: " .. math.abs(distance_found) .. " ratio: " .. math.abs(bendy_ratio * distance_found) .. " new dist:" .. distance_previous_m)
            bearing_deg = bearing_orig_deg
            distance_found_m  = distance_previous_m
        end

        return distance_found_m, bearing_deg
    end

    -- calculates the second step of the bendy ruler test - look foward a 2nd "full_distance" to see if we can still avoid obstacles
    local function test_step2(loc_test, target_bearing, avoid_step2_m, full_distance)
        local test_bearings = { 0, 45, -45 }
        local target_loc = loc_test:copy()
        target_loc:offset_bearing(target_bearing, avoid_step2_m)

        -- local obstacle_found = {}
        local bearing_found = target_bearing
        local closest_distance_m  = FLT_MAX

        for _, delta in ipairs(test_bearings) do
            local bearing_test = target_bearing + delta

            local target_distance   = loc_test:get_distance(target_loc)
            local distance          = calc_avoidance_distance(avoid_step2_m, target_distance)
            local loc_test2         = location_project(loc_test, bearing_test, distance, target_loc)

            local distance_m, _     = find_closest_obstacle(loc_test2, target_loc, current_lookahead)

            if distance_m > current_lookahead then
                -- return immediately - no obstacles in this direction
                return bearing_test, distance_m
            end
            if distance_m < closest_distance_m then
                -- return the bearing to the nearest obstacle
                bearing_found       = bearing_test
                closest_distance_m  = distance_m
            end
        end

        return bearing_found, closest_distance_m
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

        local avoid_step1_m     = current_lookahead
        local avoid_step2_m     = current_lookahead * 2.0

        -- Test bearing used to look ahead for obstacles
        -- local bearing_test = wrap_180((bearing_cd*0.01) + (bearing_delta_cd*0.01))
        local bearing_test_deg  = wrap_180(bearing_deg + bearing_delta_deg)
        -- local adjusted_loc = location_after_course_change(current_loc, bearing_test_deg, target_loc)
        -- fudge to ignore the course change
        local adjusted_loc      = current_loc

        -- Position after one step from where we think we will be after turning to bearing_test_deg
        local avoidance_distance_m  = calc_avoidance_distance(avoid_step1_m, full_distance)
        local test_loc              = location_project(adjusted_loc, bearing_test_deg, avoidance_distance_m, target_loc)

        local distance_found_m, obstacle_found = find_closest_obstacle(adjusted_loc, test_loc, current_lookahead)
        ---@cast margin number
        if distance_found_m == nil then
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. "closest returned NIL ")
            return FLT_MAX, bearing_deg, nil -- no avoidance required
        end
        if distance_found_m > current_lookahead then
            -- This direction avoids all obstacles for one step. Check if it leads to a clear path for a longer distance.
            local bearing2_deg, distance2_m = test_step2(test_loc, bearing_test_deg, avoid_step2_m, current_lookahead)
            if distance2_m >= current_lookahead then
                if i == 0 and bearing2_deg == bearing_deg then
                    -- means we have a direct unobstructed path for step1 (i == 0) and step2
            	    -- gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. "UNOBSTRUCTED  bearing: " .. bearing2)
                    return FLT_MAX, bearing_deg, nil -- no avoidance required
                end
                -- we've found at least one direction where there is no obstacle at least for 2 steps out
                distance_found_m = distance_found_m + distance2_m
            end
        end
	-- distance_found_m, bearing_test_deg = resist_bearing_change(bearing_deg, avoid_step1_m, bearing_test_deg, distance_found_m)

        return distance_found_m, bearing_test_deg, obstacle_found
    end

    -- if the plane is currently pointing far away from the target, then assume that we 
    -- will be turning sharply, so we don't look too far ahead for obstacles
    local function limit_distance(from_loc, to_loc, bearing_deg)
        local distance_to_target_m = from_loc:get_distance(to_loc)

        if (math.abs(wrap_180(bearing_deg - ground_course_deg)) > bendy_angle * 2) then
            distance_to_target_m = wp_loiter_rad * 3
        end

        return distance_to_target_m
    end

    -- crude aircraft are a special case. We do specific things if there is an aircraft nearby so we need to know the nearest one
    local function detect_aircraft()
    --[[ local obstacle = {}

        obstacle.distance_m,
        obstacle.type,
        obstacle.label,
        obstacle.sysid,
        obstacle.location,
        obstacle.post_NED_m,
        obstacle.velocity       
    --]]

        local distance_m, aircraft_obstacle = OAScripting:find_aircraft(current_loc, margin_aircraft)

        if distance_m == nil then
            aircraft_avoiding = nil
            return
        end

        local obstacle = populate_obstacle(distance_m, aircraft_obstacle)
        --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" FOUND AIRCRAFT: %s dist: %.0f m alt: %.0f m", obstacle.label, obstacle.distance_m, obstacle.location:alt() * 0.01 ))

        aircraft_avoiding = obstacle

        log_detect_aircraft(aircraft_avoiding)
        --gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(" DETECTED AIRCRAFT: xy %.0f z %.0f", aircraft_avoiding.distance_xy,aircraft_avoiding.distance_z) )
 
    end


    -- detect flying objects or fences when flying towards navigation_target_loc
    function DAA.detect()
        -- TODO be smarter about re-populating this
        local obstacle_distance_m = FLT_MAX
        obstacle_avoiding = nil
        aircraft_avoiding = nil

        -- we want to calculate avoidance towards the current NAVIGATION TARGET (navigation_target_loc) - coping to target_loc to avoid changing the copy/pasted code
        if navigation_target_loc == nil then
            gcs:send_text(MAV_SEVERITY.ERROR, " AVOIDING: NO TARGET ")
            return
        end
        local target_loc = navigation_target_loc:copy()

        --gcs:send_text(0, "got current lookahead")

        -- local bearing_cd = math.deg(current_loc:get_bearing(target_loc))*100
        local bearing_deg       = math.deg(current_loc:get_bearing(target_loc))
        local best_bearing_deg  = bearing_deg
        local best_distance_m   = -FLT_MAX

        -- get the current ground course
        -- gcs:send_text(0, string.format("bearing_deg : %.2f",bearing_deg ))

        --local distance_to_target_m = current_loc:get_distance(target_loc)
        local distance_to_target_m = limit_distance(current_loc, target_loc, bearing_deg)
        -- If the full distance is less than 20m, no avoidance is needed
        if distance_to_target_m < 20 then
            log_detect_result(false, distance_to_target_m, distance_to_target_m, best_bearing_deg, target_loc)
            return nil
        end

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

        -- we need to independenty detect aircraft because even if an aircraft may not be the closest obstacle found by bendy ruler, we may still need to deal with it
        -- in other words, sometimes aircraft have higher priority than any other obstacles
        detect_aircraft()

        if obstacle_avoiding == nil then
            --gcs:send_text(MAV_SEVERITY.ERROR, "NO OBSTACLE")
            log_detect_result(false, obstacle_distance_m, distance_to_target_m, best_bearing_deg, target_loc)
            return nil -- no avoidance required
        else
            --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. "step1 dist: " .. obstacle_avoiding.distance_m .. " src: " .. obstacle_avoiding.label .. " bearing: " .. best_bearing_deg)
        end

        if (now_ms - now_debug_ms) > 2000 then
            --gcs:send_text(MAV_SEVERITY.ERROR, string.format("DETECTED: %s distance: %.0f m", obstacle_avoiding.label, obstacle_avoiding.distance_m))
            now_debug_ms = now_ms
        end

	    distance_to_target_m, best_bearing_deg = resist_bearing_change(bearing_deg, current_lookahead, best_bearing_deg, distance_to_target_m)

        -- calculate the new target location based on the best bearing we found. target_loc is passed for altitude only
        local new_target_loc = location_project(current_loc, best_bearing_deg, distance_to_target_m, target_loc)
        log_detect_result(true, obstacle_distance_m, distance_to_target_m, best_bearing_deg, new_target_loc)
        return new_target_loc
    end

    local function alert_obstacle(alert_target_loc)
        if obstacle_avoiding == nil or alert_target_loc == nil  or obstacle_avoiding.distance_xy > lookahead_param then
            previous_label = ""
            return
        end
        if obstacle_avoiding.label == previous_label then
            -- still near the same object, no need to spam the user
            return
        end

        if (now_ms - now_obstacle_ms) > 5000 then
            gcs:send_named_string("DAA-ALERT", "obstacle")
            gcs:send_named_string("DAA-OBSTCL", obstacle_avoiding.label)
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(" ALERT: %s %.0f m",
                                obstacle_avoiding.label, obstacle_avoiding.distance_xy))
            gcs:send_named_float("DAA-DISTXY", obstacle_avoiding.distance_xy)
            gcs:send_named_float("DAA-DISTZ", obstacle_avoiding.distance_z)
            previous_label  = obstacle_avoiding.label
            now_obstacle_ms = now_ms
        end
    end

    local function NMAC_triggered(nmac_obstacle)
        NMAC_active = true
        NMAC_label  = nmac_obstacle.label
        gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: %s Near Miss: %.0fm/%.0fm ", 
                        nmac_obstacle.label, nmac_obstacle.distance_xy, nmac_obstacle.distance_z ))
        gcs:send_named_string("DAA-NMAC", "aircraft")
    end

    local function NMAC_cleared()
        if not NMAC_active then
            return
        end
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: %s Near Miss CLEAR",
                        NMAC_label ))
        gcs:send_named_string("DAA-NMACOK", "aircraft")
        NMAC_active = false
        NMAC_label  = ""
    end

    local function LoWC_triggered(lowc_obstacle)
        LoWC_active = true
        LoWC_label  = lowc_obstacle.label
        gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: %s Loss of Well Clear: %.0f/%.0f m", 
                    lowc_obstacle.label, lowc_obstacle.distance_xy, lowc_obstacle.distance_z ))
        gcs:send_named_string("DAA-LOWC", "aircraft")
    end

    local function LoWC_cleared()
        if not LoWC_active then
            return
        end
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: %s Well Clear", LoWC_label) )
        gcs:send_named_string("DAA-LOWC", "aircraft")
        LoWC_active = false
        LoWC_label  = ""
    end

    local function notify_aircraft_nearby(aircraft_obstacle)
        gcs:send_named_string("DAA-NEARBY", aircraft_obstacle.label)
        gcs:send_named_float("DAA-DISTXY", aircraft_obstacle.distance_xy)
        gcs:send_named_float("DAA-DISTZ", aircraft_obstacle.distance_z)
    end

    local function alert_aircraft()
        if aircraft_avoiding == nil then
            NMAC_cleared()
            LoWC_cleared()
            return
        end
        if (now_ms - now_aircraft_ms) > 5000 then
            --gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: xy %.0f z %.0f", aircraft_avoiding.distance_xy,aircraft_avoiding.distance_z) )
            if aircraft_avoiding.distance_xy < near_miss_xy and aircraft_avoiding.distance_z < near_miss_z then
                NMAC_triggered(aircraft_avoiding)
            elseif aircraft_avoiding.distance_xy < well_clear_xy and aircraft_avoiding.distance_z < well_clear_z then
                LoWC_triggered(aircraft_avoiding)
            else
                gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" ALERT AIRCRAFT: %s %.0f m", 
                                aircraft_avoiding.label, aircraft_avoiding.distance_xy ))
                gcs:send_named_string("DAA-ALERT", "aircraft")
            end
            notify_aircraft_nearby(aircraft_avoiding)
            now_aircraft_ms = now_ms
        end
    end

    -- alert the pilot about any obstacles found, alert_target_loc is the suggested new target location (if applicable)
    function DAA.alert(alert_target_loc)
        if daa_alert == 0 then
            return  -- parameter DAA_ALERT can be used to turn off alerting
        end
        alert_obstacle(alert_target_loc)
        alert_aircraft()
    end

    -- a wrapper around vehicle:update_target_location()
    local function update_target_location(new_target_loc)
        if new_target_loc == nil or update_target_location_save_loc == nil then
            --gcs:send_text(MAV_SEVERITY.ERROR, "AVOID: UPDATE FAILED nil")
            return false
        end
        new_target_loc:change_alt_frame(update_target_location_save_loc:get_alt_frame())
        local updated_location = vehicle:update_target_location(update_target_location_save_loc, new_target_loc)
        if updated_location then
            return true
        end
        --gcs:send_text(MAV_SEVERITY.ERROR, "AVOID: UPDATE FAILED")
        return false
    end

    local function set_avoid_location(new_target_loc)
        -- if the "new" target is nil then revert back to the original navigation target which should be the current waypoint
        if new_target_loc == nil then
            if update_target_location(navigation_target_loc) then
                daa_target_loc = nil
                --gcs:send_text(MAV_SEVERITY.WARNING, "AVOID: REVERT target to navigation target")
            end
            return false
        end
        -- if the "new" target is different from the current DAA target then lets try to go there
        if not locations_equal(daa_target_loc, new_target_loc) then
            if update_target_location(new_target_loc) then
                daa_target_loc = new_target_loc:copy()
                --gcs:send_text(MAV_SEVERITY.WARNING, "AVOID: AVOID set new target")
                return true
            end
        end
        return false
    end

    local function avoid_obstacle(new_target_loc, obstacle)
        if obstacle == nil then -- no obstacle, so clear any specific avoidance we might have been doing
            if current_state == STATE.loitering  and false then
                if loiteralt.stop(false) then
                    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt.stop NO OBSTACLE" ))
                    current_state = STATE.monitoring
                end
            end
            -- reset the target back to the original target
            new_target_loc = nil
        elseif obstacle.type == OBSTACLE_TYPE.GENERAL_AVIATION and false then
            -- depending on the obstacle we might do different things. Specifically if the obstacle is a crude aircraft
            -- in Canada we want to do a "Right 2" circuit descending to XXX altitude
            -- which for now we are doing by simply doing a loiter to alt in guided mode

            -- we might already be doing a loiter because of this aircraft. As long as it's far enough away, thats all we need to do
            if obstacle == aircraft_avoiding then
                return
            end

            loiteralt.start(ga_avoid_alt, ga_avoid_alt_frame, true, airspeed_ms)
            current_state = STATE.loitering

            gcs:send_named_string("DAA-AVOID", "loiter")
            gcs:send_named_string("DAA-ARCRFT", aircraft_avoiding.label)
            gcs:send_named_float("DAA-LOITER", ga_avoid_alt)

            return
        end
        -- if we have a new target - update it if it's different from our current target otherwise revert to the original target
        if obstacle ~= nil and new_target_loc ~= nil then
            --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(" AVOIDING: %s lat %.0f lng %.0f", obstacle.label, new_target_loc:lat(), new_target_loc:lng()))
        end
        if set_avoid_location(new_target_loc) and navigation_target_loc ~= nil then
            local avoid_dist = navigation_target_loc:get_distance(new_target_loc)
            if (now_ms - now_avoiding_ms) > 5000 and obstacle ~= nil and avoid_dist > 5 then
                local obstacle_distance = obstacle.distance_m
                if obstacle.location ~= nil then
                    avoid_dist = navigation_target_loc:get_distance(obstacle.location)
                end

                gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(" AVOIDING: %s distance %.0f m", obstacle.label, math.abs(obstacle_distance)))
                avoiding_label = obstacle.label
                gcs:send_named_string("DAA-AVOID", "obstacle")
                gcs:send_named_string("DAA-OBSTCL", avoiding_label)
                gcs:send_named_float("DAA-DIST", avoid_dist)
                now_avoiding_ms = now_ms
                current_state = STATE.avoiding
            end
        else
            if avoiding_label ~= "" then
                gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(" AVOIDING: %s done", avoiding_label))
                gcs:send_named_string("DAA-AVOID", "")
                gcs:send_named_string("DAA-OBSTCL", "")
                avoiding_label = ""
                current_state = STATE.monitoring
            end
        end
        log_avoid(obstacle, daa_target_loc)
    end

    local function do_loitering()
        if aircraft_avoiding == nil or (current_loc:get_distance(aircraft_avoiding.location) > margin_aircraft) then
            if loiteralt.stop(false) then
                -- gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": loiteralt.stop NO aircraft" ))
                current_state = STATE.monitoring
                return
            end
        end
        if aircraft_avoiding ~= nil then
            loiteralt.aircraft_seen()
            if (now_ms - now_loitering_ms) > 5000 then
                gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(" LOITERING to %.0f m for AIRCRAFT: %s", ga_avoid_alt, aircraft_avoiding.label))
                now_loitering_ms = now_ms
            end
        end
        loiteralt.update()
    end

    -- execute avoidance maneuvers depending on the nature of the obstacle
    function DAA.avoid(new_target_loc)
        if daa_action == 0 then
            return              -- parameter DAA_AVOID can be used to disable avoidance
        end
        if current_state == STATE.loitering then
            do_loitering()
        elseif current_state == STATE.hovering or current_state == STATE.avoiding or current_state == STATE.hovering or current_state == STATE.landing then
            -- do nothing for now
        elseif aircraft_avoiding ~= nil then
            gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(" LOITER AIRCRAFT: %s", aircraft_avoiding.label))
            loiteralt.start(ga_avoid_alt, ga_avoid_alt_frame, true, airspeed_ms)
            current_state = STATE.loitering

            gcs:send_named_string("DAA-AVOID", "LOITER")
            gcs:send_named_float("DAA-LOITER", ga_avoid_alt)
            gcs:send_named_string("DAA-ARCRFT", aircraft_avoiding.label)
            gcs:send_named_float("DAA-DIST", aircraft_avoiding.distance_m)

            return
        else
            current_state = STATE.monitoring
        end
        avoid_obstacle(new_target_loc, obstacle_avoiding)
    end
end) () -- DAA management class

-------------------------------------------------------------------------------
--- Main script execution and update loop including RC on/off management
-------------------------------------------------------------------------------
local last_switch_state = 0
local no_DAA_displayed  = false

local function update()
    get_vehicle_state()


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
        elseif switch_state >= 1 then -- switch High to turn off
            DAA.disable()
        end
        last_switch_state = switch_state
    end

    DAA.get_vehicle_state()
    if DAA.isactive() then
        local suggested_target_loc = DAA.detect()
        DAA.alert(suggested_target_loc)
        -- currently we only do avoidance in FW mode
        if vtol_state == MAV_VTOL_STATE.FW then
            DAA.avoid(suggested_target_loc)
        end
    end
end

-- wrapper around update(). This calls update() at REFRESHRATE Hz, i.e. every 1000/REFRESH_RATE milliseconds
-- and if update faults then an error is displayed, but the script is not stopped
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

-- wait a bit for AP to come up cleanly then start running update loop, unless armed 
if arming:is_armed() then
    return Delayed_Startup()
else
    return Delayed_Startup, 1000 * STARTUP_DELAY
end


--[[

  Calculates Modified Tau (tau_mod) for DAA logic.
  r: Current horizontal range (e.g., in feet)
  r_dot: Horizontal range rate (e.g., in feet per second). 
         Note: r_dot must be negative for aircraft that are closing.
  dmod: Distance Modification threshold (e.g., 4000 or 2000 feet)

function calculate_tau_mod(r, r_dot, dmod)
    -- If range rate is zero or positive, aircraft are not closing.
    -- Modified Tau is mathematically undefined or infinite (safe).
    if r_dot >= 0 then
        return math.huge
    end

    -- If range is already within the DMOD buffer, tau_mod is 0.
    if r <= dmod then
        return 0
    end

    -- RTCA DO-365C Modified Tau Formula:
    -- tau_mod = -(r^2 - dmod^2) / (r * r_dot)
    local tau_mod = -(math.pow(r, 2) - math.pow(dmod, 2)) / (r * r_dot)
    
    return tau_mod
end

-- Example Usage (En Route Scenario):
local current_range = 15000 -- 15,000 feet away
local closure_rate = -150   -- Closing at 150 feet per second
local dmod_enroute = 4000   -- 4,000 feet threshold

local result = calculate_tau_mod(current_range, closure_rate, dmod_enroute)

print(string.format("Modified Tau: %.2f seconds", result))


-- DO-365C DAA Alerting Script for ArduPilot
-- Thresholds for "Warning" alert (25s) and "Corrective" alert (55s)
local TAU_WARNING    = 25
local TAU_CORRECTIVE = 55
local DMOD_FEET      = 4000 -- En-Route DMOD
local H_THRESHOLD    = 450  -- Vertical threshold in feet

local FEET_TO_METERS = 0.3048
local METERS_TO_FEET = 3.28084

function update()
    -- Get UAS altitude (meters to feet)
    local my_pos = ahrs:get_location()
    if not my_pos then return update, 1000 end
    local my_alt_ft = my_pos:alt() * 0.01 * METERS_TO_FEET 

    -- Iterate through all ADSB intruders
    local num_vehicles = adsb:get_num_vehicles()
    for i = 0, num_vehicles - 1 do
        local vehicle = adsb:get_vehicle_info(i)
        
        if vehicle then
            -- 1. Get Geometry
            local r_meters = vehicle:get_distance() -- Horizontal distance
            local r_ft = r_meters * METERS_TO_FEET
            local r_dot = vehicle:get_horiz_velocity() -- Relative horizontal speed
            
            -- In ArduPilot, horiz_velocity is often positive for closure
            -- but for Tau formula we need it negative for closure.
            -- We assume the library returns relative closure speed.
            local r_dot_ft = r_dot * METERS_TO_FEET
            
            -- 2. Vertical Separation Check
            local int_alt_ft = vehicle:get_altitude() * METERS_TO_FEET
            local dh = math.abs(my_alt_ft - int_alt_ft)

            -- 3. Calculate Modified Tau
            local tau_mod = 1000 -- Safe default
            
            if r_ft <= DMOD_FEET then
                tau_mod = 0 -- Inside the 'hockey puck'
            elseif r_dot > 0 then
                -- Formula: tau_mod = -(r^2 - dmod^2) / (r * r_dot)
                -- We use r_dot as positive closure rate here to avoid double negative
                tau_mod = (math.pow(r_ft, 2) - math.pow(DMOD_FEET, 2)) / (r_ft * r_dot_ft)
            else
                tau_mod = 1000 -- Moving away
            end

            -- 4. Decision Logic (Vertical AND Horizontal/Temporal threat)
            if dh < H_THRESHOLD then
                if tau_mod <= TAU_WARNING then
                    gcs:send_text(0, string.format("DAA WARNING: Int %s at %.0fs", vehicle:callsign(), tau_mod))
                    -- Optional: Trigger automated avoidance here
                elseif tau_mod <= TAU_CORRECTIVE then
                    gcs:send_text(6, string.format("DAA Corrective: Int %s at %.0fs", vehicle:callsign(), tau_mod))
                end
            end
        end
    end

    return update, 500 -- Run at 2Hz
end

gcs:send_text(6, "DAA DO-365C Script Loaded")
return update, 1000

--]]
