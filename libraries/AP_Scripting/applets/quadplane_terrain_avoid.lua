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

 Terrain Avoidance in QuadPlane. 

 This code will detect if a quadplane following an Auto mission is likely to hit elevated terrain, such as
 a small hill, cliff edge, high trees or other obstacles that might not show up in the OOTB STRM terrain model.
 The code will attempt to avoid the impact by:-
 - Pitching up if the plane can safely fly over the obstacle
 - Otherwise switching to QuadPlane Qloiter mode (Quading) and gaining altitude using VTOL motors 
 This code requires long range rangefinders such as the LightWare long range lidars that can measure 
   distances up to 90-95 meters away.
 The terrain avoidance will be on by default but will not function at "home" or within ZTA_HOME_DIST meters
 of home. The scripting function ZTA_ACT_FN can be used to disable terrain folling at any time
 Terrain following will operate in modes Auto, Gukded, RTL and QRTL.

The "Can't make that climb" (CMTC) feature will prevent ArduPlane from flying into terrain it does know about
by calculating the required pitch to avoid terrain between the current location and the next waypoint including
all points in between. If the pitch required is > PTCH_TRIM_MAX_DEG / 2 then the code will perform a loiter to
altitude to acheive a safe AMSL altitude to avoid the terrain before continuing the mission.

--]]

SCRIPT_NAME = "OvrhdIntl Terrain Avoid"
SCRIPT_NAME_SHORT = "TerrAvoid"
SCRIPT_VERSION = "4.6.0-001"

REFRESH_RATE = 0.1  -- in seconds, so 10Hz
STARTUP_DELAY = 20  -- wait this many seconds for the FC to come up before starting the script

FLIGHT_MODE = {AUTO=10, RTL=11, LOITER=12, GUIDED=15, QHOVER=18, QLOITER=19, QRTL=21}

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
MAV_FRAME = { GLOBAL = 0, GLOBAL_RELATIVE_ALT = 3}
MAV_CMD_INT = { DO_SET_MODE = 176, DO_CHANGE_SPEED = 178, DO_REPOSITION = 192, 
                  GUIDED_CHANGE_SPEED = 43000, GUIDED_CHANGE_ALTITUDE = 43001, GUIDED_CHANGE_HEADING = 43002 }
MAV_SPEED_TYPE = { AIRSPEED = 0, GROUNDSPEED = 1, CLIMB_SPEED = 2, DESCENT_SPEED = 3 }
MAV_HEADING_TYPE = { COG = 0, HEADING = 1} -- COG = Course over Ground, i.e. where you want to go, HEADING = which way the vehicle points 

RANGEFINDER_ORIENT = {DOWNWARD = 25, FORWARD = 0}
RANGEFINDER_STATUS = {NOTCONNECTED = 0, NODATA = 1, OUTOFRANGELOW = 2, OUTOFRANGEHIGH = 3, GOOD = 4}

local rangefinder_down_value = 0.0
local rangefinder_forward_value = 0.0
MAX_RANGEFINDER_VALUE = 90

local PARAM_TABLE_KEY = 99
local PARAM_TABLE_PREFIX = "ZTA_"
local PARAM2_TABLE_KEY = 100
local PARAM2_TABLE_PREFIX = "ZTB_"

-- bind a parameter to a variable
function bind_param(name)
    return Parameter(name)
end
 
-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), SCRIPT_NAME_SHORT .. string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end
-- add a parameter and bind it to a variable
function bind_add_param2(name, idx, default_value)
    assert(param:add_param(PARAM2_TABLE_KEY, idx, name, default_value), SCRIPT_NAME_SHORT .. string.format('could not add param %s', name))
    return bind_param(PARAM2_TABLE_PREFIX .. name)
end

-- setup follow mode specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), SCRIPT_NAME_SHORT .. 'could not add param table: ' .. PARAM_TABLE_PREFIX)
assert(param:add_table(PARAM2_TABLE_KEY, PARAM2_TABLE_PREFIX, 10), SCRIPT_NAME_SHORT .. 'could not add param table: ' .. PARAM2_TABLE_PREFIX)

--[[
    // @Param: ZTA_ACT_FN
    // @DisplayName: Terrain Avoidance Activation Function
    // @Description: Setting an RC channel's _OPTION to this value will use it for Terrain Avoidance enable/disable
    // @Range: 300 307
--]]
ZTA_ACT_FN = bind_add_param("ACT_FN", 1, 305)

--[[
    // @Param: ZTA_PTCH_DWN_MIN
    // @DisplayName: Terrain Avoidance minimum down distance for Pitching
    // @Description: If the downward distance is less than this value then start Pitching up to gain altitude.
    // @Units: m
    // @Default: 46
--]]
ZTA_PTCH_DWN_MIN = bind_add_param("PTCH_DWN_MIN", 2, 46)

--[[
    // @Param: ZTA_PTCH_FWD_MIN
    // @DisplayName: Terrain Avoidance minimum forward distance for Pitching
    // @Description: If the farwardward distance is less than this value then start Pitching up to gain altitude.
    // @Units: m
    // @Default: 80
--]]
ZTA_PTCH_FWD_MIN = bind_add_param("PTCH_FWD_MIN", 3, 80)

--[[
    // @Param: ZTA_QUAD_DWN_MIN
    // @DisplayName: Terrain Avoidance minimum down distance for Quading
    // @Description: If the downward distance is less than this value then start Quading up to gain altitude.
    // @Units: m
    // @Default: 46
--]]
ZTA_QUAD_DWN_MIN = bind_add_param("QUAD_DWN_MIN", 4, 35)

--[[
    // @Param: ZTA_QUAD_FWD_MIN
    // @DisplayName: Terrain Avoidance minimum forward distance for Quading
    // @Description: If the farwardward distance is less than this value then start Quading up to gain altitude.
    // @Units: m
    // @Default: 20
--]]
ZTA_QUAD_FWD_MIN = bind_add_param("QUAD_FWD_MIN", 5, 20)

--[[
    // @Param: ZTA_gitlog_MIN
    // @DisplayName: Terrain Avoidance minimum ground speed for Pitching
    // @Description: Minimum Groundspeed (not airspeed) to be flying for Pitching to be used.
    // @Units: m/s
    // @Default: 17
--]]
ZTA_PTCH_GSP_MIN = bind_add_param("PTCH_GSP_MIN", 6, 17)

--[[
    // @Param: ZTA_PTCH_TIMEOUT
    // @DisplayName: Terrain Avoidance timeout Pitching
    // @Description: Minimum down or forward distance must be triggered for more than this many seconds to start Pitching
    // @Units: sec
    // @Default: 2
--]]
ZTA_PTCH_TIMEOUT = bind_add_param("PTCH_TIMEOUT", 7, 2)

--[[
    // @Param: ZTA_HOME_DIST
    // @DisplayName: Terrain Avoidance safe distance around home
    // @Description: Terrain avoidance will not be applied if the vehicle is less than this distance from home
    // @Units: m
    // @Default: 20
--]]
ZTA_HOME_DIST = bind_add_param("HOME_DIST", 8, 20)

--[[
    // @Param: ZTA_SIM_DWN_FN
    // @DisplayName: Terrain Avoidance Sim Down Function
    // @Description: Setting an RC channel's _OPTION to this value will use it to generate a simulated "down" rangefinder value based on the PWM value of the channel
    // @Range: 300 307
--
ZTA_SIM_DWN_FN = bind_add_param("SIM_DWN_FN", 9, 306)

--[[
    // @Param: ZTA_SIM_FWD_FN
    // @DisplayName: Terrain Avoidance Sim Forward Function
    // @Description: Setting an RC channel's _OPTION to this value will use it to generate a simulated "forward" rangefinder value based on the PWM value of the channel
    // @Range: 300 307
--
ZTA_SIM_FWD_FN = bind_add_param("SIM_FWD_FN", 10, 307)
--]]

--[[
    // @Param: ZTA_ALT_MAX
    // @DisplayName: Terrain Avoidance Altitude ceiling for pitching/quading
    // @Description: This is a limit on how high the terrain avoidane will take the vehicle. It acts a failsafe to prevent vertical flyaways.
    // @Range: 20 1000
    // @Units: m
    // @Default: 100
--]]
ZTA_ALT_MAX = bind_add_param("ALT_MAX", 9, 100)

--[[
    // @Param: ZTB_GSP_MAX
    // @DisplayName: Maximum Groundspeed
    // @Description: This is a limit on how fast in groundspeeed terrain avoidance will take the vehicle. This is to allow for reliable sensor readings.
    // @Range: 10 40
    // @Units: m/s
    // @Default: -1 (disabed)
--]]
ZTB_GSP_MAX = bind_add_param2("GSP_MAX", 1, -1)

--[[
    // @Param: ZTB_GSP_AIRBRAKE
    // @DisplayName: Groudspeed Airbrake limt
    // @Description: This is the limit for triggering airbrake to slow groundspeed as a difference between the airspeed and groundspeed
    // @Range: -1 -10
    // @Units: m/s
    // @Default: 0 (disabed)
--]]

ZTB_GSP_AIRBRAKE = bind_add_param2("GSP_AIRBRAKE", 2, 0)

local pitch_groundspeed_min = ZTA_PTCH_GSP_MIN:get()
local pitch_down_min = ZTA_PTCH_DWN_MIN:get()
local pitch_forward_min = ZTA_PTCH_FWD_MIN:get()
local pitch_timeout = ZTA_PTCH_TIMEOUT:get()
local home_distance_max = ZTA_HOME_DIST:get()

local quad_down_min = ZTA_QUAD_DWN_MIN:get()
local quad_forward_min = ZTA_QUAD_FWD_MIN:get()

local altitude_max = ZTA_ALT_MAX:get()

local groundspeed_max = ZTB_GSP_MAX:get() or -1
local groundspeed_airbrake_limit = ZTB_GSP_AIRBRAKE:get() or 0

MIN_ALT_MAX = 20

Q_ENABLE = bind_param('Q_ENABLE')
Q_WVANE_ENABLE = bind_param('Q_WVANE_ENABLE')
THR_MAX = bind_param('THR_MAX')
AIRSPEED_CRUISE = bind_param('AIRSPEED_CRUISE')
AIRSPEED_MIN = bind_param("AIRSPEED_MIN")
AIRSPEED_MAX = bind_param("AIRSPEED_MAX")
WP_LOITER_RAD = bind_param("WP_LOITER_RAD")
TERRAIN_ENABLE = bind_param("TERRAIN_ENABLE")
TERRAIN_SPACING = bind_param("TERRAIN_SPACING")
PTCH_LIM_MAX_DEG = bind_param("PTCH_LIM_MAX_DEG")
TECS_CLMB_MAX = bind_param("TECS_CLMB_MAX")
local airspeed_cruise = AIRSPEED_CRUISE:get() or 18
local airspeed_min = AIRSPEED_MIN:get() or 10
local airspeed_max = AIRSPEED_MAX:get() or 30
local wp_loiter_rad = WP_LOITER_RAD:get() or 150
local q_enable = Q_ENABLE:get() or 0
local terrain_enable = TERRAIN_ENABLE:get() or 0
local terrain_spacing = TERRAIN_SPACING:get() or 100
local ptch_lim_max_deg = PTCH_LIM_MAX_DEG:get() or 25
local tecs_climb_max = TECS_CLMB_MAX:get() or 5.0

RCMAP_THROTTLE = bind_param('RCMAP_THROTTLE')
THROTTLE_CHANNEL = rc:get_channel(RCMAP_THROTTLE:get()) -- The RC channel used for throttle

PITCH_TOLERANCE = 1.1

local vehicle_mode = vehicle:get_mode()

local current_location = ahrs:get_location()
local current_bearing = 0.0
local previous_location
local current_altitude = 0.0
local current_location_target
local new_location_target = Location()
local current_mission_item
local terrain_altitude = terrain:height_above_terrain(true)
local terrain_max_exceeded = false
local groundspeed_vector = ahrs:groundspeed_vector()
local ground_speed = groundspeed_vector:length()

local now = millis():tofloat() * 0.001
local pitch_last_good_timestamp = now
local pitch_last_bad_timestamp = now
local pitch_bad_timer = -10

local pre_pitch_mode = vehicle_mode
local pre_pitch_target_altitude= 0.0
local pre_pitch_target_location

local quading_active = false
local pitching_active = false
local slowdown_quading = false
local cmtc_active = false

local q_wvane_enable_save = Q_WVANE_ENABLE:get()
local avoid_enter_mode = -1
--local quading_enter_distance = -1
--local quading_enter_location

local cmtc_target_alt_amsl = -1

-- function forward declarations
local start_quading -- foward declaration of start_quading/stop_quading defined below
local stop_quading
local avoid_terrain
local terrain_approaching
local pitch_obstacle_detected
local pitch_obstacle_down
local pitch_obstacle_forward

local speedpi = require("speedpi")
local speed_PI = speedpi.speed_controller(0.1, 0.1, 2.5, airspeed_min, airspeed_max)

local mavlink = require("mavlink_cmdint")

local location_tracker

-- constrain a value between limits
local function constrain(v, vmin, vmax)
    if v < vmin then
       v = vmin
    end
    if v > vmax then
       v = vmax
    end
    return v
end

local function set_avoid_mode(new_mode)
    avoid_enter_mode = vehicle_mode
    vehicle:set_mode(new_mode)
end

local function reset_avoid_mode()
    local new_mode = FLIGHT_MODE.AUTO
    if avoid_enter_mode == FLIGHT_MODE.AUTO or avoid_enter_mode == FLIGHT_MODE.GUIDED or avoid_enter_mode == FLIGHT_MODE.LOITER or
        avoid_enter_mode == FLIGHT_MODE.QHOVER or avoid_enter_mode == FLIGHT_MODE.QLOITER or avoid_enter_mode == FLIGHT_MODE.QRTL or
        avoid_enter_mode == FLIGHT_MODE.RTL then
       new_mode= avoid_enter_mode
    end
    vehicle:set_mode(new_mode)
    avoid_enter_mode = -1

    if new_mode == FLIGHT_MODE.AUTO then
        previous_location = location_tracker.get_saved_location()
        if previous_location ~= nil then
            vehicle:set_crosstrack_start(previous_location)
        end
    end
end
--[[
local function PI_controller(kP,kI,iMax,min,max)
    -- the new instance. You can put public variables inside this self
    -- declaration if you want to
    local self = {}
 
    -- private fields as locals
    local _kP = kP or 0.0
    local _kI = kI or 0.0
    local _iMax = iMax
    local _min = min
    local _max = max
    local _last_t = nil
    local _I = 0
    local _P = 0
    local _total = 0
    local _counter = 0
    local _target = 0
    local _current = 0
    local nowPI = millis():tofloat() * 0.001
 
    -- update the controller.
    function self.update(target, current)
       local now = millis():tofloat() * 0.001
       if not _last_t then
          _last_t = now
       end
       local dt = now - _last_t
       _last_t = now
       local err = target - current
       _counter = _counter + 1
 
       local P = _kP * err
       if ((_total < _max and _total > _min) or (_total >= _max and err < 0) or (_total <= _min and err > 0)) then
          _I = _I + _kI * err * dt
       end
       if _iMax then
        _I = constrain(_I, -_iMax, iMax)
       end
       local I = _I
       local ret = target + P + I
       if math.floor(now) ~= math.floor(nowPI) then
           --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": target %f err %f P %f I %f ret %f", target,err, P, I, ret) )
           nowPI = millis():tofloat() * 0.001
       end
       _target = target
       _current = current
       _P = P
 
       ret = constrain(ret, _min, _max)
       _total = ret
       return ret
    end
 
    -- reset integrator to an initial value
    function self.reset(integrator)
       _I = integrator
    end
 
    function self.set_I(I)
       _kI = I
    end

    function self.set_P(P)
       _kP = P
    end

    function self.set_Imax(Imax)
       _iMax = Imax
    end

    -- log the controller internals
    function self.log(name, add_total)        
       -- allow for an external addition to total
       logger.write(name,'Targ,Curr,P,I,Total,Add','ffffff',_target,_current,_P,_I,_total,add_total)
    end

    -- return the instance
    return self
end

local function speed_controller(kP_param, kI_param, iMax)
    local self = {}
    local PI = PI_controller(kP_param, kI_param, iMax, airspeed_min, airspeed_max)
 
    function self.update(spd_current, spd_error)
       local adjustment = PI.update(spd_current + spd_error, spd_current)
       -- PI.log("AESA", 0)
       return adjustment
    end
 
    function self.reset()
       PI.reset(0)
    end
 
    return self
end
--]]


--local previous_speed = -1.0

--[[local function set_vehicle_speed(speed)
    local new_speed = speed.speed or -2.0   -- special airspeed value meaning "default"
    local speed_type = speed.type or MAV_SPEED_TYPE.AIRSPEED
    local throttle = speed.throttle or 0.0
    local slew = speed.slew or 0.0

    -- Ignore a speed difference of 1 m/s kind of arbitrary but easy to understand
    if math.floor(new_speed) == math.floor(previous_speed) then
        return
    end
    previous_speed = new_speed
    if speed_type == MAV_SPEED_TYPE.AIRSPEED and speed.slew ~= 0 and (vehicle_mode == FLIGHT_MODE.GUIDED) then
       if not gcs:run_command_int(MAV_CMD_INT.GUIDED_CHANGE_SPEED, { frame = MAV_FRAME.GLOBAL,
                                  p1 = speed_type,
                                  p2 = new_speed,
                                  p3 = slew }) then 
          gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": MAVLink GUIDED_CHANGE_SPEED returned false")
       end
    else
       if not gcs:run_command_int(MAV_CMD_INT.DO_CHANGE_SPEED, { frame = MAV_FRAME.GLOBAL,
                                  p1 = speed_type,
                                  p2 = new_speed,
                                  p3 = throttle }) then
          gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": MAVLink DO_CHANGE_SPEED returned false")
       end
    end
end
--]]

local function disable_wvane()
    q_wvane_enable_save = Q_WVANE_ENABLE:get()
    Q_WVANE_ENABLE:set(0)
end

local function enable_wvane()
    if q_wvane_enable_save >=0 then
        Q_WVANE_ENABLE:set(q_wvane_enable_save)
    else
        Q_WVANE_ENABLE:set(0)
    end
    q_wvane_enable_save = -1
end

--local now_slowdown = millis():tofloat() * 0.001
--local now_airbrake = millis():tofloat() * 0.001

local airbrake_trigger = false
local airbrake_on = false
local airbrake_triggered_now = millis():tofloat() * 0.001

-- This method terminates airbraking if it had been acive
local function slowdown_airbrake_end()
    airbrake_trigger = false
    if not airbrake_on then
        return
    end
    airbrake_on = false
    enable_wvane()
    gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": airbraking stopping")
    if vehicle_mode ~= FLIGHT_MODE.QLOITER then
        -- user or a failesafe has exitied QLoiter, so we don't want to mess with that
        return
    end
    now_slowdown = millis():tofloat() * 0.001
    reset_avoid_mode()
end

local function slow_down(groundspeed)
    local groundspeed_error = groundspeed.error or 0.0
    local airspeed_current = ahrs:airspeed_estimate()
    local airspeed_new = speed_PI.update(airspeed_current, groundspeed_error)
 
    airspeed_new = airspeed_current + groundspeed_error
    airspeed_new = constrain(airspeed_new, airspeed_min, airspeed_max)

    --[[
    if math.floor(now) ~= math.floor(now_slowdown) then 
        if groundspeed_error < -2 then
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": slow down current %f error %f new %f", airspeed_current,groundspeed_error,airspeed_new) )
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT)
        end
        now_slowdown = millis():tofloat() * 0.001
    end
    --]]

    if groundspeed_airbrake_limit ~= 0 and not airbrake_on and groundspeed_error < groundspeed_airbrake_limit then
        if not airbrake_trigger then
            airbrake_trigger = true
            airbrake_triggered_now = millis():tofloat() * 0.001
        end

        if (now - airbrake_triggered_now) > pitch_timeout then
            airbrake_on = true
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": airbrake current %f error %f new %f", airspeed_current,groundspeed_error,airspeed_new) )
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": airbraking Starting")
            disable_wvane()
            set_avoid_mode(FLIGHT_MODE.QLOITER)
        end
    elseif airbrake_trigger then
        airbrake_trigger = false
    end
    if airbrake_on then
        if groundspeed_error >= 0 or (now - airbrake_triggered_now) > 10 then -- it's not working, give up
            slowdown_airbrake_end()
        else
            -- we want to hover in place
            THROTTLE_CHANNEL:set_override(1500)
        end
    else
        --set_vehicle_speed({speed=airspeed_new})
        mavlink.set_vehicle_speed({speed=airspeed_new})
    end
end

local function set_altitude_target(new_altitude)
    local current_location_target = vehicle:get_target_location()
    if current_location_target ~= nil then
        new_location_target = current_location_target:copy()
        if new_location_target ~= nil and new_altitude ~= nil then
            local target_alt_cm = math.floor(math.min(new_altitude * 100, 319000))
            new_location_target:alt(target_alt_cm)
            -- can't use MAVLink for this because we might not be in GUIDED mode
--            mavlink.set_vehicle_target_altitude({alt = new_altitude, frame = mavlink.ALT_FRAME.ABOVE_TERRAIN})
            -- gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": set alt(cm): " .. target_alt_cm .. " frame o: ".. current_location_target:get_alt_frame() .. " frame n: " .. new_location_target:get_alt_frame())
            if not vehicle:update_target_location(current_location_target,new_location_target) then
                gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. 
                    string.format(": failed to set alt(cm): %.0f frame: %d current frame: %d", target_alt_cm, new_location_target:get_alt_frame(), current_location_target:get_alt_frame()))
            end
        else
            gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": failed to set alt: " .. new_altitude)
        end
    end
    return new_altitude
end

-- This method forces the plane to target a very high altitude to force it to gain altitude quicky
local function set_altitude_high()
    local target_altitude = current_altitude + 100
    local old_target_altitude = 0.0
    --current_location_target = vehicle:get_target_location()
    if current_location_target == nil and current_location ~= nil then
        old_target_altitude = current_location:alt() * 0.01
    elseif current_location_target ~= nil then
        old_target_altitude = current_location_target:alt() * 0.01
    end
    set_altitude_target(target_altitude)
    return old_target_altitude
end

-- Attempts to duplicate the code that updates the prev_WP_loc variable in the c++ code
local function LocationTracker()

    local self = {}

    -- to get this to work, need to keep 2 prior generations of "target_location"
    local target_location
    local previous_target_location          -- the target prior to the current one
    local previous_previous_target_location -- the target prior to that - this is the one we want

    function self.same_loc_as(A, B)
       if A == nil or B == nil then
          return false
       end
       if (A:lat() ~= B:lat()) or (A:lng() ~= B:lng()) then
          return false
       end
       return (A:alt() == B:alt()) and (A:get_alt_frame() == B:get_alt_frame())
    end

    function self.update()
       target_location = vehicle:get_target_location()
       if target_location ~= nil then
          if not self.same_loc_as(previous_target_location, target_location) then
            -- maintain three generations of location
            if previous_target_location ~= nil then
                previous_previous_target_location = previous_target_location:copy()
            end
            previous_target_location = target_location:copy()
            --gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..
            --    string.format(": saved loc lat %.1f lng %.1f alt %.1f", target_location:lat(), target_location:lng(), target_location:alt()))
            end
       else
          previous_target_location = ahrs:get_location()
          previous_previous_target_location = previous_target_location
       end
    end

    function self.get_saved_location()
        return previous_previous_target_location
    end

    return self
end

location_tracker = LocationTracker()

local function start_cmtc(target_alt_amsl)
    if cmtc_active then
        gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": CMTC to ALREADY ACTIVE: " .. cmtc_target_alt_amsl)
        return
    end
    cmtc_active = true
    --cmtc_target_location:change_alt_frame(mavlink.ALT_FRAME.ABSOLUTE)
    --cmtc_target_alt_amsl = cmtc_target_location:alt() * 0.01

    local direction = avoid_terrain(target_alt_amsl)

    -- AP libraries use a 0.5m "near enough" buffer for matching altitude
    cmtc_target_alt_amsl = target_alt_amsl - 3.0

    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": CMTC loiter to the %s to %.0f alt", direction, cmtc_target_alt_amsl) )

    --[[local current_location_amsl = current_location:copy()
    current_location_amsl:change_alt_frame(mavlink.ALT_FRAME.ABSOLUTE)

    gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. 
            string.format(": CMTC to: %.0f %s current %.0f frame %d", cmtc_target_alt_amsl, direction, current_location_amsl:alt() * 0.01, current_location_amsl:get_alt_frame()))
    --]]
end

local function stop_cmtc()
    if current_location ~= nil then
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": CMTC Done alt: %.0f", current_location:alt() * 0.01) )
    end
    cmtc_active = false
    cmtc_target_alt_amsl = -1
    reset_avoid_mode()
end

local function do_cmtc()
    if current_location ~= nil then
        local current_location_amsl = current_location:copy()
        current_location_amsl:change_alt_frame(mavlink.ALT_FRAME.ABSOLUTE)
        if current_location_amsl:alt() * 0.01 > cmtc_target_alt_amsl then
            -- gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": update stop_cmtc() altidude: " .. current_location_amsl:alt() * 0.01 )
            stop_cmtc()
        end
    end
end

local pre_pitch_

local function start_pitching()
    if slowdown_quading then
        start_quading() -- we are already in multirotor mode, so go straight to quading
        return
    end

    pitching_active = true
    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": Pitching started")

    -- save the current target location so we can track if we pass a waypoint while pitching
    pre_pitch_mode = vehicle_mode
    pre_pitch_target_altitude = set_altitude_high()
    pre_pitch_target_location = current_location_target:copy()

    -- pitch up by setting a very high altitude and high speed. TECS will make it so.
    local desired_airspeed = 30
    if AIRSPEED_MAX ~= nil then
        desired_airspeed = AIRSPEED_MAX:get()
    end
    --set_vehicle_speed({speed=desired_airspeed})
    mavlink.set_vehicle_speed({speed=desired_airspeed})

    pitch_last_bad_timestamp = millis():tofloat() * 0.001
end

local function check_pitching()
    if ground_speed ~= nil and ground_speed < pitch_groundspeed_min then
        return false
    end

    if pitch_obstacle_detected(1.0) then
        if not pitching_active then
            -- we don't jump into pitching right away, we give it a ZTA_PTCH_TIMEOUT seconds to be sure
            local time_since_good = (now - pitch_last_good_timestamp)
            if time_since_good > pitch_timeout then
                start_pitching()
                if pitch_obstacle_down(1.0) then
                    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Obstacle down: %.2f m", rangefinder_down_value) )
                end
                if pitch_obstacle_forward(1.0) then
                    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Obstacle forward: %.2f m", rangefinder_forward_value) )
                end
            else
                if time_since_good > pitch_bad_timer + 1 then
                    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": high terrain detected")
                    pitch_bad_timer = math.floor(time_since_good + 0.5)
                end
            end
        end
        return true
    else
        if pitch_bad_timer >= 0 and not terrain_max_exceeded then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": terrain Ok")
        end
        pitch_last_good_timestamp = now
        pitch_bad_timer = -1
    end
    return false
end

local function stop_pitching()
    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": Pitching DONE")
    pitching_active = false
    if pre_pitch_mode ~= vehicle_mode then
        return  -- user or maybe a failsafe changed mdoes. Don't interfere
    end
    if pre_pitch_target_altitude ~= nil then
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": reset Alt: " .. pre_pitch_target_altitude)
        set_altitude_target(pre_pitch_target_altitude)
        speed_PI.reset(0)
--        set_vehicle_speed({})   -- special airspeed value meaning "default"
        mavlink.set_vehicle_speed({}) -- special airspeed value meaning "default"
        previous_location = location_tracker.get_saved_location()
        if previous_location ~= nil then
            vehicle:set_crosstrack_start(previous_location)
        end
    else
        -- don't know where to go, so lets go home
        vehicle:set_mode(FLIGHT_MODE.RTL)
    end
end

local function do_pitching()
    -- quading takes precedence over pitching
    if quading_active then
        pitching_active = false
        return
    end

    -- if we are above the pitching altitude (with a margin) we might want to stop pitching
    -- we don't stop pitching right away, we give it a ZTA_PTCH_TIMEOUT seconds * 2 to be sure
    if not pitch_obstacle_detected(PITCH_TOLERANCE) and (now - pitch_last_bad_timestamp) > (pitch_timeout * 2) or
        not (pre_pitch_target_location:lat() == current_location_target:lat() and pre_pitch_target_location:lng() == current_location_target:lng() ) then
        -- we are done, need to restore the waypoint target
        --[[if not pitch_obstacle_detected(PITCH_TOLERANCE) then
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": Stop Pitching from do_pitching not pitch_obstacle_detected")
        end
        if (now - pitch_last_bad_timestamp) > (pitch_timeout * 2) then
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": Stop Pitching from time: " .. (now - pitch_last_bad_timestamp))
        end
        if not (pre_pitch_target_location:lat() == current_location_target:lat() and pre_pitch_target_location:lng() == current_location_target:lng()) then
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": Stop Pitching new WP ")
        end
        --]]
        stop_pitching()
    else
        --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. ": Still Pitching")
        set_altitude_high()
        pitch_last_bad_timestamp = millis():tofloat() * 0.001
    end
end

start_quading = function() -- forward declaration above
    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": Quading started")
    if slowdown_quading then
        slowdown_quading = false -- already in multi-motor mode, so just switch to quading
    else
        set_avoid_mode(FLIGHT_MODE.QLOITER)
        disable_wvane()
    end
    quading_active = true
    pitching_active = false
    THROTTLE_CHANNEL:set_override(2000)
end

local function check_quading()
    if quad_obstacle_detected() then
        if not quading_active then
            if quad_obstacle_down() then
                gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Obstacle down: %.2f m", rangefinder_down_value) )
            end
            if quad_obstacle_forward() then
                gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": Obstacle forward: %.2f m", rangefinder_forward_value) )
            end
            if pitching_active then
                gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": Stop Pitching")
                stop_pitching()
            end
            start_quading()
            return true
        end
    end
    return false
end

stop_quading = function() -- forward declaration above
    if quading_active then
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": Quading DONE")
    end
    quading_active = false
    airbrake_on = false
    speed_PI.reset(0)
    enable_wvane()
    if vehicle_mode ~= FLIGHT_MODE.QLOITER then
        gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": exit Quading NOT QLoiter")
        avoid_enter_mode = 0
        return -- user or failsafe has changed modes. Don't interfere
    end
    reset_avoid_mode()
end

local function do_quading()
    -- not a typo, if we kick into quading we want to go higher than would trigger pitching when when we are done
    if not pitch_obstacle_detected(PITCH_TOLERANCE) or terrain_max_exceeded then
        -- we are done quadding
        stop_quading()
    else
        -- continually override the throttle to keep going up
        THROTTLE_CHANNEL:set_override(2000)
    end
end

-- This function decides if there is an obstacle for pitching based on
-- 1. if terrain height is < pitch_down_min or
-- 2. if there is a valid rangefinder down and rangefinder_down_value < pitch_down_min or
-- 3. if there is a valid rangefinder forward and rangefinder_forward_value < pitch_forward_min
-- 
pitch_obstacle_down = function(multiplier)
    if rangefinder_down_value > 0 and rangefinder_down_value < MAX_RANGEFINDER_VALUE
       and rangefinder_down_value < (pitch_down_min * multiplier) then
        --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. ": Obstacle Down " .. rangefinder_down_value .. " below: " .. (pitch_down_min * multiplier))

        return true
    end
    return false
end
pitch_obstacle_forward = function(multiplier)
    if rangefinder_forward_value > 0 and rangefinder_forward_value < MAX_RANGEFINDER_VALUE 
       and rangefinder_forward_value < (pitch_forward_min * multiplier) then
        return true
    end
    return false
end

pitch_obstacle_detected = function(multiplier)
    -- prevent pitching if we are in a flyaway situation
    if terrain_max_exceeded then
        if pitching_active then
            gcs:send_text(MAV_SEVERITY.CRITICAL, SCRIPT_NAME_SHORT .. ": Pitching " .. terrain_altitude .. " above: " .. altitude_max)
        end
        if quading_active then
            gcs:send_text(MAV_SEVERITY.CRITICAL, SCRIPT_NAME_SHORT .. ": Quading " .. terrain_altitude .. " above: " .. altitude_max)
        end
        if cmtc_active then
            gcs:send_text(MAV_SEVERITY.CRITICAL, SCRIPT_NAME_SHORT .. ": CMTC " .. terrain_altitude .. " above: " .. altitude_max)
        end
        return false
    end

    if pitch_obstacle_down(multiplier) then
        return true
    end
    if pitch_obstacle_forward(multiplier) then
        return true
    end

    -- no obstacle, we are good
    return false
end


-- This function decides if there is an obstacle for quading based on
-- 1. if terrain height is < quad_down_min or
-- 2. if there is a valid rangefinder down and rangefinder_down_value < pitch_quad_min or
-- 3. if there is a valid rangefinder forward and rangefinder_forward_value < quad_forward_min
-- 
function quad_obstacle_forward()
    if rangefinder_forward_value > 0 and rangefinder_forward_value < MAX_RANGEFINDER_VALUE
       and rangefinder_forward_value < quad_forward_min then
        return true
    end
end

function quad_obstacle_down()
    if rangefinder_down_value > 0 and rangefinder_down_value < MAX_RANGEFINDER_VALUE
       and rangefinder_down_value < quad_down_min then
        return true
    end
    return false
end

function quad_obstacle_detected()
    -- prevent quading if we are in a flyaway situation
    if terrain_max_exceeded then
        if quading_active then
            gcs:send_text(MAV_SEVERITY.CRITICAL, SCRIPT_NAME_SHORT .. ": Quading " .. terrain_altitude .. " above: " .. altitude_max)
        end
        return false
    end

    if quad_obstacle_down() then
        return true
    end
    if quad_obstacle_forward() then
        return true
    end

    -- no obstacle, we are good
    return false
end

-- this method checks the distance down and forward. There are no sanity checks (yet)
-- and this uses RC8 to simulate forward rangefinder and RC5 to simulate downward
local function populate_rangefinder_values()
    -- Get the new values of the range finders every update cycle
    -- We'll probably want some kind of certainty check for the range finders
    -- So a small error won't cause it to freakout. 
  
    if rangefinder:has_data_orient(RANGEFINDER_ORIENT.DOWNWARD)
        and rangefinder:status_orient(RANGEFINDER_ORIENT.DOWNWARD) == RANGEFINDER_STATUS.GOOD then
        rangefinder_down_value = rangefinder:distance_cm_orient(RANGEFINDER_ORIENT.DOWNWARD) * 0.01
    else
        -- if we don't have a downward rangefinder revert to terrain altitude
        rangefinder_down_value = terrain:height_above_terrain(true) or 0.0
    end
    if rangefinder:has_data_orient(RANGEFINDER_ORIENT.FORWARD)
        and rangefinder:status_orient(RANGEFINDER_ORIENT.FORWARD) == RANGEFINDER_STATUS.GOOD then
        rangefinder_forward_value = rangefinder:distance_cm_orient(RANGEFINDER_ORIENT.FORWARD) * 0.01
    else
        rangefinder_forward_value = 0.0
    end

    terrain_altitude = terrain:height_above_terrain(true)
    terrain_max_exceeded = false
    if altitude_max ~= nil and terrain_altitude ~= nil then
        terrain_max_exceeded = (altitude_max > MIN_ALT_MAX and terrain_altitude > altitude_max)
    end

    if rangefinder_down_value == nil or rangefinder_down_value <= 0 or 
        rangefinder_down_value > MAX_RANGEFINDER_VALUE then
        rangefinder_down_value = terrain_altitude or 0
    end
    if rangefinder_forward_value == nil or rangefinder_forward_value <= 0 then
        rangefinder_forward_value = 0
    end
    if rangefinder_forward_value > MAX_RANGEFINDER_VALUE then
        rangefinder_forward_value = MAX_RANGEFINDER_VALUE
    end
end

local function wrap_360(angle)
    local res = math.fmod(angle, 360.0)
     if res < 0 then
         res = res + 360.0
     end
     return res
 end

--[[
Some useful terrain functions
terrain:height_amsl(Loc, Height, extrapolate boolean)
terrain:lookahead(bearing, distance, climb_rate) - returns additional height needed but where?
terrain:lookahead_location(Location (result), starting_location, bearing, distance, climb_rate)
]]
--[[
c++_code
// check for terrain at grid spacing intervals
while (distance > 0) {
    gcs().send_text(MAV_SEVERITY_INFO, "lookahead distance %.1f", distance);
    loc.offset_bearing(bearing, grid_spacing);
    climb += climb_ratio * grid_spacing;
    distance -= grid_spacing;
    float height;
    if (height_amsl(loc, height)) {
        float rise = (height - base_height) - climb;
        //if(rise > 0)
            gcs().send_text(MAV_SEVERITY_INFO, "lookahead alt %.1f climb %.1f rise %.1f", height, climb, rise);
        if (rise > lookahead_estimate) {
            lookahead_estimate = rise;
            loc_highest = loc;
            gcs().send_text(MAV_SEVERITY_INFO, "lookahead estimate %.1f", lookahead_estimate);
        }
    }
}
--]]

local function terrain_lookahead(start_location, search_bearing, search_distance, search_ratio)
    local highest_location = nil
    local climb = 0.0
    local highest_rise = 0.0
    local height
    local search_location = start_location:copy()
    search_location:change_alt_frame(mavlink.ALT_FRAME.ABSOLUTE)

    local base_height = terrain:height_amsl(search_location, true)
    --gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. string.format("BASE terrain height %.0f alt %.0f", base_height, search_location:alt() * 0.01) )

    while search_distance > 0 do
        search_location:offset_bearing(search_bearing, terrain_spacing)
        climb = climb + search_ratio * terrain_spacing

        height = terrain:height_amsl(search_location, true)
        if height ~= nil then
            --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(": terrain height %.0f alt %.0f height %.0f", height, search_location:alt() * 0.01, height) )
            local rise = (height - base_height) - climb
            if rise > highest_rise then
                local highest_alt_cm = math.floor(math.min(height * 100.0, 320000))
                --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(": highest height %.0f rise %.0f", highest_alt_cm, highest_rise) )
                highest_rise = rise
                highest_location = search_location:copy()
                highest_location:alt(highest_alt_cm)
            end
        end

        search_distance = search_distance - terrain_spacing
    end

    return highest_location
end

-- returns required pitch to avoid hitting something between now and the next waypoint or other destination such as
-- home locaiton for RTL
terrain_approaching = function(clearance)
    if current_location == nil then
        gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": current_location NIL") )
        return
    end
    local wp_distance = current_location:get_distance(current_location_target)
    local wp_bearing = math.deg(current_location:get_bearing(current_location_target))
    local pitch_required = 0
    local alt_required_amsl = -1
    local highest_location
    local highest_alt_difference = 0.0
    local current_location_amsl = current_location:copy()
    current_location_amsl:change_alt_frame(mavlink.ALT_FRAME.ABSOLUTE)

    if wp_distance == nil then
        gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": wp_distance NIL") )
        return 0.0, -1.0, 0.0
    end

    --local height = terrain:height_amsl(current_location, true)
    --if height ~= nil then
    --    gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": terrain height %f alt %f", height, current_location:alt()) )
    --end

    -- lookahead to the next waypoint

    highest_location = terrain_lookahead(current_location, wp_bearing, wp_distance, 0.5 * tecs_climb_max / ground_speed)
    if highest_location == nil then
        return 0.0, -1.0, 0.0
    end

    -- need to know how far ahead the highest locaiotn is and how much higher than the current 
    -- altitude to calculate a minimum required glide slope (which TECS already does)
    --local highest_altitude_amsl = highest_location:alt() * 0.01
    highest_location:change_alt_frame(mavlink.ALT_FRAME.ABSOLUTE)
    local altitude_difference =  (highest_location:alt() * 0.01) + clearance - (current_location_amsl:alt() * 0.01)
    if altitude_difference > 0 then
        -- what is the pitch up requried to acheive that altitude?
        local highest_distance = current_location_amsl:get_distance(highest_location)
        pitch_required = math.deg(math.atan(altitude_difference,highest_distance))
        -- the target location we need to hit needs to be AMSL to ensure we fly above terrain
        alt_required_amsl = highest_location:alt() * 0.01
        highest_alt_difference = altitude_difference
        --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": lookahead alt diff %.0f dist %.1f pitch %.1f", altitude_difference, highest_distance, pitch_required) )
    end
    --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(": max %.0f curr %.0f diff %.0f pitch %.1f",
    --        highest_location:alt() * 0.01, current_location_amsl:alt() * 0.01, altitude_difference, pitch_required) )

    return pitch_required, alt_required_amsl, highest_alt_difference
end

local function highest_arc_terrain(loiter_center, bearing_start, bearing_step, arc_max)
    local next_increment = bearing_step
    local highest_terrain = 0.0
    while math.abs(next_increment) < arc_max do
        local test_bearing = wrap_360(bearing_start + next_increment)

        local loiter_edge = loiter_center:copy()
        loiter_edge:offset_bearing(test_bearing, wp_loiter_rad)
        local terrain_height = terrain:height_amsl(loiter_edge, true)
        --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": arc increment %.0f max %.0f step %.0f height %.0f", 
        --                next_increment, arc_max, bearing_step, terrain_height) )
        if terrain_height > highest_terrain then
            highest_terrain = terrain_height or 0.0
        end
        next_increment = next_increment + bearing_step
    end

    return highest_terrain
end

-- avoids upcoming terrain entering a loiter to altitude
-- loiters either left or right depending on which is less likely to hit terrain
avoid_terrain = function(target_alt_amsl) -- forward declaration above
    -- calculate the highest location in an arc assuming that we loiter first right and then left
    -- then we choose the one that has the lowest terrain either way
    if current_location == nil then
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": avoid_terrain no current_location")
        return
    end
    --gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT ..": avoid_terrain: " .. target_alt_amsl)
    local direction = ""
    local loiter_center_left = current_location:copy()
    local loiter_center_right = current_location:copy()
    loiter_center_left:offset_bearing(wrap_360(current_bearing - 90), wp_loiter_rad)
    loiter_center_right:offset_bearing(wrap_360(current_bearing + 90), wp_loiter_rad)
    -- at a radius of WP_LOITER_RAD how many degrees is TERRAIN_SPACING
    -- Central Angle = Arc length(AB) / Radius(OA) = (s × 360°) / 2πr
    local spacing_degrees = (terrain_spacing * 360.0) / (2.0 * math.pi * wp_loiter_rad)

    local highest_left_terrain = highest_arc_terrain(loiter_center_left, current_bearing, -spacing_degrees, 180)
    local highest_right_terrain = highest_arc_terrain(loiter_center_right, current_bearing, spacing_degrees, 180)
    local roll = ahrs:get_roll()

    --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT ..
    --    string.format(": direction? left %.0f right %.0f roll %.1f", highest_left_terrain, highest_right_terrain, math.deg(ahrs:get_roll()) ) )

    set_avoid_mode(FLIGHT_MODE.GUIDED)

    -- loiter up to the requjired AMSL height, in the direction of lowest terrain
    if highest_left_terrain < highest_right_terrain or roll < -30 then
        --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " LOITER LEFT to alt:" .. target_alt_amsl )
        mavlink.set_vehicle_target_location({lat = loiter_center_left:lat(),
            lng = loiter_center_left:lng(),
            alt = target_alt_amsl,
            alt_frame = mavlink.ALT_FRAME.ABSOLUTE,
            yaw = 1 })
        direction = "left"
    else
        --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " LOITER RIGHT to alt: " .. target_alt_amsl )
        mavlink.set_vehicle_target_location({lat = loiter_center_right:lat(),
            lng = loiter_center_right:lng(),
            alt = target_alt_amsl,
            alt_frame = mavlink.ALT_FRAME.ABSOLUTE,
            yaw = -1 })
        direction = "right"
    end

    -- Set an extra hight altitude to ensure the plane tries to climb as fast as possible
    mavlink.set_vehicle_target_altitude({alt = target_alt_amsl + 100.0, alt_frame = mavlink.ALT_FRAME.ABSOLUTE})
    mavlink.set_vehicle_speed({speed=airspeed_max})

    return direction
end

local switch_on = true
local last_switch_state = -1

local function activate()
    switch_on = true
    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": activated")
    pitch_last_good_timestamp = now
end

local function deactivate()
    switch_on = false
    gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT ..": deactivated")
end


-- This method checks if the activation switch has been newly enabled or disabled
local function check_activation_switch()
    local switch_function = ZTA_ACT_FN:get()
    if switch_function == nil then
        return
    end
    local switch_state = rc:get_aux_cached(switch_function) or -1
    if (switch_state ~= last_switch_state) then
        if switch_state == 0 then -- switch Low to activate - so defaults to on
            activate()
        elseif switch_state == 2 then -- switch Jight to turn off
            deactivate()
        end
        -- Don't know what to do with the 3rd switch position right now.
        last_switch_state = switch_state
    end
end

-- this method checks that all the conditions for terrain avoidance are met
local close_to_home = false
local function terravoid_active()
    if not switch_on then  
        return false
    end
    if not (arming:is_armed()) then
        return false
    end
    if not(vehicle_mode == FLIGHT_MODE.AUTO or vehicle_mode == FLIGHT_MODE.GUIDED or
        vehicle_mode == FLIGHT_MODE.RTL or vehicle_mode == FLIGHT_MODE.QRTL or
        ((quading_active or airbrake_on) and (vehicle_mode == FLIGHT_MODE.QLOITER or vehicle_mode == FLIGHT_MODE.QHOVER))) then
        return false
    end

    local home = ahrs:get_home()
    local home_distance = 0.0
    if home ~= nil and current_location ~= nil then
        home_distance = home:get_distance(current_location) or 0.0
    end
    if home_distance ~= nil and home_distance_max ~= nil and home_distance < home_distance_max then
        if not close_to_home then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": close to home")
            close_to_home = true
        end
        return false
    end
    if close_to_home then
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": away from home")
    end
    close_to_home = false

    return true
end

local old_now = millis():tofloat() * 0.001

-- main update function called by protected_wrapper REFRESH_RATE times per second
local function update()
    now = millis():tofloat() * 0.001
    current_location = ahrs:get_location()
    if current_location ~= nil then
        current_altitude = current_location:alt() * 0.01
    end
    vehicle_mode = vehicle:get_mode()
    current_location_target = vehicle:get_target_location()
    current_bearing = vehicle:get_wp_bearing_deg() or -1
    groundspeed_vector = ahrs:groundspeed_vector()
    ground_speed = ahrs:groundspeed_vector():length()

       -- save the previous target location only if in auto mode, if restoring it in AUTO mode
       -- don't update it if already pitching or quading because the altitude change will mess up the history
    if vehicle_mode == FLIGHT_MODE.AUTO and location_tracker ~= nil and not (pitching_active or quading_active) then
        location_tracker.update()
    end

    --[[
    -- display terrain altitude every 1 s
    local terrain_altitude = terrain:height_above_terrain(true)
    if math.floor(old_now) ~= math.floor(now) then
        gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": terrain alt: " .. terrain_altitude)
    end
    --]]

    -- make the pitching and quading parameters updatable in the air (refresh every second)
    if math.floor(old_now) ~= math.floor(now) then
        pitch_groundspeed_min = ZTA_PTCH_GSP_MIN:get()
        pitch_down_min = ZTA_PTCH_DWN_MIN:get()
        pitch_forward_min = ZTA_PTCH_FWD_MIN:get()
        pitch_timeout = ZTA_PTCH_TIMEOUT:get()
        home_distance_max = ZTA_HOME_DIST:get()

        quad_down_min = ZTA_QUAD_DWN_MIN:get()
        quad_forward_min = ZTA_QUAD_FWD_MIN:get()

        altitude_max = ZTA_ALT_MAX:get()

        groundspeed_max = ZTB_GSP_MAX:get() or -1
        groundspeed_airbrake_limit = ZTB_GSP_AIRBRAKE:get() or 0
        airspeed_min = AIRSPEED_MIN:get() or 5
        airspeed_max = AIRSPEED_MAX:get() or 30
        airspeed_cruise = AIRSPEED_CRUISE:get() or 18
    end
    check_activation_switch()
    if not terravoid_active() then
        if quading_active then
            stop_quading()
        end
        if pitching_active then
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": Stop Pitching done update")
            stop_pitching()
        end
        pitching_active = false
        quading_active = false
        cmtc_active = false
        return
    end

    populate_rangefinder_values()

    -- first decide if we are seriously close to the terrain and need to start quading
    if not quading_active and not check_quading() and not cmtc_active then
        -- lets check if our current flight path is likely to hit terrain 
        -- sometime soon, and if so we need to avoid it.
        local pitch_required_deg, alt_required_amsl, terrain_diff = terrain_approaching(pitch_down_min)
        --if pitch_required_deg > 0 then
        --    gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": required pitch: %.1f alt %.0f", pitch_required_deg, alt_required_amsl))
        --end
        if pitch_required_deg > (ptch_lim_max_deg * 0.5) then
            gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(": Can't Make that climb", terrain_diff) )
            gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(": CMTC pitch required %.0f deg", pitch_required_deg) )
            -- need to fly OVER the highest point - with pitch_down_min clearance * 2
            start_cmtc(alt_required_amsl + pitch_down_min * 2.0)
        else
            -- otherwise - lets see if we are close enough to need to start pitching
            check_pitching()
        end
    end

    if terrain_max_exceeded and not cmtc_active then
        if quading_active then
            stop_quading()
        end
        if pitching_active then
            stop_pitching()
        end
        --if cmtc_active then
        --    stop_cmtc()
        --end
    end

    -- quading is the priority. If we are quading, do that, otherwise if we are pitching do that
    if quading_active then
        --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. ": update do_quading()")
        do_quading()
        return
    elseif pitching_active then
        --gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. ": update do_pitching()")
        do_pitching()
        return
    elseif cmtc_active then
        do_cmtc()
        return
    end

    -- in "normal" flight we want to maintain a minimum forward groundspeed
    if groundspeed_vector == nil then
        gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%s: no groundspeed", SCRIPT_NAME_SHORT) )
        -- slowdown_airbrake_stop()
    else
        local groundspeed_current = groundspeed_vector:length() or 0
        --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": groundspeed current %f max %f", groundspeed_current, groundspeed_max) )
        if groundspeed_max > 0 and groundspeed_current > groundspeed_max then
            -- if the groundspeed is too hign we need to slow down
            --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": groundspeed current %f max %f", groundspeed_current, groundspeed_max) )
            slow_down({error = (groundspeed_max - groundspeed_current)})
        elseif groundspeed_airbrake_limit ~= 0 then
            --gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. string.format(": calling STOP groundspeed current %f max %f", groundspeed_current, groundspeed_max) )
            slowdown_airbrake_end()
--            set_vehicle_speed({})
            mavlink.set_vehicle_speed({}) -- special airspeed value meaning "default"
        end
    end

end

-- wrapper around update(). This calls update() at 1/REFRESHRATE Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
       gcs:send_text(0, SCRIPT_NAME_SHORT .. ": Error: " .. err)
       -- when we fault we run the update function again after 1s, slowing it
       -- down a bit so we don't flood the console with errors
       return protected_wrapper, 1000
    end
    return protected_wrapper, 1000 * REFRESH_RATE
end

local function delayed_startup()
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
    return protected_wrapper()
end

-- wait a bit for AP to come up then start running update loop
if FWVersion:type() == 3 and q_enable == 1 and terrain_enable == 1 then
    return delayed_startup, 1000 * STARTUP_DELAY
else
    gcs:send_text(MAV_SEVERITY.NOTICE,string.format("%s: Must run on QuadPlane with terrain follow", SCRIPT_NAME_SHORT))
end
