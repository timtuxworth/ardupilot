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

   Quadplane Motor fail recovery
   This script monitors the VTOL motors on a quadplane in VTOL flight modes.

   If it detects excessive uncommanded changes in yaw then it assumes motor failure
   if the vehicle is not trying to rotate (yaw_target_rate is small)
   if the actual yaw is diverging from the target (yaw_error is large)
   then we accumulate that error in yaw_error_integral
   if the yaw_error_integral gets too high it assumes motor failure (this is very much a heuristic)


   If it detects a failure in one or more of the motors, it switches the plane
   into a fixed wing LOITER at RTL_ALT at the current location and tries
   to notify the pilot by sending error messages to the GCS.
--]]

SCRIPT_VERSION = "4.7.0-003"
SCRIPT_NAME = "Quadplane Motor Fail Recover"
SCRIPT_NAME_SHORT = "QMFsave"

REFRESH_RATE = 0.2   -- in seconds, so 5Hz

-- FOLL_ALT_TYPE and Mavlink FRAME use different values 
ALT_FRAME = { GLOBAL = 0, RELATIVE = 1, TERRAIN = 3}

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
MAV_FRAME = {GLOBAL = 0, GLOBAL_RELATIVE_ALT = 3,  GLOBAL_TERRAIN_ALT = 10}
MAV_CMD_INT = { ATTITUDE = 30, GLOBAL_POSITION_INT = 33, 
                  DO_SET_MODE = 176, DO_CHANGE_SPEED = 178, DO_REPOSITION = 192,
                  CMD_SET_MESSAGE_INTERVAL = 511, CMD_REQUEST_MESSAGE = 512,
                  GUIDED_CHANGE_SPEED = 43000, GUIDED_CHANGE_ALTITUDE = 43001, GUIDED_CHANGE_HEADING = 43002 }
MAV_SPEED_TYPE = { AIRSPEED = 0, GROUNDSPEED = 1, CLIMB_SPEED = 2, DESCENT_SPEED = 3 }
MAV_HEADING_TYPE = { COG = 0, HEADING = 1, DEFAULT = 2} -- COG = Course over Ground, i.e. where you want to go, HEADING = which way the vehicle points 

FLIGHT_MODE = {AUTO=10, RTL=11, LOITER=12, GUIDED=15, QHOVER=18, QLOITER=19, QRTL=21}
---FLIGHT_MODE = {STABILIZE = 0, ALT_HOLD = 2, AUTO = 3, GUIDED = 4, LOITER = 5, RTL = 6, CIRCLE = 7, LAND = 9, FOLLOW = 23 }

local now = millis():tofloat() * 0.001
local now_display = now
local mode = vehicle:get_mode()

local PARAM_TABLE_KEY = 123
local PARAM_TABLE_PREFIX = "ZMF_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end
-- setup follow mode specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')

--[[
    // @Param: ZMF_PWM_LOW
    // @DisplayName: Motor Failure low motor pwm
    // @Description: If one motor is below ZMF_LOW_PWM and one is above ZMF_PWM_HIGH
    // @User: Standard
    // @Range: 800 2200
--]]
ZMF_PWM_LOW = bind_add_param('PWM_LOW', 1, 1200)

--[[
    // @Param: ZMF_PWM_HIGH
    // @DisplayName: Motor Failure high motor pwm
    // @Description: If one motor is below ZMF_LOW_PWM and one is above ZMF_PWM_HIGH
    // @User: Standard
    // @Range: 800 2200
--]]
ZMF_PWM_HIGH = bind_add_param('PWM_HIGH', 2, 1950)

--[[
    // @Param: ZMF_YAW_DEG
    // @DisplayName: Yaw rate sensitivity 
    // @Description: If yaw rate exceeded + yaw error exceeded + yaw integral exceeded = motor failure
    // @User: Standard
    // @Unit: deg
--]]
ZMF_YAW_DEG = bind_add_param('YAW_DEG', 3, 3.0)

--[[
    // @Param: ZMF_YAW_ERR_DEG
    // @DisplayName: Yaw error sensitivity 
    // @Description: If yaw rate exceeded + yaw error exceeded + yaw integral exceeded = motor failure
    // @User: Standard
    // @Unit: deg
--]]
ZMF_YAW_ERR_DEG = bind_add_param('YAW_ERR_DEG', 4, 8.0)

--[[
    // @Param: ZMF_YAW_INT_DEG
    // @DisplayName: Yaw integral sensitivity 
    // @Description: If yaw rate exceeded + yaw error exceeded + yaw integral exceeded = motor failure
    // @User: Standard
    // @Unit: deg
--]]
ZMF_YAW_INT_DEG = bind_add_param('YAW_INT_DEG', 5, 45.0)

--[[
    // @Param: ZMF_ACTION
    // @DisplayName: Action to perform on failure 
    // @Description: Action to perform on detected motor faiure. Ignore, Report, LOITER
    // @User: Standard
    // @Values: 0:Ignore,1:Report,2:LOITER
--]]
ZMF_ACTION = bind_add_param('ACTION', 6, 1)

local yaw_rate_sensitivity_deg = ZMF_YAW_DEG:get() or 3.0
local yaw_error_sensivitity_deg = ZMF_YAW_ERR_DEG:get() or 8.0
local yaw_integral_sensitivity_deg = ZMF_YAW_INT_DEG:get() or 45.0

local pwm_low = ZMF_PWM_LOW:get() or 1200
local pwm_high = ZMF_PWM_HIGH:get() or 1950

Q_ENABLE = Parameter("Q_ENABLE")
Q_ASSIST_ALT = Parameter("Q_ASSIST_ALT")
TERRAIN_ENABLE = Parameter("TERRAIN_ENABLE")

local function switch_to_forward_flight()
    local altitude = ahrs:get_hagl()
    if TERRAIN_ENABLE:get() == 1 then
        altitude = terrain:height_above_terrain(true)
    end

    if altitude <= Q_ASSIST_ALT:get() then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("Motors: failed - altitude %0.1f too low", altitude ))
    else
        local zmf_action = (ZMF_ACTION:get() or 1)
        if zmf_action == 1 then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("Motors: failed!"))
        elseif zmf_action == 2 then
            gcs:send_text(MAV_SEVERITY.INFO, string.format("Motors: failed - Fixed Wing save"))
            vehicle:set_mode(FLIGHT_MODE.LOITER)
        end
    end
    -- temporarily disable Q_ASSIST since it is likely not useful with a motor missing (set but not save)
    param:set("Q_ASSIST_SPEED",-1)
end

local last_motor_outputs = { 0, 0, 0, 0 }
local last_yaw_target_rad = nil
local yaw_error_integral = 0.0
local failed = false

local function VTOL_motor_failure_reset()
    yaw_error_integral = 0.0
    failed = false
end

local function VTOL_motor_failure()
    if failed then
        return
    end

    -- check the parameters every cycle so they can be adjusted/tuned at runtime.
    pwm_low = ZMF_PWM_LOW:get() or 1200
    pwm_high = ZMF_PWM_HIGH:get() or 1950
    yaw_rate_sensitivity_deg = ZMF_YAW_DEG:get() or 3.0
    yaw_error_sensivitity_deg = ZMF_YAW_ERR_DEG:get() or 8.0
    yaw_integral_sensitivity_deg = ZMF_YAW_INT_DEG:get() or 45.0

    -- the best indication of motor faiure is undemanded yaw extreme changes
    -- so lets get the yaw
    local yaw_actual_rad = ahrs:get_yaw()
    local target_attitude = AC_AttitudeControl:get_attitude_target_quat()
    local yaw_target_rad = target_attitude:get_euler_yaw()
    -- print(string.format("yaw actual: %.0f yaw target: %.0f", math.deg(yaw_actual_rad), math.deg(yaw_target_rad)))

    local yaw_error = math.atan(math.sin(yaw_actual_rad - yaw_target_rad), math.cos(yaw_actual_rad - yaw_target_rad))

    -- track the change in target yaw to detect intentional yaw commands
    if last_yaw_target_rad == nil then
        last_yaw_target_rad = yaw_target_rad
    end
    local yaw_target_rate_rad = math.atan(math.sin(yaw_target_rad - last_yaw_target_rad), math.cos(yaw_target_rad - last_yaw_target_rad))
    last_yaw_target_rad = yaw_target_rad

    -- if the target yaw isn't changing much, but yaw error is building, suspect failure
    if math.abs(yaw_target_rate_rad) < math.rad(yaw_rate_sensitivity_deg) then
        if math.abs(yaw_error) > math.rad(yaw_error_sensivitity_deg)  then
            yaw_error_integral = yaw_error_integral + yaw_error
        else
            yaw_error_integral = yaw_error_integral * 0.9 -- decay small error
        end
    else
        yaw_error_integral = yaw_error_integral * 0.8 -- decay when rotating intentionally
    end
    if math.abs(yaw_error_integral) > math.rad(yaw_integral_sensitivity_deg) then
        print("MOTOR FAILURE suspected: yaw diverging")
        return true
    end

    -- the other way to do this is to look for a very low pwm output and a very high pwm_output - this copies Tridges code
    local pwm_low_seen = false
    local pwm_high_seen = false
    local motor_lost = -1
    local pwm_high_count = 0
    for i = 1, 4 do
        local motor_pwm = SRV_Channels:get_output_pwm(32+i)
        if motor_pwm < pwm_low and last_motor_outputs[i] < pwm_low then
            pwm_low_seen = true
        end
        if motor_pwm > pwm_high and last_motor_outputs[i] > pwm_high then
            pwm_high_seen = true
            pwm_high_count = 1
        end

        last_motor_outputs[i] = motor_pwm
    end
    if pwm_high_seen and pwm_high_count == 1 then
        print ("high pwm")
        for i = 1, 4 do
            local motor_pwm = SRV_Channels:get_output_pwm(32+i)
            print(string.format("motor %d pwm %d", i, motor_pwm))
        end
        gcs:send_text(MAV_SEVERITY.ERROR, string.format("PWM motor lost"))
        return true
    end
    if pwm_low_seen and pwm_high_seen then
        -- we probably have a motor failure
        gcs:send_text(MAV_SEVERITY.ERROR, string.format("PWM motor lost"))
        return true
    end

    return false
end


-- main update function
local function update()
    now = millis():tofloat() * 0.001
    mode = vehicle:get_mode()

    if quadplane:in_vtol_mode() and vehicle:get_likely_flying() then
        if VTOL_motor_failure() then
            switch_to_forward_flight()
        end
    else
        VTOL_motor_failure_reset()
    end
end

-- wrapper around update(). This calls update() at 1/REFRESH_RATE Hz
-- and if update faults then an error is displayed, but the script is not
-- stopped
local function protected_wrapper()
    local success, err = pcall(update)

    if not success then
       gcs:send_text(MAV_SEVERITY.ALERT, SCRIPT_NAME_SHORT .. "Internal Error: " .. err)
       -- when we fault we run the update function again after 1s, slowing it
       -- down a bit so we don't flood the console with errors
       return protected_wrapper, 1000
    end
    return protected_wrapper, 1000 * REFRESH_RATE
end

local function delayed_start()
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
    return protected_wrapper()
end

-- start running update loop - waiting 20s for the AP to initialize
if FWVersion:type() == 3 and Q_ENABLE:get() == 1 then
    return delayed_start, 20000
else
    gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: must run on QuadPlane", SCRIPT_NAME_SHORT))
end
