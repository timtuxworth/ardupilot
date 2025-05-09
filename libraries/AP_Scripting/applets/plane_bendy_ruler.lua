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

   Bendy Ruler for Plane
   This script is based on work by Canberra UAV and Andrew Tridgell for the 2018 Outback Challenge competition.
   The original work was done in c++.

   This version of the code implements dynamic object avoidance based on ADSB inputs,
   which could be real ADSB inputs from an onboard receiver or remote source, or from
   another source (such as FOLLOW_TARGET or GLOBAL_POSITION_INT from other vehicles) that
   have been modified to send ADSB packets.

--]]

SCRIPT_VERSION = "4.7.0-001"
SCRIPT_NAME = "Plane Bendy Ruler Avoid"
SCRIPT_NAME_SHORT = "PBRAvd"

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
MAV_COLLISION_ACTION={
    NONE=0, -- Ignore any potential collisions 
    REPORT=1, -- Report potential collision 
    ASCEND_OR_DESCEND=2, -- Ascend or Descend to avoid threat 
    MOVE_HORIZONTALLY=3, -- Move horizontally to avoid threat 
    MOVE_PERPENDICULAR=4, -- Aircraft to move perpendicular to the collision's velocity vector 
    RTL=5, -- Aircraft to fly directly back to its launch point 
    HOVER=6, -- Aircraft to stop in place 
}

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

ALT_FRAME = { ABSOLUTE=0, ABOVE_HOME=1, ABOVE_ORIGIN=2, TERRAIN=3}

local gcs_threat = {
    src = nil,
    src_id = 0,
    timestamp_ms = 0,
    _location = nil,
    _velocity = nil,
    threat_level = nil,
    closest_approach_xy = 0.0,
    closest_approach_z = 0.0,
    time_to_closest_approach = 0.0,
    distance_to_closest_approach = 0.0
}
local gcs_action = MAV_COLLISION_ACTION.NONE

local avoidance_request = {
    current_loc = nil,
    target_loc = nil,
    groundspeed = 0.0,
    airspeed = 0.0,
    wind_dir_rad = 0.0,
    wind_speed = 0.0,
    request_time_ms = 0
}

local avoid_req2 = {
    current_loc = nil,
    target_loc = nil,
    groundspeed = 0.0,
    airspeed = 0.0,
    wind_dir_rad = 0.0,
    wind_speed = 0.0,
    request_time_ms = 0
}

local avoidance_result = {
    target_loc = nil,
    new_target_loc = nil,
    result_time_ms = 0,
    avoidance_needed = nil
}

local PARAM_TABLE_KEY = 129
local PARAM_TABLE_PREFIX = "ZBR_"

-- bind a parameter to a variable
local function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('%s: could not find %s parameter', SCRIPT_NAME_SHORT, name))
   return p
end

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('%s: could not add param %s', SCRIPT_NAME_SHORT, name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 20), string.format('%s: could not add param table', SCRIPT_NAME_SHORT))

--[[
  // @Param: ZBR_ENABLE
  // @DisplayName: Enable/Disable bendy ruler script
  // @Description: Whether Bendy Ruler is enabled/active or disabled
  // @Values: 0:Disabled,1:Enabled
--]]
ZBR_ENABLE     = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: ZBR_MARGIN_FENCE
  // @DisplayName: fence margin 
  // @Description: Avoidance margin for fence
  // @Units: m
--]]
ZBR_MARGIN_FENCE = bind_add_param('MARGIN_FENCE', 2, 50)

--[[
  // @Param: ZBR_MARGIN_DYN
  // @DisplayName: dynamic margin
  // @Description: Avoidance margin for dynamic objects
  // @Units: m
--]]
ZBR_MARGIN_DYN   = bind_add_param('MARGIN_DYN', 3, 20)

--[[
  // @Param: ZBR_MARGIN_EXCL
  // @DisplayName: exclusion zone margin
  // @Description: Avoidance margin for exclusion zones
  // @Units: m
--]]
ZBR_MARGIN_EXCL   = bind_add_param('MARGIN_EXCL', 4, 20)

--[[
  // @Param: ZBR_MARGIN_WIDE
  // @DisplayName: wide avoidance margin
  // @Description: Avoidance margin for wide avoidance
  // @Units: m
--]]
ZBR_MARGIN_WIDE   = bind_add_param('MARGIN_WIDE', 5, 30)

--[[
  // @Param: ZBR_MARGIN_HGT 
  // @DisplayName: height avoidance margin
  // @Description: Avoidance margin for height avoidance 
  // @Units: m
--]]
ZBR_MARGIN_HGT   = bind_add_param('MARGIN_HGT', 6, 60)

--[[
  // @Param: ZBR_LKAHD
  // @DisplayName: avoidance lookahead distance
  // @Description: Avoidance lookahead distance
  // @Units: m
--]]
ZBR_LKAHD  = bind_add_param('LKAHD', 7, 500)

--[[
  // @Param: ZBR_UPDATE_RATE
  // @DisplayName: rate to process avoidance
  // @Description: Avoidance processing rate
  // @Units: hz
--]]
ZBR_UPDATE_RATE  = bind_add_param('UPDATE_RATE', 8, 10.0)

--[[
  // @Param: ZBR_HEIGHT_USE
  // @DisplayName: Include height differences
  // @Description: Avoidance will consider height differences when calculating collisions
  // @Values: 0:Use Height,1:Ignore Height
--]]
ZBR_HEIGHT_USE  = bind_add_param('HEIGHT_USE', 9, 0)

--[[
  // @Param: ZBR_RAD_GA
  // @DisplayName: Radius for General Aviation
  // @Description: Avoidance radius for Fixed Wing aircraft/General Aviation (Helicopters? eVTOL?)
  // @Units: m
--]]
ZBR_RAD_GA  = bind_add_param('RAD_GA', 10, 300)

--[[
  // @Param: ZBR_RAD_WTHR
  // @DisplayName: Radius for Weather
  // @Description: Avoidance radius for Weather/Clouds/Rain 
  // @Units: m
--]]
ZBR_RAD_WTHR  = bind_add_param('RAD_WTHR', 11, 173)

--[[
  // @Param: ZBR_RAD_BIRD
  // @DisplayName: Radius for Birds
  // @Description: Avoidance radius for Migratory Birds 
  // @Units: m
--]]
ZBR_RAD_BIRD  = bind_add_param('RAD_BIRD', 12, 100)

--[[
  // @Param: ZBR_RAD_PREY
  // @DisplayName: Radius for Birds of Prey
  // @Description: Avoidance radius for Birds of Prey
  // @Units: m
--]]
ZBR_RAD_PREY  = bind_add_param('RAD_PREY', 13, 200)

--[[
  // @Param: ZBR_RAD_UAV
  // @DisplayName: Radius for UAVs/Drones
  // @Description: Avoidance radius for UAV/drone (MAVLink sourced)
  // @Units: m
--]]
ZBR_RAD_UAV  = bind_add_param('RAD_UAV', 14, 50)

WARN_DIST_XY  = bind_param("AVD_W_DIST_XY")
WARN_ACTION  = bind_param("AVD_W_ACTION")
local warn_act = (WARN_ACTION:get() or 1)
AVD_ENABLE  = bind_param("AVD_ENABLE")
ROLL_LIMIT_DEG = bind_param("ROLL_LIMIT_DEG")

local roll_limit_deg = (ROLL_LIMIT_DEG:get() or 45.0)
local lookahead_param = (ZBR_LKAHD:get() or 500.0)
local margin_fence = (ZBR_MARGIN_FENCE:get() or 50.0)
local refresh_rate = 1000.0 / (ZBR_UPDATE_RATE:get() or 10.0)

local current_lookahead = 0.0
local GRAVITY_MSS = 9.80665
local LOCATION_SCALING_FACTOR_INV = 89.83204953368922

COLLISION_DETECTED = false
SAVE_CURRENT_TARGET_LOC = Location()

--Auxiliary functions
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

--[[
    return true if two locations are identical
--]]
local function locations_equal(loc1, loc2)
    return (loc1:lat() == loc2:lat()) and (loc1:lng() == loc2:lng())
            and (loc1:alt() == loc2:alt())
            and (loc1:get_alt_frame() == loc2:get_alt_frame())
end

local function closest_distance_between_radial_and_point(w, p)
    local closest = closest_point(p, w)
    return math.sqrt(length_squared(closest - p))
end

local function closest_approach_xy(my_loc, my_vel, obstacle_loc, obstacle_vel, time_horizon)
    local delta_vel_ne = Vector2f()
    delta_vel_ne:x(obstacle_vel:x() - my_vel:x())
    delta_vel_ne:y(obstacle_vel:y() - my_vel:y())
    local delta_pos_ne = obstacle_loc:get_distance_NE(my_loc)     
    local line_segment_ne =Vector2f()
    line_segment_ne:x(delta_vel_ne:x() * time_horizon)
    line_segment_ne:y(delta_vel_ne:y() * time_horizon)
    local ret = closest_distance_between_radial_and_point(line_segment_ne, delta_pos_ne)    
    --print(string.format("   time_horizon: (%d)", time_horizon))
    --print(string.format("   delta pos: (y=%f, x=%f)", delta_pos_ne.x, delta_pos_ne.y))
    --print(string.format("   delta vel: (y=%f, x=%f)", delta_vel_ne.x, delta_vel_ne.y))
    --print(string.format("   line segment: (y=%f, x=%f)", line_segment_ne.x, line_segment_ne.y))
    --print(string.format("   closest: (%f)", ret))    
    return ret
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

-- Project forward from loc1 to a newlocation in the direction bearing_deg and distance m
-- the altitude of the new projected location should be based on alt_target_loc, including frame
local function location_project(loc1, bearing_deg, distance, alt_target_loc)
    -- Create a copy of the location projected distance meters in bearing_deg direction
    -- the projection should be in the frame project_in_frame
    --local loc2 = Location()
    --loc2=loc1
    --loc2:offset_bearing(bearing_deg, distance)
    local loc2 = alt_target_loc:copy()
    local lat, lon = offset_bearing(loc1:lat(), loc1:lng(), bearing_deg, distance)
    loc2:lat(lat)
    loc2:lng(lon)
    -- use the altitude/frame copied from loc1
    --loc2:alt(loc1:alt())
    --gcs:send_text(0, string.format("IN: Lat: %.2f, Lon: %.2f", loc1:lat(),loc1:lng()))
    --gcs:send_text(0, string.format("Dist: %.2f, Bear: %.2f", distance,bearing_deg)) 
    --gcs:send_text(0, string.format("Out: Lat: %.2f, Lon: %.2f", loc2:lat(),loc2:lng()))  
    --gcs:send_text(0, string.format("Frame: IN: %d, OUT: %d", loc1:get_alt_frame(),loc2:get_alt_frame()))
    return loc2
end

-- translate an obstacle src_id into a string for display purposes
local function get_obstacle_desc(src_id)
    if src_id < 256 then
        -- it's some form of UAV (because this will be a MAV_SYSID), 100m radius
        -- for some reason the mavproxy obstacle generator generates ADSB objects as MAVLink IDs
        return "MAV"
    elseif src_id < 20000 then
        -- fixed wing (general aviation), 300m radius
        return "GA"
    elseif src_id < 30000 then
        -- weather, radius 150 at ground, 300m at 3000m, 173m at 1500ft
        return "Weather"
    elseif src_id < 40000 then
        -- migratory bird, 100m
        return "Migratory Bird"
    elseif src_id < 50000 then
        -- bird of prey, 200m
        return "Bird of Prey"
    else
        -- default to 300, which is worst case
        return "Unknown"
    end
end

--get the avoidance radius in meters of a given obstacle type
local function get_avoidance_radius(obstacle)
    src_id = obstacle.src_id
    if src_id < 256 then
        -- it's some form of UAV (because this will be a MAV_SYSID), 100m radius
        return ZBR_RAD_UAV:get() or 100
    elseif src_id < 20000 then
        -- fixed wing, 300m radius
        return ZBR_RAD_GA:get() or 300
    elseif src_id < 30000 then
        -- weather, radius 150 at ground, 300m at 3000m, 173m at 1500ft
        return ZBR_RAD_WTHR:get() or 173
    elseif src_id < 40000 then
        -- migratory bird, 100m
        return ZBR_RAD_BIRD:get() or 100
    elseif src_id < 50000 then
        -- bird of prey, 200m
        return ZBR_RAD_PREY:get() or 200
    else
        -- default to 300, which is worst case
        return ZBR_RAD_GA:get() or 300
    end
end

--check if we are within the height range to need to avoid an obstacle
local function within_avoidance_height(obstacle,margin, deltat)
    local src_id = obstacle.src_id
    local loc = obstacle.location
    local velocity = obstacle.velocity

    if ZBR_ENABLE:get() == 1 and (ZBR_HEIGHT_USE:get() or 0) == 0 then
        return true
    end

    if src_id >= 20000 and src_id < 30000 then
        --weather, always avoid
        gcs:send_text(0, "Weather Alt")
        return true
    end
    local alt_cm

    if not loc:change_alt_frame(ALT_FRAME.ABSOLUTE) then
        return true
    else
        alt_cm=loc:alt()
    end    
    local obstacle_alt = alt_cm * 0.01
    local alt_min, alt_max

    if src_id < 20000 or (src_id >= 30000 and src_id < 40000) then
        -- fixed wing or migrating bird, height range 150m, deltat seconds of height change
        alt_min = obstacle_alt - (75 + margin)
        alt_max = obstacle_alt + (75 + margin)
    else
        -- bird of prey, from location to ground
        alt_max = obstacle_alt + margin
        alt_min = -10000
    end    
    -- note that velocity is NED
    if velocity:z() < 0 then
        alt_max = alt_max - (deltat * velocity:z())
    else
        alt_min = alt_min - (deltat * velocity:z())
    end
    local myloc = ahrs:get_position()
    if not myloc:change_alt_frame(ALT_FRAME.ABSOLUTE) then
        return true
    else
        alt_cm=myloc:alt()
    end
    local alt = alt_cm * 0.01
    -- are we in the range of avoidance heights?
    return (alt > alt_min) and (alt < alt_max)
end

--get height difference between an obstacle and our location (can we use the new Location:get_height_above?)
local function obstacle_height_difference(obstacle)
    local obstacle_loc = obstacle.location:copy()
    local alt1_cm=0;
    obstacle_loc:change_alt_frame(ALT_FRAME.ABSOLUTE)
    alt1_cm = obstacle_loc:alt()

    local alt2_cm=0;
    local myloc = ahrs:get_position()
    myloc:change_alt_frame(ALT_FRAME.ABSOLUTE)
    alt2_cm = myloc:alt()
    return (alt1_cm - alt2_cm) * 0.01;
end

--[[
    this function will be used to implement
    MAV_CMD_NAV_DELAY_AIRSPACE_CLEAR, CONDITION_TYPE_AIRSPACE_CLEAR
    and CONDITION_TYPE_AIRSPACE_NOT_CLEAR
--]]
local function mission_clear(current_loc, xy_clearance, z_clearance, time_s)
    local timeout_ms = 5000
    local now = millis()
    --assume we are not moving

    local my_vel = Vector3f()
    my_vel:x(0)
    my_vel:y(0)
    my_vel:z(0)
    local obs_count = 0
    local closest_idx = -1
    local closest_dist = 5000
    local closest_radius = 0
    local obstacle_count = avoid:num_obstacles()
    if obstacle_count == 0 then
        --gcs:send_text(0, "No obstacles detected")
    else
        --gcs:send_text(0, string.format("Num of Obstacle: %d", obstacle_count))
    end
    for i = 0, obstacle_count-1 do
        local obstacle = {}
        obstacle.timestamp_ms=avoid:get_obstacle_timeout(i)
        obstacle.velocity=avoid:get_obstacle_vel(i)
        obstacle.location=avoid:get_obstacle_loc(i)
        obstacle.src_id=avoid:get_obstacle_id(i)  
        if (now - obstacle.timestamp_ms) > timeout_ms then
            goto continue
        end        
        obs_count = obs_count + 1 

        if not within_avoidance_height(obstacle, z_clearance, time_s) then
            goto continue
        end        
        -- get updated obstacle position
        local obstacle_loc =Location()
        obstacle_loc = obstacle.location
        local obstacle_velocity = Vector2f()
        obstacle_velocity:x(obstacle.velocity:x())
        obstacle_velocity:y(obstacle.velocity:y())
        local dt = (now - obstacle.timestamp_ms):tofloat() * 0.001
        obstacle_loc:offset(obstacle_velocity:x() * dt, obstacle_velocity:y() * dt)        

        local closest_xy = closest_approach_xy(current_loc, my_vel, obstacle_loc, obstacle.velocity, time_s)
        local radius = get_avoidance_radius(obstacle)

        if closest_xy < closest_dist then
            closest_dist = closest_xy
            closest_idx = i
            closest_radius = radius
        end
        ::continue::
    end

    gcs_threat.src = obs_count
    gcs_threat.closest_approach_xy = closest_dist - closest_radius
    gcs_threat.closest_approach_z = 0
    gcs_threat.threat_level = MAV_COLLISION_THREAT_LEVEL.NONE
    gcs_action = MAV_COLLISION_ACTION.NONE
    if closest_dist < (xy_clearance + closest_radius) then
        -- it could come within the radius in the given time
        local obstacle = {}
        obstacle.timestamp_ms=avoid:get_obstacle_timestamp(closest_idx)
        obstacle.velocity=avoid:get_obstacle_vel(closest_idx)
        obstacle.location=avoid:get_obstacle_loc(closest_idx)
        obstacle.src_id=avoid:get_obstacle_id(closest_idx)
        gcs_threat.src_id = obstacle.src_id
        gcs_threat.threat_level = MAV_COLLISION_THREAT_LEVEL.HIGH
        gcs_threat.time_to_closest_approach = 0
        gcs_threat.closest_approach_z = obstacle_height_difference(obstacle)
        gcs_action = MAV_COLLISION_ACTION.REPORT
        return false
    end
    -- All clear
    return true
end

--[[
  given our location, velocity and a avoidance time, find the closest
  we will come to any of the obstacles avoidance radiuses
  A negative result means we will come within the avoidance radius of at least one object
--]]
local function obstacle_avoidance_margin(our_loc, our_velocity, avoid_sec)
    local timeout_ms = 5000
    local margin = math.max((ZBR_MARGIN_WIDE:get() or 30.0), (WARN_DIST_XY:get() or 1000.0))
    local num_outside_height_range = 0
    local num_timed_out = 0
    local closest_dist = 0
    local closest_id = 0
    gcs_threat.closest_approach_z = 1000
    local obs_count = 0

    local obstacle_count = avoid:num_obstacles()
    for i = 0, obstacle_count-1 do
        -- obstacles can update via MAVLink while we are calculating
        -- avoidance margins
        local obstacle = {}
        obstacle.timestamp_ms = avoid:get_obstacle_timestamp(i)
        obstacle.velocity = avoid:get_obstacle_vel(i)
        obstacle.location = avoid:get_obstacle_loc(i)
        obstacle.src_id = avoid:get_obstacle_id(i)

        local now =millis()
        if now - obstacle.timestamp_ms > timeout_ms then
            num_timed_out = num_timed_out + 1
            goto continue
        end
        obs_count = obs_count + 1
        if not within_avoidance_height(obstacle, ZBR_MARGIN_HGT :get(), avoid_sec) then
            num_outside_height_range = num_outside_height_range + 1
            goto continue
        end        
        -- use our starting point as origin
        local obstacle_position = our_loc:get_distance_NE(obstacle.location)
        -- update obstacle position by delta time since we logged its position
        local obstacle_velocity = Vector2f()
        obstacle_velocity:x(obstacle.velocity:x())
        obstacle_velocity:y(obstacle.velocity:y())
        local dt = (now - obstacle.timestamp_ms):tofloat() * 0.001
        obstacle_position:x(obstacle_position:x() + (obstacle_velocity:x() * dt))   
        obstacle_position:y(obstacle_position:y() + (obstacle_velocity:y() * dt))       
        -- get our velocity relative to obstacle
        local relative_velocity = our_velocity - obstacle_velocity
        local final_pos = Vector2f()
        final_pos:x(relative_velocity:x() * avoid_sec)
        final_pos:y(relative_velocity:y() * avoid_sec)
        -- lookup the min distance to keep from the object
        local radius = get_avoidance_radius(obstacle)
        -- assume that messages about aircraft position could be up to 2s old when we get them
        local position_lag = 2.0
        local position_error = position_lag * obstacle_velocity:length()
        local dist = closest_distance_between_radial_and_point(final_pos, obstacle_position) - (radius + position_error)
        dist = dist - (ZBR_MARGIN_DYN:get() or 20)
        --gcs:send_text(MAV_SEVERITY.INFO, string.format("distance:%.1f, margin: %.1f",dist, margin))
        if dist < margin then
            margin = dist
            closest_dist = dist
            closest_id = i
        end
        ::continue::
    end
    local obstacle = {}
    if closest_dist > 0 then
        -- update threat report for GCS
        obstacle.timestamp_ms = avoid:get_obstacle_timestamp(closest_id)
        obstacle.velocity = avoid:get_obstacle_vel(closest_id)
        obstacle.location = avoid:get_obstacle_loc(closest_id)
        obstacle.src_id = avoid:get_obstacle_id(closest_id)
        obstacle.distance = closest_dist + (ZBR_MARGIN_DYN:get() or 20)

        gcs_threat.src = obs_count
        gcs_threat.src_id = obstacle.src_id
        gcs_threat.threat_level = MAV_COLLISION_THREAT_LEVEL.LOW
        gcs_threat.time_to_closest_approach = 0
        gcs_threat.closest_approach_xy = closest_dist + (ZBR_MARGIN_DYN:get() or 20)
        gcs_threat.closest_approach_z = obstacle_height_difference(obstacle)
        gcs:send_text(MAV_SEVERITY.NOTICE, string.format("Closest: %d(%s) level: %d dist: %.0f", gcs_threat.src_id, get_obstacle_desc(gcs_threat.src_id), gcs_threat.threat_level, gcs_threat.closest_approach_xy))
        --gcs:send_text(0, string.format("margin: %f",margin))
    end
    if margin < 0 then
        -- within the margin, need to avoid
        --gcs:send_text(0, string.format("AVOID to:%d oh:%d t:%d", num_timed_out, num_outside_height_range, obstacle_count))
        --gcs:send_text(0, string.format("margin: %f",margin))
    end
    return margin, obstacle
end

local function calc_avoidance_margin(loc1, loc2, our_velocity, avoid_sec)
    -- By projecting 1m along the line we avoid a problem with the
    -- exclusion avoidance being happy to skirt along a line parallel
    -- to an exclusion zone
    local bearing_deg = math.deg(loc1:get_bearing(loc2))
    local loc1_shifted = location_project(loc1, bearing_deg, 1, loc2)
    local obs_margin, obstacle = obstacle_avoidance_margin(loc1_shifted, our_velocity, avoid_sec)
    return obs_margin, obstacle
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

local function have_collided(current_loc)
    local timeout_ms = 5000
    local ret = false
    local now = millis()

    local obstacle_count = avoid:num_obstacles()
    for i = 0, obstacle_count-1 do
        local obstacle = {}
        obstacle.timestamp_ms = avoid:get_obstacle_timestamp(i)
        obstacle.velocity = avoid:get_obstacle_vel(i)
        obstacle.location = avoid:get_obstacle_loc(i)
        obstacle.src_id = avoid:get_obstacle_id(i)
        -- If the obstacle data is too old, skip it
        if now - obstacle.timestamp_ms > timeout_ms then
            goto continue
        end
        -- Check if the obstacle is within the avoidance height
        if not within_avoidance_height(obstacle, 0, 0) then
            goto continue
        end
        -- Get updated obstacle position
        local obstacle_loc = avoid:get_obstacle_loc(i)
        local obstacle_velocity = Vector2f()
        obstacle_velocity:x(obstacle.velocity:x())
        obstacle_velocity:y(obstacle.velocity:y())
        local dt = (now - obstacle.timestamp_ms):tofloat() * 0.001
        -- Update the obstacle's position using its velocity
        obstacle_loc:offset(obstacle_velocity:x() * dt, obstacle_velocity:y() * dt)   
        local radius = get_avoidance_radius(obstacle)
        local distance = current_loc:get_distance(obstacle_loc)
        -- If the distance to the obstacle is less than its radius, a collision is detected
        if distance < radius and warn_act == 1 then
            --DEBUG(1, string.format("Collided with %u %.0fm", obstacle.src_id, distance))
            gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: Collision", SCRIPT_NAME_SHORT))
            ret = true
        end
        ::continue::
    end

    COLLISION_DETECTED = ret
    return ret
end

--[[
    calculate a new avoidance target location, returning the new location or nil
    avd contains some meta-data on the current state

    takes target_loc as the current (non-avoidance) target
    returns a new target location if avoidance is needed, otherwise nil
--]]
local function update_mission_avoidance(avd, target_loc)
    --gcs:send_text(0, "Started mission avoidance update")
    local airspeed = math.max(avd.airspeed, 1.0)
    local current_loc = avd.current_loc
    current_lookahead = math.min(math.max(current_lookahead, lookahead_param * 0.5), lookahead_param)
    --gcs:send_text(0, "got current lookahead")
    local full_distance = current_loc:get_distance(target_loc)
    --gcs:send_text(0, string.format("obstacle: %.1f m", full_distance))
    -- the distance we look ahead is adjusted dynamically based on avoidance results
    local avoid_step1_m = current_lookahead
    local avoid_step2_m = current_lookahead * 2
    -- test for flying past the waypoint, so if we are close, we have room to dodge after the waypoint
    local avoid_max = math.min(avoid_step1_m, full_distance + math.min(margin_fence / 2, 100))
    local avoid_sec1 = avoid_max / airspeed
    local bearing_inc_cd = 1500
    local distance = airspeed * avoid_sec1
    local bearing_cd =math.deg(current_loc:get_bearing(target_loc))*100
    -- check for collisions, yes we don't actually do anything with this, just report it (if warn_action == 1)
    have_collided(current_loc)
    -- get the current ground course
    local ground_course_deg
    ground_course_deg = wrap_180(math.deg(ahrs:groundspeed_vector():angle()))
    --gcs:send_text(0, string.format("ground_course_deg : %.2f",ground_course_deg ))  

    -- If the full distance is less than 20m, no avoidance is needed
    if full_distance < 20 then
        return nil
    end
    -- Try 5 degree increments around a circle, alternating left and right. Check each one to see if flying in that direction would avoid all obstacles.
    local best_bearing = bearing_cd * 0.01
    local have_best_bearing = false
    local best_margin = -10000.0
    local best_margin_bearing = best_bearing
    local rate_of_turn_dps = math.deg(GRAVITY_MSS * math.tan(math.rad(roll_limit_deg * 0.6)) / (airspeed + 0.1))
    for i = 0, 360 / (bearing_inc_cd / 100) do
        local bearing_delta_cd = i * bearing_inc_cd / 2
        --gcs:send_text(0, string.format("i: %d", i)) 
        if i % 2 == 1 then
            -- Alternate between left and right of the target
            bearing_delta_cd = -bearing_delta_cd
        end    

        -- Test bearing
        local bearing_test = wrap_180((bearing_cd*0.01) + (bearing_delta_cd*0.01))
        local course_change_deg = wrap_180(bearing_test - ground_course_deg)  
        local ground_speed = effective_groundspeed(airspeed, bearing_test, avd.wind_dir_rad, avd.wind_speed)

        if math.abs(course_change_deg) > 170 then
            -- Skip 180-degree turns as we can't predict the turn direction
            goto continue
        end
        -- Calculate how long it will take to change course
        local turn_time_s = math.abs(course_change_deg / rate_of_turn_dps)
        -- Approximate turn by flying forward for half of the turn time
        local projected_loc = location_project(current_loc, ground_course_deg, ground_speed * turn_time_s * 0.5, target_loc)
        -- If turning more than 90 degrees, add sideways movement
        if math.abs(course_change_deg) > 90 then
            local direction = course_change_deg > 0 and (ground_course_deg + 90) or (ground_course_deg - 90)
            local proportion = math.sin(math.rad(math.abs(course_change_deg) - 90))
            projected_loc = location_project(projected_loc, direction, ground_speed * proportion * turn_time_s * 0.5, target_loc)
        end
        -- Position after one step
        local loc_test = location_project(projected_loc, bearing_test, distance, target_loc)
        -- Calculate velocity and margin
        local loc_diff = projected_loc:get_distance_NE(loc_test)
        local our_velocity = Vector2f()
        our_velocity:x(loc_diff:x() / avoid_sec1)
        our_velocity:y(loc_diff:y() / avoid_sec1)
        local margin, _ = calc_avoidance_margin(projected_loc, loc_test, our_velocity, avoid_sec1)
        ---@cast margin number
        --gcs:send_text(0, string.format("Mission avoidance margin: %f", margin))
        if margin > best_margin then
            best_margin_bearing = bearing_test
            best_margin = margin
        end
        -- gcs:send_text(0, string.format("Mission avoidance margin: %.0f, Mission Wide: %.0fx",margin,ZBR_MARGIN_WIDE:get()))
        if margin > (ZBR_MARGIN_WIDE:get() or 30.0) then
            -- This direction avoids all obstacles for one step. Check if it leads to a clear path for a longer distance.
            if not have_best_bearing then
                best_bearing = bearing_test
                have_best_bearing = true
            elseif math.abs(wrap_180(ground_course_deg - bearing_test)) < math.abs(wrap_180(ground_course_deg - best_bearing)) then
                -- Replace with a closer direction
                best_bearing = bearing_test
            end
            local test_bearings = { 0, 45, -45 }
            local target_bearing = math.deg(loc_test:get_bearing(target_loc))
            for _, delta in ipairs(test_bearings) do
                local new_bearing = target_bearing + delta
                local target_distance2 = loc_test:get_distance(target_loc)
                local distance2 = math.min(math.max(avoid_step2_m, 10), target_distance2)
                local avoid_sec2 = distance2 / airspeed
                local loc_test2 = location_project(loc_test, new_bearing, distance2, target_loc)
                local loc_diff2 = loc_test:get_distance_NE(loc_test2)
                local our_velocity2 = Vector2f()
                our_velocity2:x(loc_diff2:x() / avoid_sec2)
                our_velocity2:y(loc_diff2:y() / avoid_sec2)
                local margin2, obstacle2  = calc_avoidance_margin(loc_test, loc_test2, our_velocity2, avoid_sec2)

                if margin2 > (ZBR_MARGIN_WIDE:get() or 30.0) then
                    -- Project the new target in the chosen direction by the full distance
                    local new_loc = location_project(projected_loc, bearing_test, full_distance, target_loc)
                    current_lookahead = math.min(lookahead_param, current_lookahead * 1.1)
                    gcs_action = (i ~= 0 or delta ~= 0) and 1 or 0
                    if i ~= 0 or delta ~= 0 then
                        logger.write("AVDM", 'Res,BCh,M1,M2','ffff',gcs_action, bearing_delta_cd*0.01, margin, margin2)
                        --gcs:send_text(MAV_SEVERITY.INFO, string.format("Avoiding: %d(%s) dist %.0f margin %.0f", obstacle2.src_id, get_obstacle_desc(obstacle2.src_id), obstacle2.distance, margin2))
                        return new_loc
                    end
                    -- no avoidance to be done
                    return nil
                end
            end
        --else
          --  gcs:send_text(0, string.format("No first if"))
        end
        ::continue::
    end
    -- If no good direction was found, choose the best based on margin
    local chosen_bearing
    if have_best_bearing then
        chosen_bearing = best_bearing
        current_lookahead = math.min(lookahead_param, current_lookahead * 1.05)
        gcs_action = MAV_COLLISION_ACTION.ASCEND_OR_DESCEND
        logger.write("AVDM", 'Res,BCh,M1,M2','ffff',gcs_action, wrap_180(best_bearing - (bearing_cd*0.01)), best_margin, -1)
    else
        chosen_bearing = best_margin_bearing
        current_lookahead = math.max(lookahead_param * 0.5, current_lookahead * 0.9)
        gcs_action = MAV_COLLISION_ACTION.MOVE_HORIZONTALLY
        logger.write("AVDM", 'Res,BCh,M1,M2','ffff',gcs_action, wrap_180(best_bearing - (bearing_cd*0.01)), best_margin, -1)
    end
    -- Calculate new target location based on the best effort
    gcs:send_text(MAV_SEVERITY.INFO, string.format("AVOID - no good direction so best bearing %.0f dist %.1f", chosen_bearing, full_distance))
    return location_project(current_loc, chosen_bearing, full_distance, target_loc)
end

--[[
    calculate and return an adjusted target location or return nil
    for no avoidance needed
--]]
local function mission_avoidance(current_loc, target_loc, groundspeed)
    --[[
        fill in avoidance_result with current state
    --]]
    local now = millis()    -- Populate avoidance request with current data
    avoidance_request.current_loc = current_loc
    avoidance_request.target_loc = target_loc:copy()
    -- all altitude calculations should be done in absolute frame
    avoidance_request.target_loc:change_alt_frame(ALT_FRAME.ABSOLUTE)
    avoidance_request.groundspeed = groundspeed
    -- Estimate airspeed (use groundspeed if no airspeed estimate available)
    avoidance_request.airspeed = ahrs:airspeed_estimate() or groundspeed

    -- Get wind estimate and convert to 2D
    local wind_3d = ahrs:wind_estimate()
    local wind_2d = Vector2f()
    wind_2d:x(wind_3d:x())
    wind_2d:y(wind_3d:y())

    -- Calculate wind direction and speed
    avoidance_request.wind_dir_rad = wind_2d:angle()
    avoidance_request.wind_speed = wind_2d:length()
    -- Record request time
    avoidance_request.request_time_ms = now

    -- call the main mission avoidance logic
    return update_mission_avoidance(avoidance_request, target_loc)
end

local orig_target_loc = nil
local last_updated_target_loc = nil

--[[
    update avoidance at ZBR_UPDATE_RATE Hz. Calls mission_avoidance() and
    updates target location in the vehicle code
--]]
local function update()
    if AVD_ENABLE:get() == 0 then
        -- disabled by the user
        gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: AVD_ENABLE = 0", SCRIPT_NAME_SHORT))
        return
    end
    if not arming:is_armed() then
        -- no need to do avoidance while disarmed
        return
    end

    local current_loc = ahrs:get_position()
    local current_target_loc = vehicle:get_target_location()

    if current_loc == nil or current_target_loc == nil then
        -- no position or not navigating
        orig_target_loc = nil
        last_updated_target_loc = nil
        gcs:send_text(MAV_SEVERITY.INFO, "NOT NAVIGATING")
        return
    end
    local current_target_save_loc = current_target_loc:copy()

    if orig_target_loc == nil or last_updated_target_loc == nil or
        not locations_equal(last_updated_target_loc, current_target_loc) then
        -- the vehicle navigation code has changed it's target
        orig_target_loc = current_target_loc:copy()
    end

    local groundspeed = ahrs:groundspeed_vector():length()

    -- run avoidance calculation
    --gcs:send_text(MAV_SEVERITY.INFO, "---- avoidance calc ----")
    --gcs:send_text(MAV_SEVERITY.INFO, string.format(" LAT orig %d", orig_target_loc:lat()))
    --gcs:send_text(MAV_SEVERITY.INFO, string.format(" LNG orig %d", orig_target_loc:lng()))
    --gcs:send_text(MAV_SEVERITY.INFO, string.format(" ALT orig %d frame %d", orig_target_loc:alt(), orig_target_loc:get_alt_frame()))
    local new_target_loc = mission_avoidance(current_loc, orig_target_loc, groundspeed)
    --gcs:send_text(MAV_SEVERITY.INFO, string.format(" ALT orig %d frame %d", orig_target_loc:alt(), orig_target_loc:get_alt_frame()))
    if new_target_loc == nil then
        -- if avoidance doesn't return a new target - revert to the original target
        new_target_loc = orig_target_loc
    end
    -- if we have a new target - update it if it's different from our current target otherwise revert
    if not locations_equal(current_target_save_loc, new_target_loc) then
        -- make sure the frame of the new target is the same as the original frame
        new_target_loc:change_alt_frame(orig_target_loc:get_alt_frame())
        local updated_location = vehicle:update_target_location(current_target_save_loc, new_target_loc)
        if updated_location then
            last_updated_target_loc = new_target_loc:copy()
            local avoid_dist = orig_target_loc:get_distance(new_target_loc)
            gcs:send_named_float("AVOIDING - DIST", avoid_dist)
            gcs:send_text(MAV_SEVERITY.ERROR, "BR: AVOIDING: dist: " .. avoid_dist .. " alt: " .. new_target_loc:alt() .. ":".. new_target_loc:get_alt_frame())
        else
            --gcs:send_text(MAV_SEVERITY.ERROR, "BR: AVOIDING: updated: FALSE")
        end
    end
    return

    --[[if new_target_loc ~= nil then
        -- tell the vehicle to fly to the new calculated target
        local updated_location = vehicle:update_target_location(current_target_save_loc, new_target_loc)
        --local updated_location = vehicle:update_target_location(orig_target_loc, new_target_loc)

        if updated_location then
            last_updated_target_loc = new_target_loc:copy()
            local avoid_dist = orig_target_loc:get_distance(new_target_loc)
            gcs:send_named_float("AVOIDING - DIST", avoid_dist)
        else
            gcs:send_text(MAV_SEVERITY.ERROR, "BR: AVOIDING: updated: FALSE")
            if orig_target_loc:lat() ~= new_target_loc:lat() then
                gcs:send_text(MAV_SEVERITY.ERROR, string.format(" LAT diff: old %d new %d", orig_target_loc:lat(), new_target_loc:lat()))
            end
            if orig_target_loc:lng() ~= new_target_loc:lng() then
                gcs:send_text(MAV_SEVERITY.ERROR, string.format(" LNG diff: old %d new %d", orig_target_loc:lng(), new_target_loc:lng()))
            end
            if orig_target_loc:alt() ~= new_target_loc:alt() then
                gcs:send_text(MAV_SEVERITY.ERROR, string.format(" ALT diff: old %d (%d) new %d (%d)",
                        orig_target_loc:alt(), orig_target_loc:get_alt_frame(),
                        new_target_loc:alt(), new_target_loc:get_alt_frame()))
            end
        end
    else
        -- revert to the original target location, no avoidance needed
        local updated_location = vehicle:update_target_location(current_target_save_loc, orig_target_loc)
        if updated_location then
            last_updated_target_loc = orig_target_loc:copy()
            gcs:send_named_float("AVD_DIST", -1)
        else
            gcs:send_text(MAV_SEVERITY.ERROR, "BR: REVERTING: updated: FALSE")
        end
    end
    --]]
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
    return protected_wrapper, 1000 * refresh_rate
end

 local function delayed_startup()
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
    --[[if avoid == nil then
        gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: AP_Avoidance singleton not available", SCRIPT_NAME_SHORT))
        return
    end
    --]]

    return protected_wrapper()
 end

-- start running update loop - waiting 20s for the AP to initialize
if FWVersion:type() == 3 then
    if arming:is_armed() then
       return delayed_startup, 1000
    else
       return delayed_startup, 20000
    end
else
    gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: must run on Plane", SCRIPT_NAME_SHORT))
end
