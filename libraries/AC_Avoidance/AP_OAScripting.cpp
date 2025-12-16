/*
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
 */

#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_ENABLED 
#if AP_SCRIPTING_ENABLED

#include "AP_OAScripting.h"

#if AP_FENCE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#endif // AP_FENCE_ENABLED

/// Constructor
AP_OAScripting::AP_OAScripting()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Can only be one AP_OAScripting");
    }
    _singleton = this;
    //AP_Param::setup_object_defaults(this, var_info);
}

// singleton instance
AP_OAScripting *AP_OAScripting::_singleton;

bool AP_OAScripting::distance_obstacle_test(const Location &start_loc, const Location &end_loc, const float &lookahead_m, float &distance_m, Location &location_out) const
{
    distance_m = lookahead_m;
    return true;
}
/*bool AP_OAScripting::distance_obstacle_test2(const float lookahead_m, float &distance_m) const
{
    distance_m = lookahead_m;
    return true;
}*/

/*bool AP_OAScripting::distance_to_obstacle_full(const Location &start_loc, const Location &end_loc, float lookahead_m,
                                                // Return values
                                                float    &distance_min_m, 
                                                uint16_t &type, 
                                                char    *&label, 
                                                uint32_t &sys_id,
                                                Location &location, 
                                                Vector3f &pos_NED_m,
                                                Vector3f &velocity_NED_ms
                                                )
{
    if(find_closest_obstacle_stash(start_loc, end_loc)) {
        return find_closest_obstacle(distance_min_m, type, label, sys_id, location, pos_NED_m, velocity_NED_ms);
    }
    return false;
}
// Fudge to Lua binding issue - 1st call stashes values used in the 2nd call which only returns them
bool AP_OAScripting::find_closest_obstacle_stash(const Location &start_loc, const Location &end_loc)
{
    _lua_start_loc   = start_loc;
    _lua_end_loc     = end_loc;

    return true;
}*/

// Lua binding to be used in Object Detection to find the neareast fence, ADS-B object, or proximity obstacle
//   the use of the word "obstacle" is intended to be generic, unfortunately one of the cases of ArduPilot is "Obstacles" stored in the AP_OAAvoidance library
// Note that the distance is the distance to any margin around the obstacles. AP_OAAvoidance obstacles have this as do fences. So the distance can be negative if you are too close.
bool AP_OAScripting::find_closest_obstacle(const Location &start_loc, const Location &end_loc, float lookahead_m,
                                            float &distance_m, 
                                            uint16_t &type, 
                                            char *&label, 
                                            uint32_t &sysid,
                                            Location &location, 
                                            Vector3f &pos_NED_m,
                                            Vector3f &velocity_NED_ms
                                            ) const
{
    float distance_new_m = FLT_MAX;
    Obstacle obstacle;

    //Location start_loc  = _lua_start_loc;
    //Location end_loc    = _lua_end_loc;

    distance_m = lookahead_m;

    // convert start and end to offsets from EKF origin (waiting for NEU/NED changes)
    Vector3f start_NED_m,end_NED_m;
    if (!start_loc.get_vector_from_origin_NEU_m(start_NED_m) ||
        !end_loc.get_vector_from_origin_NEU_m(end_NED_m)) {
        return false;
    }
    if (start_NED_m == end_NED_m) {
        return false;
    }

    // "obstacles" are stored in AP_Avoidance - the are typically populated by MAVLink (ADSB, GLOBAL_POSITION, FOLLOW_TARGET)
    // These have priority over all other obstacles, especially if they are ADSB messages representing crude aircraft
    Obstacle obstacle_found;
    distance_new_m = _distance_to_avoidance(start_NED_m, end_NED_m, obstacle_found);
    if (distance_new_m < distance_m) {
        obstacle = obstacle_found;
        distance_m  = distance_new_m;

        if(!obstacle.location.initialised()) {
            // This should NEVER happen
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LOCATION NIL src_id %d", obstacle.src_id);
        }
    }

    // "objects" are stored in the AP_OADatabase - they are typically populated by proximity sensors
    distance_new_m = _distance_to_object(start_NED_m, end_NED_m, obstacle_found); 
    if (distance_new_m < distance_m) {
        obstacle    = obstacle_found;
        distance_m = distance_new_m;
        
        if(!obstacle.location.initialised()) {
            // This should NEVER happen
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "LOCATION NIL src_id %d", obstacle.src_id);
        }
    }

#ifdef AP_FENCE_ENABLED
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence != nullptr) {
        // fences use cm (for now), so do this once
        //const Vector3f start_NED_cm = start_NED_m * 100.0f;
        //const Vector3f end_NED_cm = end_NED_m * 100.0f;
        const Vector2f start_NE_cm(start_NED_m.x * 100.0f, start_NED_m.y * 100.0f);
        const Vector2f end_NE_cm(end_NED_m.x * 100.0f, end_NED_m.y * 100.0f);

        // We do each type of fence one at a time, because a. they are stored in separate lists and b. we want to tell the user which fence it is
        distance_new_m = fence->distance_line_to_home_inclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            obstacle.type   = ObstacleType::FENCE_HOME;
            obstacle.label  = (char *)"tin can";
            distance_m      = distance_new_m;
        }
        distance_new_m = fence->distance_line_to_circle_inclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            obstacle.type   = ObstacleType::FENCE_CIRCLE_INCLUSION;
            obstacle.label  = (char *)"circle inc.";
            distance_m      = distance_new_m;
        }
        distance_new_m = fence->distance_line_to_circle_exclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            obstacle.type   = ObstacleType::FENCE_CIRCLE_EXCLUSION;
            obstacle.label  = (char *)"circle exc.";
            distance_m      = distance_new_m;
        }
        distance_new_m = fence->distance_line_to_polygon_inclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            obstacle.type   = ObstacleType::FENCE_POLYGON_INCLUSION;
            obstacle.label  = (char *)"poly inc.";
            distance_m      = distance_new_m;
        }
        distance_new_m = fence->distance_line_to_polygon_exclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            obstacle.type   = ObstacleType::FENCE_POLYGON_EXCLUSION;
            obstacle.label  = (char *)"poly exc.";
            distance_m      = distance_new_m;
        }
    }
#endif
    if (distance_m < lookahead_m) {
        type            = (uint16_t)obstacle.type;
        label           = obstacle.label;
        sysid           = obstacle.sysid;
        velocity_NED_ms = obstacle.velocity_NED_ms;
        if (obstacle.location.initialised()) {
            location        = obstacle.location;
            location.get_vector_from_origin_NEU_m(pos_NED_m);
        }
        return true;
    }

    return false;
}

// Distance to objects in the AP_OADatabase
float AP_OAScripting::_distance_to_object(const Vector3f &start_NED_m, const Vector3f end_NED_m, Obstacle &script_obstacle) const
{
    float distance_new_m = FLT_MAX;
    
    // exit immediately if db is empty
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy() || oaDb->database_count() == 0) {
        return distance_new_m;
    }

    for (uint16_t i=0; i < oaDb->database_count(); i++) {
        const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);
        // result is distance between line segment and obstacle minus obstacle's radius
        const float distance_m = Vector3f::closest_distance_between_line_and_point(start_NED_m, end_NED_m, item.pos) - item.radius;
        if (distance_m < distance_new_m) {
            distance_new_m = distance_m;
            Vector3p item_pos_3p(item.pos.x, item.pos.y, item.pos.z);
            if (AP::ahrs().get_origin(script_obstacle.location)) {
                script_obstacle.location.offset(item_pos_3p);
            }
            switch(item.source) {
            case AP_OADatabase::OA_DbItem::Source::proximity:
                script_obstacle.label = (char *)"proximity";
                script_obstacle.type = ObstacleType::PROXIMITY;
                break;
            case AP_OADatabase::OA_DbItem::Source::AIS:
                script_obstacle.label = (char *)"AIS";
                script_obstacle.type = ObstacleType::AIS;
                break;
            }
            script_obstacle.src_id          = -1;
            script_obstacle.sysid           = -1;
        }
    }

    return distance_new_m;
}

// translate an AP_Avoidance obstacle src_id into a string for display purposes
char* AP_OAScripting::_get_obstacle_label(int32_t obstacle_id) const
{
    if (obstacle_id < 256) {
        return (char *)"MAV";
    }
    else if (obstacle_id < 20000) {
        // fixed wing, 300m radius
        return (char *)"GA";
    }
    else if (obstacle_id < 30000) {
        // weather, radius 150 at ground, 300m at 3000m, 173m at 1500ft
        return (char *)"Weather";
    }
    else if (obstacle_id < 40000) {
        // migratory bird, 100m
        return (char *)"Migratory Bird";
    }
    else if (obstacle_id < 50000) {
        // bird of prey, 200m
        return (char *)"Bird of Prey";
    }
    //default to 300, which is worst case
    return (char *)"Unknown";
}

// translate an AP_Avoidance obstacle src_id into an enum for further processing in Lua
AP_OAScripting::ObstacleType AP_OAScripting::_get_obstacle_type(int32_t obstacle_id) const
{
    if (obstacle_id < 256) {
        return ObstacleType::MAV_SYSID;
    }
    else if (obstacle_id < 20000) {
        return ObstacleType::GENERAL_AVIATION;
    }
    else if (obstacle_id < 30000) {
        return ObstacleType::WEATHER;
    }
    else if (obstacle_id < 40000) {
        return ObstacleType::BIRD_MIGRATORY;
    }
    else if (obstacle_id < 50000) {
        return ObstacleType::BIRD_OF_PREY;
    }
    return ObstacleType::GENERAL;
}

// Distance to objects in the AP_Avoidance database
float AP_OAScripting::_distance_to_avoidance(const Vector3f &start_NED_cm, const Vector3f &end_NED_cm, 
                                                // return values
                                                Obstacle &script_obstacle
                                                ) const
{
    AP_Avoidance *avoid = AP_Avoidance::get_singleton();
    AP_Avoidance::Obstacle avoid_obstacle;

    float distance_m = avoid->distance_to_obstacle(start_NED_cm, end_NED_cm, avoid_obstacle);
    if (distance_m < FLT_MAX) {
        script_obstacle.timestamp_ms    = avoid_obstacle.timestamp_ms;
        script_obstacle.type            = _get_obstacle_type(avoid_obstacle.src_id);
        script_obstacle.src_id          = avoid_obstacle.src_id;
        script_obstacle.sysid           = (script_obstacle.type == ObstacleType::MAV_SYSID) ? avoid_obstacle.src_id : -1;
        script_obstacle.label           = _get_obstacle_label(avoid_obstacle.src_id);
        script_obstacle.velocity_NED_ms = avoid_obstacle._velocity_ned_ms;
        script_obstacle.location        = avoid_obstacle._location;
        if(script_obstacle.location.initialised()) {
            script_obstacle.location.get_vector_from_origin_NEU_m(script_obstacle.pos_NED_m);
        }
    }
    return distance_m;
}

#endif // AP_SCRIPTING_ENABLED
#endif // AP_OAPATHPLANNER_ENABLED