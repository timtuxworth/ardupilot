#pragma once


#if AP_OAPATHPLANNER_ENABLED
#if AP_SCRIPTING_ENABLED

#include "AC_Avoidance_config.h"
#include "AP_OADatabase.h"
#include <AC_Fence/AC_Fence.h>
#include <AP_Avoidance/AP_Avoidance.h>

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include "AP_OAVisGraph.h"
#include <AP_Logger/AP_Logger_config.h>
#include <GCS_MAVLink/GCS.h>

class AP_OAScripting {
public:

    AP_OAScripting();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_OAScripting);

    static AP_OAScripting *get_singleton() {
        return _singleton;
    }

    // enum of obstacle types for passing back to Lua
    enum class ObstacleType : uint16_t {
        GENERAL                     = 0,
        MAV_SYSID                   = 1,
        GENERAL_AVIATION            = 2,
        WEATHER                     = 3,
        BIRD_MIGRATORY              = 4,
        BIRD_OF_PREY                = 5,
        FENCE_HOME                  = 6,
        FENCE_CIRCLE_INCLUSION      = 7,
        FENCE_CIRCLE_EXCLUSION      = 8,
        FENCE_POLYGON_INCLUSION     = 9,
        FENCE_POLYGON_EXCLUSION     = 10,
        FENCE_LUA                   = 11,
        PROXIMITY                   = 12,
        AIS                         = 13,
    };

    struct Obstacle {
        uint32_t                    timestamp_ms;
        uint32_t                    src_id;         // The AP_Avoid src_id
        uint32_t                    sysid;          // The MAV_SYSID of the target -1 if n/a
        ObstacleType                type;
        char                        *label;
        Vector3f                    velocity_NED_ms;
        Location                    location;
        Vector3f                    pos_NED_m;
    };

    // This function is for Lua, so each Obstacle field gets returned as multiple parameters
    bool find_closest_obstacle(const Location &start_loc, const Location &end_loc, float lookahead_m,
                                                // Return values
                                                float    &distance_min_m, 
                                                uint16_t &type, 
                                                char    *&label, 
                                                uint32_t &sys_id,
                                                Location &location, 
                                                Vector3f &pos_NED_m,
                                                Vector3f &velocity_NED_ms
                                                ) const;
    
    // This function is for Lua, so each Obstacle field gets returned as multiple parameters
    bool distance_obstacle_test(const Location &start_loc, const Location &end_loc, const float &lookahead_m, float &distance_min_m, Location &location_out) const;
    //bool distance_obstacle_test2(const float &lookahead_m, float &distance_min_m) const;

private:

    static AP_OAScripting *_singleton;

    float _distance_to_avoidance(const Vector3f &start_NED_cm, const Vector3f &end_NED_cm, Obstacle &script_obstacle) const;
    float _distance_to_object(const Vector3f &start_NED_m, const Vector3f end_NED_m, Obstacle &script_obstacle) const;

    ObstacleType _get_obstacle_type(int32_t obstacle_id) const;
    char* _get_obstacle_label(int32_t obstacle_id) const;

    // Properties to work around Lua binding problem of the binding generator not being able
    // to pass in Locations and return a number of other values
    Location    _lua_start_loc;
    Location    _lua_end_loc;

};

#endif
#endif
