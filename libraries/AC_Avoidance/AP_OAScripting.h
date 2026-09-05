#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include "AC_Avoidance_config.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Avoidance/AP_Avoidance_config.h>

// AP_OA_SCRIPTING_ENABLED is defined in AP_Avoidance_config.h, included above, so
// that this library and the AP_Avoidance queries and AVD_ parameters it depends on
// compile in and out as one unit.  It is ALWAYS defined (to 0 or 1), so
// "#if AP_OA_SCRIPTING_ENABLED" is valid under -Werror=undef wherever this header
// is included.

#if AP_OA_SCRIPTING_ENABLED

#include "AP_OADatabase.h"
#include <AC_Fence/AC_Fence.h>
#include <AP_Avoidance/AP_Avoidance.h>

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include "AP_OAVisGraph.h"
#include <AP_Logger/AP_Logger_config.h>
#include <GCS_MAVLink/GCS.h>

// Forward declare the class and its nested struct cleanly
class AP_Avoidance;

struct OAObstacle;

class AP_OAScripting {
public:

    AP_OAScripting();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_OAScripting);

    static AP_OAScripting *get_singleton() {
        return _singleton;
    }

    // enum of obstacle types for passing back to Lua
    enum class ObstacleType : uint8_t {
        GENERAL                     = 0,
        MAV_SYSID                   = 1,
        // Named AIRCRAFT, not "ADSB", on purpose: the source being ADS-B does not
        // fix the type. An ADS-B contact with a UAV emitter is a drone (MAV_SYSID above),
        // so "ADSB" would span two ObstacleTypes. This one is specifically crewed aircraft.
        CREWED_AIRCRAFT             = 2,
        FENCE_HOME                  = 3,
        FENCE_CIRCLE_INCLUSION      = 4,
        FENCE_CIRCLE_EXCLUSION      = 5,
        FENCE_POLYGON_INCLUSION     = 6,
        FENCE_POLYGON_EXCLUSION     = 7,
        FENCE_LUA                   = 8,
        PROXIMITY                   = 9,
        AIS                         = 10,
        FENCE_ALT_MAX               = 11,   // max altitude fence (AC_FENCE_TYPE_ALT_MAX, FENCE_TYPE bit 0)
        FENCE_ALT_MIN               = 12,   // min altitude fence (AC_FENCE_TYPE_ALT_MIN, FENCE_TYPE bit 3)
    };



    // Returns the single closest obstacle to the line, whichever source it came from:
    // ADS-B objects from AP_Avoidance
    // Proximity objects from AP_OADatabase
    // Fences from AC_Fence
    // Crewed aircraft are found separately by find_aircraft(), which applies the
    // caller's vertical gate rather than the per-emitter table.
    bool find_threats(const Location &start_loc, const Location &end_loc, float lookahead_m,
                                                // Return values
                                                float       &distance_m,
                                                OAObstacle  &any_obstacle
                                                ) const;

    // As find_threats(), but fences only - never masked by a moving obstacle (ADS-B/
    // MAVLink traffic, AP_OADatabase objects) that happens to be closer along the same
    // segment. Shares find_threats()'s line-query fence checks (_find_fence_threats_NE
    // below) so a fence-only caller sees the same clearance-along-the-whole-segment
    // answer, not just the endpoint.
    bool find_fence_threats(const Location &start_loc, const Location &end_loc, float lookahead_m,
                                                // Return values
                                                float       &distance_m,
                                                OAObstacle  &any_obstacle
                                                ) const;

    bool find_aircraft(const Location &vehicle_loc, const float lookahead_m, const float vertical_lookahead_m,
                                    float       &distance_m,
                                    OAObstacle  &aircraft_obstacle
                            ) const;

    // closest distance (metres) from a location to the nearest fence boundary edge.  For Lua so
    // the AVOIDING message can report a real distance to a fence obstacle (which, unlike ADS-B
    // point obstacles, has no single usable "location").  fence_type scopes the search to the
    // matching fence category (an AP_OAScripting::ObstacleType FENCE_* value) so the reported
    // distance belongs to the same kind of fence the message names; pass 0 (GENERAL) or any
    // non-fence value to search all polygon/circle fences.  Returns true and sets distance_m if a
    // matching polygon/circle fence is found, false otherwise.
    bool fence_distance(const Location &loc, uint8_t fence_type, float &distance_m) const;

    // Breach-inclusive twin of find_fence_threats(): a fence AC_Fence itself has flagged as
    // breached is NOT skipped here, unlike find_threats()/find_fence_threats() (see
    // _find_fence_threats_NE()'s own comment for why they skip it - that is still correct
    // for ordinary threat dispatch). This exists for recovery scoring while a breach is
    // active or being approached, where the true (possibly negative) clearance is exactly
    // the signal needed to tell an exiting path from one still penetrating.
    //
    // Returns two values, not one: min_path_m is the worst clearance found anywhere along
    // the whole start_loc->end_loc segment (as find_fence_threats() would report, minus the
    // breach skip); endpoint_m is the clearance at end_loc alone, from that same governing
    // fence category. Every recovery candidate shares its starting position, so a shared,
    // possibly-deep-negative start point dominates a segment-minimum comparison identically
    // for every candidate; the endpoint is what actually differs between an improving and a
    // worsening heading. start_loc == end_loc is a valid point-only query (no early-return
    // guard, unlike find_fence_threats()) - it is how a caller gets "current clearance".
    bool find_fence_clearance(const Location &start_loc, const Location &end_loc, float lookahead_m,
                                                // Return values
                                                float       &min_path_m,
                                                float       &endpoint_m,
                                                OAObstacle  &any_obstacle
                                                ) const;


private:

    static AP_OAScripting *_singleton;

    float _distance_to_avoidance(const Vector3f &start_NED_cm, const Vector3f &end_NED_cm, OAObstacle &script_obstacle) const;
    // Shared by find_threats() and find_fence_threats(): the fence-only portion of the
    // search (every polygon/circle fence category), taking NE offsets in CENTIMETRES
    // (the fence loader's native units) rather than Locations, since both callers
    // already have to do that conversion once for their own start/end points.
    float _find_fence_threats_NE(const Vector2f &start_NE_cm, const Vector2f &end_NE_cm, float lookahead_m, OAObstacle &obstacle) const;
    // Shared by find_fence_clearance() only - see its own header comment for why this is a
    // separate function rather than a bool argument to _find_fence_threats_NE(): a breach
    // is never skipped here, and an extra endpoint_m value is returned alongside.
    float _find_fence_clearance_NE(const Vector2f &start_NE_cm, const Vector2f &end_NE_cm, float lookahead_m, float &endpoint_m, OAObstacle &obstacle) const;
#if AP_OA_SCRIPTING_OADB_ENABLED
    float _distance_to_object(const Vector3f &start_NED_m, const Vector3f end_NED_m, OAObstacle &script_obstacle) const;
#endif
    float _distance_to_aircraft(const Vector3f &vehicle_NED_cm, const float lookahead_m, const float vertical_lookahead_m, OAObstacle &script_obstacle) const;

    // create a "Scripting Obstacle" to easily pass info about an obstacle to Lua
    static void _populate_scripting_obstacle(OAObstacle &scripting_obstacle, const AP_Avoidance::Obstacle *avoidance_obstacle);
    static void _populate_fence_obstacle(OAObstacle &fence_obstacle, AP_OAScripting::ObstacleType obstacle_type);

    static ObstacleType _get_obstacle_type(uint8_t emitter_type, int32_t obstacle_id);

    // Properties to work around Lua binding problem of the binding generator not being able
    // to pass in Locations and return a number of other values
    Location    _lua_start_loc;
    Location    _lua_end_loc;

    // parameters
};

struct OAObstacle {
    public:
    uint32_t                    timestamp_ms;
    uint32_t                    src_id;         // The AP_Avoid src_id
    uint32_t                    icao_code;      // The ICAO code (if relevant) from ADSB
    uint8_t                     emitter_type;   // The ADSB_EMITTER of the obstacle (if relevant)
    uint8_t                     obstacle_type;

    Vector3f                    velocity_NED_ms;
    Location                    location;
    Vector3f                    position_NED_m;
};

#endif // AP_OA_SCRIPTING_ENABLED
