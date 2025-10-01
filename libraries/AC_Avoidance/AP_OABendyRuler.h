#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_Common/Location.h>

class AP_OABendyRuler {
public:
    // Define OABendyType INSIDE the class
// Use the existing OABendyType enum
    enum class OABendyType {
        OA_BENDY_DISABLED   = 0,
        OA_BENDY_HORIZONTAL = 1,
        OA_BENDY_VERTICAL   = 2,
        OA_BENDY_OBJECT     = 3,
    };

    // Define the result struct INSIDE the class
    struct OABendyResult {
        Vector2f avoidance_vec;
        Vector2f origin;
        float nearest_distance;
        OABendyType bendy_type;  // Use the enum type
        
        void reset() {
            avoidance_vec.zero();
            origin.zero();
            nearest_distance = 0;
            bendy_type = OABendyType::OA_BENDY_DISABLED;
        }
    };

    AP_OABendyRuler();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_OABendyRuler);

    static AP_OABendyRuler *get_singleton(void) { return _singleton; }

    // init - perform any required initialisation
    void init();

    // pre-calculation that applies to all avoidance threads
    void pre_update();

    // main update function to find new path
    bool update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new, OABendyResult &bendy_result, bool proximity_only);

    // set fence system
    void set_fence(const AC_Fence* fence) { _fence = fence; }

    // configure the BendyRuler
    void set_config(float margin_max);

    // Logging method
    void Write_OABendyRuler(const uint8_t type, const bool active, const float target_yaw, const float target_pitch, const bool resist_chg, const float margin, const Location &final_dest, const Location &oa_dest) const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_OABendyRuler *_singleton;

    // Spatial hash for ray tracing optimization
    static const uint16_t OA_MAX_OBSTACLES = 256;
    static const uint16_t OA_MAX_FENCE_SEGMENTS = 100;
    static const uint16_t OA_BITMASK_SIZE = (OA_MAX_OBSTACLES + 31) / 32;

    class AP_OASpatialHash {
    private:
        static const uint16_t OA_SPATIAL_HASH_SIZE = 64;
        static const uint16_t OA_MAX_OBSTACLES_PER_CELL = 10;
        
        struct OACell {
            uint16_t obstacle_count;
            uint16_t obstacle_indices[OA_MAX_OBSTACLES_PER_CELL];
        };
        
        float _cell_size;
        OACell _cells[OA_SPATIAL_HASH_SIZE][OA_SPATIAL_HASH_SIZE];
        Vector2f _grid_origin;
        
        bool _world_to_grid(const Vector2f& pos, uint16_t& grid_x, uint16_t& grid_y) const;
        
    public:
        void init(float cell_size, const Vector2f& origin);
        void clear();
        bool add_obstacle(const Vector2f& pos, float radius, uint16_t obstacle_id);
        void query_radius(const Vector2f& pos, float radius, uint32_t obstacle_mask[OA_BITMASK_SIZE]);
        bool ray_intersect_dda(const Vector2f& start, const Vector2f& end, float& distance, uint16_t& obstacle_id);
        bool obstacle_in_mask(const uint32_t obstacle_mask[OA_BITMASK_SIZE], uint16_t obstacle_id) const;
    };

    // Parameters
    AP_Float _lookahead;            // object avoidance will look this many meters ahead of vehicle
    AP_Float _bendy_ratio;          // object avoidance will avoid major directional change if change in margin ratio is less than this param
    AP_Int16 _bendy_angle;          // object avoidance will try avoiding change in direction over this much angle
    AP_Int8  _bendy_type;           // Type of BendyRuler to run

    // Fence integration
    const AC_Fence* _fence;
    bool _fences_loaded;
    uint32_t _last_fence_update_ms;

    // Fence segment cache
    struct OAFenceSegment {
        Vector2f start;
        Vector2f end;
        uint8_t fence_type;
        uint8_t fence_instance;
    };
    
    uint16_t _fence_segment_count;
    OAFenceSegment _fence_segments[OA_MAX_FENCE_SEGMENTS];

    // Spatial hash for ray tracing
    AP_OASpatialHash _spatial_hash;
    bool _spatial_hash_initialized;
    uint32_t _last_spatial_hash_update_ms;

    // Methods
    bool get_origin_and_direction(const Location& current_loc, const Vector2f &ground_speed_vec, Vector2f &origin, Vector2f &direction) WARN_IF_UNUSED;
    bool _check_segment_with_raytrace(const Vector2f& start, const Vector2f& end, OABendyResult &result) WARN_IF_UNUSED;
    bool _find_nearest_intersection(const Vector2f& start, const Vector2f& end, float& distance, OABendyType& obstacle_type) WARN_IF_UNUSED;
    void _update_spatial_hash();
    bool _load_fence_segments();
    bool _ray_intersects_fence_segment(const Vector2f& start, const Vector2f& end, const OAFenceSegment& segment, Vector2f& intersection) const WARN_IF_UNUSED;
    bool check_collision_with_fences(const Vector2f& start, const Vector2f& end, float& intersection_dist, uint32_t fence_mask[OA_BITMASK_SIZE]) WARN_IF_UNUSED;
    bool location_to_vector(const Location& loc, Vector2f& pos) const;
};

namespace AP {
    AP_OABendyRuler *ap_oabendyruler();
};