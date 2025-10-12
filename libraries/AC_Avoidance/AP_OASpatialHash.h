#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>

class AP_OASpatialHash {
public:
    AP_OASpatialHash();
    
    // Enhanced obstacle types
    enum ObstacleType {
        OBSTACLE_DATABASE = 0,
        FENCE_INCLUSION_POLYGON = 1,
        FENCE_EXCLUSION_POLYGON = 2, 
        FENCE_INCLUSION_CIRCLE = 3,
        FENCE_EXCLUSION_CIRCLE = 4
    };

    struct Obstacle {
        ObstacleType type;
        Vector3f pos;           // Position in meters from origin (NED)
        Vector3f pos2;          // Second position for line segments (NED)
        float radius;           // For circles and point obstacles
        uint32_t timestamp_ms;
    };

    // Initialize with origin location
    // void set_origin(const Location &origin);
    
    // Clear all obstacles
    void clear();
    
    // Insert methods for different obstacle types
    bool insert_database_obstacle(const Vector3f &new_pos, float radius, uint32_t timestamp_ms);
    bool insert_fence_inclusion_polygon(const Vector3f &start_NEU_m, const Vector3f &end_NEU_m)     { return _insert_line_obstacle(start_NEU_m, end_NEU_m, FENCE_INCLUSION_POLYGON);};
    bool insert_fence_exclusion_polygon(const Vector3f &start_NEU_m, const Vector3f &end_NEU_m)     { return _insert_line_obstacle(start_NEU_m, end_NEU_m, FENCE_EXCLUSION_POLYGON);};
    bool insert_fence_inclusion_circle(const Vector3f &center_NEU_m, float radius);
    bool insert_fence_exclusion_circle(const Vector3f &center_NEU_m, float radius);
    
    // Find closest obstacle distance
    float find_closest_obstacle_distance(const Location &loc, float max_search_radius) const;
    
    // Get number of items in spatial hash
    uint16_t get_num_items() const { return _num_obstacles; }

private:
    static const uint16_t OA_SPATIAL_HASH_BUCKETS = 1024;
    static const uint16_t OA_SPATIAL_HASH_POOL_SIZE = 512;
    static constexpr float GRID_CELL_SIZE = 50.0f; // 50 meter grid cells

    // Search radius configuration
    static constexpr uint8_t SEARCH_SAFETY_MARGIN_PERCENT = 140;  // 40% safety margin (1.4x = 140%)
    static constexpr uint8_t MIN_SEARCH_RADIUS_CELLS = 3;         // Derived: 150m / 50m per cell = 3 cells
    static constexpr uint8_t MAX_SEARCH_RADIUS_CELLS = 10;        // Derived: 500m / 50m per cell = 10 cells
    static constexpr uint16_t PERCENT_DIVISOR = 100;              // For percentage calculations (140% = 140/100)
    static constexpr uint16_t CM_PER_METER = 100;                 // Centimeters per meter (unit conversion)

    struct HashKey {
        int32_t x;
        int32_t y;
        bool operator==(const HashKey &other) const {
            return x == other.x && y == other.y;
        }
    };

    struct BucketItem {
        Obstacle obstacle;
        BucketItem *next;
    };

    // Convert Location to NEU coordinates relative to origin
    Vector3f _location_to_neu(const Location &loc) const;
    
    // Convert NED coordinates to grid key
    HashKey _neu_to_grid_key(const Vector3f &ned) const;
    uint16_t _grid_key_to_bucket(const HashKey &key) const;
    
    // Insert a point obstacle
    bool _insert_point_obstacle(const Location &center, float radius,  uint32_t timestamp_ms, ObstacleType type);
    bool _insert_point_obstacle(const Vector3f &neu_pos, float radius,  uint32_t timestamp_ms, ObstacleType type);
    
    // Special cases to deal with large circles
    bool _insert_large_inclusion_circle(const Vector3f &center, float radius);
    bool _insert_large_exclusion_circle(const Vector3f &center, float radius);
    
    // Insert a line obstacle  
    bool _insert_line_obstacle(const Location &start, const Location &end, ObstacleType type);
    bool _insert_line_obstacle(const Vector3f &start_neu, const Vector3f &end_neu, ObstacleType type);

    // Distance calculations
    float _distance_to_point(const Vector3f &neu_pos, const Obstacle &obstacle) const;
    float _distance_to_line(const Vector3f &neu_pos, const Obstacle &obstacle) const;
    float _distance_to_circle(const Vector3f &neu_pos, const Obstacle &obstacle) const;
    
    // Calculate search radius in grid cells
    uint8_t _calculate_search_radius(float max_search_radius) const;

    //Location _origin;
    //bool _origin_set;
    
    BucketItem* _buckets[OA_SPATIAL_HASH_BUCKETS];
    BucketItem _pool[OA_SPATIAL_HASH_POOL_SIZE];
    uint16_t _pool_used;
    uint16_t _num_obstacles;

    // Pool management
    BucketItem* _alloc_bucket_item();
    void _free_all_bucket_items();
};