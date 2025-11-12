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
        float margin;           // outside the radius (if applicable) or away from the line
        uint32_t timestamp_ms;
    };

    struct HashKey {
        int32_t x;
        int32_t y;
        bool operator==(const HashKey &other) const {
            return x == other.x && y == other.y;
        }
    };

    // Initialize with origin location
    // void set_origin(const Location &origin);
    
    // Clear all obstacles
    void clear();

    // Insert methods for different obstacle types
    bool insert_database_obstacle(const Vector3f &new_pos, float radius, float margin, uint32_t timestamp_ms);
    
    bool insert_fence_inclusion_polygon(const Vector3f &start_NEU_m, const Vector3f &end_NEU_m, float margin);
    bool insert_fence_exclusion_polygon(const Vector3f &start_NEU_m, const Vector3f &end_NEU_m, float margin);
    //bool insert_fence_inclusion_polygon(const Vector3f &start_NEU_m, const Vector3f &end_NEU_m, float margin)     { return _insert_line_obstacle(start_NEU_m, end_NEU_m, margin, FENCE_INCLUSION_POLYGON);};
    //bool insert_fence_exclusion_polygon(const Vector3f &start_NEU_m, const Vector3f &end_NEU_m, float margin)     { return _insert_line_obstacle(start_NEU_m, end_NEU_m, margin, FENCE_EXCLUSION_POLYGON);};
    // bool insert_fence_inclusion_circle(const Vector3f &center_NEU_m, float radius, float margin);
    // bool insert_fence_exclusion_circle(const Vector3f &center_NEU_m, float radius, float margin);
    
    bool insert_fence_inclusion_circle(const Vector3f &center_NEU_m, float radius);
    
    bool insert_fence_exclusion_circle(const Vector3f &center_NEU_m, float radius);
    
    // Find closest obstacle distance
    float find_closest_obstacle_distance(const Location &loc, float max_search_radius) const;

    // New ray tracing method for line segments
    float find_closest_obstacle_to_line(const Location& start, const Location& end, float margin) const;
    
    // Get number of items in spatial hash
    uint16_t get_num_items() const { return _num_obstacles; }

    // Recenter the grid as the vehicle moves in space
    void set_current_position(const Vector3f& current_position_neu);
    
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

    struct BucketItem {
        Obstacle obstacle;
        BucketItem *next;
    };

    // Convert Location to NEU coordinates relative to origin
    Vector3f _location_to_neu(const Location &loc) const;
    
    // Convert NED coordinates to grid key
    HashKey _neu_to_grid_key(const Vector3f &neu) const;
    uint16_t _grid_key_to_bucket(const HashKey &key) const;

    // deal with the grid moving as the vehicle moves
    void _recenter_grid(const Vector3f& current_position_neu);
    void _slide_grid(const Vector2f& new_center);
    void _update_obstacle_master_list();

    // manage adding new items
    BucketItem* _new_bucket_item(const Vector3f &neu_pos, float radius,  float margin, uint32_t timestamp_ms, ObstacleType type, uint16_t bucket);

    bool _insert_obstacle_reference(Obstacle& obstacle, const HashKey& cell_key);

    // Insert a point obstacle
    bool _insert_point_obstacle_loc(const Location &center, float radius,  float margin, uint32_t timestamp_ms, ObstacleType type);
    bool _insert_point_obstacle_neu(const Vector3f &neu_pos, float radius, float margin, uint32_t timestamp_ms,  ObstacleType type);
    bool _insert_point_obstacle_to_hash(const Vector3f &neu_pos, float radius, float margin, uint32_t timestamp_ms,  ObstacleType type);
    
    // Insert circles into all covered cells
    bool _insert_exclusion_circle_multicell(const Vector3f &center, float radius);
    bool _insert_inclusion_circle_multicell(const Vector3f &center, float radius);
    
    // Circle Helper methods
    bool _cell_overlaps_circle(const HashKey& cell_key, const Vector3f& center, float radius) const;
    bool _cell_inside_circle(const HashKey& cell_key, const Vector3f& center, float radius) const;


    // Insert a line obstacle  
    bool _insert_line_obstacle_loc(const Location &start, const Location &end, float margin, ObstacleType type);
    bool _insert_line_obstacle_neu(const Vector3f &start_neu, const Vector3f &end_neu, float margin, ObstacleType type);
    bool _insert_line_obstacle_to_hash(const Vector3f &start_neu, const Vector3f &end_neu, float margin, ObstacleType type);

    // Distance calculations
    float _distance_to_point(const Vector3f &neu_pos, const Obstacle &obstacle) const;
    float _distance_to_line(const Vector3f &neu_pos, const Obstacle &obstacle) const;
    float _distance_to_circle(const Vector3f &neu_pos, const Obstacle &obstacle) const;
    float _distance_to_circle_boundary(const Vector3f &neu_pos, const Obstacle &obstacle) const;

    // Calculate search radius in grid cells
    uint8_t _calculate_search_radius(float max_search_radius) const;

    // Helper for segment-to-segment distance calculation
    float _distance_between_segments(const Vector3f& seg1_start, const Vector3f& seg1_end, 
                                    const Vector3f& seg2_start, const Vector3f& seg2_end) const;

    /// Helper for line segment to circle distance calculation
    float _distance_between_line_and_circle(const Vector3f& line_start, const Vector3f& line_end, 
                                           const Vector3f& circle_center, float circle_radius) const;

    // Helper to allow reuse of obstacles in multiple cells
    Obstacle* _create_or_find_obstacle(const Vector3f& center, float radius, ObstacleType type, uint32_t timestamp_ms);

    BucketItem* _buckets[OA_SPATIAL_HASH_BUCKETS];
    BucketItem _pool[OA_SPATIAL_HASH_POOL_SIZE];
    uint16_t _pool_used;
    uint16_t _num_obstacles;

    // Pool management
    BucketItem* _alloc_bucket_item();
    void _free_all_bucket_items();

     // Grid recentering
    Vector2f _grid_center_neu;      // Current grid center in NEU coordinates
    bool _grid_center_valid;        // Whether grid center has been initialized
    uint32_t _last_recenter_time;   // Last time we recentered the grid
    
    // Sliding window management
    ObjectArray<Obstacle> _all_obstacles;  // Master list of all obstacles
};