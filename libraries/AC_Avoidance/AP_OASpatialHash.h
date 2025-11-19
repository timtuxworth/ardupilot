#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>

#define AP_OASPATIAL_EXCLUDE_LABEL (char *)"Exclude"
#define AP_OASPATIAL_INCLUDE_LABEL (char *)"Include"

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
        ObstacleType    type;
        char            *label;         // text label for the obstacle if availailable;
        Vector3f        pos;            // Position in meters from origin (NED)
        Vector3f        pos2;           // Second position for line segments (NED)
        float           radius;         // For circles and point obstacles
        float           margin;         // outside the radius (if applicable) or away from the line
        uint32_t        timestamp_ms;

         // For polygons
        const Vector2f* polygon_points_cm;      // Pointer to array of polygon vertices (in NE, 2D, cm)
        uint16_t        polygon_point_count;    // Number of vertices in the polygon
    };

    struct HashKey {
        int32_t         x;
        int32_t         y;
        bool            operator==(const HashKey &other) const {
            return x == other.x && y == other.y;
        }
    };

    // Initialize with origin location
    // void set_origin(const Location &origin);
    
    // Clear all obstacles
    void clear();

    // Insert methods for different obstacle types
    bool insert_database_obstacle(const Vector3f &vehicle_NEU_m, const Vector3f &obstacle_NEU_m, float radius, float margin, uint32_t timestamp_ms, char *label);
    
    bool insert_fence_inclusion_polygon(const Vector3f &vehicle_NEU_m, const Vector3f &start_NEU_m, const Vector3f &end_NEU_m, float margin);
    bool insert_fence_exclusion_polygon(const Vector3f &vehicle_NEU_m, const Vector3f &start_NEU_m, const Vector3f &end_NEU_m, float margin);
    //bool insert_fence_inclusion_polygon(const Vector3f &start_NEU_m, const Vector3f &end_NEU_m, float margin)     { return _insert_line_obstacle(start_NEU_m, end_NEU_m, margin, FENCE_INCLUSION_POLYGON);};
    //bool insert_fence_exclusion_polygon(const Vector3f &start_NEU_m, const Vector3f &end_NEU_m, float margin)     { return _insert_line_obstacle(start_NEU_m, end_NEU_m, margin, FENCE_EXCLUSION_POLYGON);};
    // bool insert_fence_inclusion_circle(const Vector3f &center_NEU_m, float radius, float margin);
    // bool insert_fence_exclusion_circle(const Vector3f &center_NEU_m, float radius, float margin);
    
    bool insert_fence_inclusion_circle(const Vector3f &center_NEU_m, float radius);
    
    bool insert_fence_exclusion_circle(const Vector3f &center_NEU_m, float radius);

    bool insert_fence_inclusion_polygon(const Vector3f &vehicle_NEU_m, const Vector2f *points, uint16_t point_count, float margin);

    bool insert_fence_exclusion_polygon(const Vector3f &vehicle_NEU_m, const Vector2f *points, uint16_t point_count, float margin);

    // Find closest obstacle distance
    //float find_closest_obstacle_to_loc(const Location &loc, float max_search_radius) const;

    // New ray tracing method for line segments
    float find_closest_obstacle_to_line(const Location& start, const Location& end, float margin) const;
    
    // Recenter the grid as the vehicle moves in space
    void set_current_position(const Vector3f& current_NEU_m);

    uint16_t get_pool_used() { return _pool_used; };
    
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
    Vector3f _grid_key_to_neu(const HashKey& key) const;
    uint16_t _grid_key_to_bucket(const HashKey &key) const;

    // deal with the grid moving as the vehicle moves
    //void _recenter_grid(const Vector3f& current_position_neu);
    //void _slide_grid(const Vector3f& new_center);
    //void _update_obstacle_master_list();

    // manage adding new items
    //BucketItem* _new_bucket_item(const Vector3f &neu_pos, float radius,  float margin, uint32_t timestamp_ms, ObstacleType type, uint16_t bucket);

    bool _insert_obstacle_reference(Obstacle& obstacle, const HashKey& cell_key);

    // Insert a point obstacle
    bool _insert_point_obstacle_loc(const Vector3f &vehicle_NEU_m, const Location &center, float radius,  float margin, uint32_t timestamp_ms, ObstacleType type, char *label);
    bool _insert_point_obstacle_neu(const Vector3f &vehicle_NEU_m, const Vector3f &neu_pos, float radius, float margin, uint32_t timestamp_ms,  ObstacleType type, char *label);
    bool _insert_point_obstacle_to_hash(const Vector3f &neu_pos, float radius, float margin, uint32_t timestamp_ms,  ObstacleType type, char *label);
    
    // Insert circles into all covered cells
    bool _insert_exclusion_circle_multicell(const Vector3f &center_NEU_m, float radius);
    bool _insert_inclusion_circle_multicell(const Vector3f &center_NEU_m, float radius);
    bool _insert_exclusion_point_multicell(const Vector3f &center_NEU_m, float radius, AP_OASpatialHash::ObstacleType type, char *label);
    bool _insert_exclusion_point_multicell_fastest(const Vector3f &center_NEU_m, float radius, ObstacleType type, char *label);

    // Circle Helper methods
    bool _cell_overlaps_circle(const HashKey& cell_key, const Vector3f& center, float radius) const;
    bool _cell_inside_circle(const HashKey& cell_key, const Vector3f& center, float radius) const;


    // Insert a line obstacle  
    bool _insert_line_obstacle_loc(const Vector3f &vehicle_NEU_m, const Location &start, const Location &end, float margin, ObstacleType type, char *label);
    bool _insert_line_obstacle_neu(const Vector3f &vehicle_NEU_m, const Vector3f &start_neu, const Vector3f &end_neu, float margin, ObstacleType type, char *label);
    bool _insert_line_obstacle_to_hash(const Vector3f &start_neu, const Vector3f &end_neu, float margin, ObstacleType type, char *label);

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

    float _point_to_line_segment_distance_sq(const Vector3f& start, const Vector3f& end, const Vector3f& point) const;
    bool _is_obstacle_near_line(const Vector3f& start, const Vector3f& end, const Obstacle& obstacle, float search_radius_sq) const;
    float _circle_to_line_segment_distance_quick(const Vector3f& start, const Vector3f& end, const Vector3f& center, float radius) const;
    bool _is_obstacle_near_line_fast(const Vector3f& start, const Vector3f& end, const Obstacle& obstacle, float search_radius_sq) const;


    bool _circle_overlaps_cell(const Vector3f& circle_center_NEU_m,
                                            float radius,
                                            const HashKey& cell_key) const;
    

                                            // Simplified polygon insertion
    bool _insert_polygon_obstacle(const Vector3f &vehicle_NEU_m, const Vector2f *points, 
                                 uint16_t point_count, float margin, ObstacleType type, 
                                 char *label, bool is_inclusion);
    
    // Helper methods
    bool _point_in_polygon(const Vector3f &point, const Vector2f *points, uint16_t point_count) const;
    bool _polygon_overlaps_cell(const HashKey& cell_key, const Vector2f *points, uint16_t point_count) const;

    bool _circle_cell_aabb_overlap(const Vector3f& center, float radius, const HashKey& cell_key) const;
    bool _circle_overlaps_cell_fast(const Vector3f& center, float radius_sq, const HashKey& cell_key) const;

    bool _polygon_bbox_overlaps_cell(const HashKey& cell_key, const Vector2f& poly_min_m, const Vector2f& poly_max_m) const;

    BucketItem* _buckets[OA_SPATIAL_HASH_BUCKETS];
    BucketItem _pool[OA_SPATIAL_HASH_POOL_SIZE];
    uint16_t _num_obstacles;

    // Pool management
    BucketItem* _alloc_bucket_item();
    void _free_all_bucket_items();    
    uint16_t _pool_used;


     // Grid recentering
    Vector3f _grid_center_neu;      // Current grid center in NEU coordinates
    bool _grid_center_valid;        // Whether grid center has been initialized
    //uint32_t _last_recenter_time;   // Last time we recentered the grid
    
    // Sliding window management
    //ObjectArray<Obstacle> _all_obstacles;  // Master list of all obstacles
};