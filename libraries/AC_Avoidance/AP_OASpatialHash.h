#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>

class AP_OASpatialHash {
public:
    AP_OASpatialHash();
    ~AP_OASpatialHash();
    
    // Initialize spatial hash with search radius and grid size
    void init(float search_radius, uint16_t grid_size);
    
    // Clear all entries
    void clear();
    
    // Insert a location into the spatial hash
    bool insert(const Location &loc, char *label, uint32_t update_time_ms, float bearing, float distance);
    
    // Check for collision within radius
    bool check_collision(const Location &loc, float radius) const;

    // look for the closest obstacle and return the distance 
    float find_closest_obstacle_distance(const Location &loc, float max_search_radius) const;
    
    // Check if spatial hash is populated
    bool is_populated() const { return _num_items > 0; }
    
    // Get number of items in hash
    uint16_t get_num_items() const { return _num_items; }

    void debug_print_contents() const;
    uint16_t _count_items_in_bucket(uint16_t bucket) const;

private:
    static const uint16_t OA_SPATIAL_HASH_BUCKETS = 64;
    static const uint16_t OA_SPATIAL_HASH_GRID_SIZE = 16;
    static const uint16_t OA_SPATIAL_HASH_POOL_SIZE = 256;
    
    struct LocationItem {
        Location        location;
        uint32_t        update_time_ms;
        LocationItem    *next;
        char            *label;
        float           bearing;    // bearing from current location of the vehicle. probably for debug only
        float           distance;   // distance from current location of the vehicle. probably for debug only
    };
    
    LocationItem        *_buckets[OA_SPATIAL_HASH_BUCKETS];
    uint16_t            _num_items;
    float               _search_radius;
    float               _grid_cell_size;
    uint16_t            _grid_size;
    LocationItem        *_pool;
    
    // Hash function to convert location to bucket index
    uint16_t            _calc_hash_bucket(const Location &loc) const;
    
    // Convert location to grid coordinates
    void                _location_to_grid(const Location &loc, int16_t &grid_x, int16_t &grid_y) const;
    
    // Memory management
    LocationItem*       _alloc_item();
    void                _free_item(LocationItem *item);
};