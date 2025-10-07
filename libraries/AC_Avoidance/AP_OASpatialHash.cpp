#include "AP_OASpatialHash.h"
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <string.h>
#include <stdio.h>

AP_OASpatialHash::AP_OASpatialHash() :
    _num_items(0),
    _search_radius(0.0f),
    _grid_cell_size(0.0f),
    _grid_size(OA_SPATIAL_HASH_GRID_SIZE),
    _pool(nullptr)
{
    memset(_buckets, 0, sizeof(_buckets));
    _pool = (LocationItem*)calloc(OA_SPATIAL_HASH_POOL_SIZE, sizeof(LocationItem));
}

AP_OASpatialHash::~AP_OASpatialHash()
{
    clear();
    if (_pool != nullptr) {
        free(_pool);
        _pool = nullptr;
    }
}

void AP_OASpatialHash::init(float search_radius, uint16_t grid_size)
{
    _search_radius = search_radius;
    _grid_size = grid_size;
    _grid_cell_size = (2.0f * search_radius) / grid_size;
    clear();
}

void AP_OASpatialHash::clear()
{
    for (uint16_t i = 0; i < OA_SPATIAL_HASH_BUCKETS; i++) {
        LocationItem *item = _buckets[i];
        while (item != nullptr) {
            LocationItem *next = item->next;
            _free_item(item);
            item = next;
        }
        _buckets[i] = nullptr;
    }
    _num_items = 0;
}

bool AP_OASpatialHash::insert(const Location &loc, char *label, uint32_t update_time_ms, float bearing, float distance)
{
    if (loc.is_zero() || _pool == nullptr) {
        return false;
    }
    
    LocationItem *new_item = _alloc_item();
    if (new_item == nullptr) {
        return false;
    }
    
    new_item->location          = loc;
    new_item->label             = label;
    new_item->update_time_ms    = update_time_ms;
    new_item->bearing           = bearing;
    new_item->distance          = distance;
    
    uint16_t bucket             = _calc_hash_bucket(loc);
    new_item->next              = _buckets[bucket];
    _buckets[bucket]            = new_item;
    _num_items++;
    
    return true;
}

// check_collision() Key Algorithm Notes

// Spatial Partitioning: The spatial hash divides the world into a grid, so we only check obstacles in nearby cells instead of all obstacles
// 3x3 Grid Check: We check the center cell plus all 8 surrounding cells to handle edge cases
// Squared Distance: Using squared distance for comparison is much faster than using actual distance
// Early Exit: Returns immediately when any collision is found
// O(1) Lookup: In ideal cases, this is constant time vs O(n) for brute force
// This method is the core of the spatial hash performance improvement - it reduces collision checks from checking every obstacle to only checking obstacles in the 9 closest grid cells.

bool AP_OASpatialHash::check_collision(const Location &loc, float radius) const
{
    // Validate input location to ensure we're checking a valid position
    if (loc.is_zero()) {
        return false; // Invalid location, no collision possible
    }
    
    // Convert the search location to grid coordinates
    // This tells us which cell in our spatial grid contains the point we're checking    int16_t center_x, center_y;
    int16_t center_x, center_y;
    _location_to_grid(loc, center_x, center_y);
    
    // Pre-calculate squared radius for efficient distance comparison
    // Using squared distance avoids expensive square root operations
    const float radius_sq = radius * radius;

    // Check not only the center cell but also adjacent cells (3x3 grid area)
    // This handles cases where obstacles are near cell boundaries
    for (int16_t dx = -1; dx <= 1; dx++) {
        for (int16_t dy = -1; dy <= 1; dy++) {
            // Calculate coordinates of adjacent cell to check
            int16_t grid_x = center_x + dx;
            int16_t grid_y = center_y + dy;
            
            // Verify the adjacent cell coordinates are within our grid bounds
            if (grid_x >= 0 && grid_x < (int16_t)_grid_size && 
                grid_y >= 0 && grid_y < (int16_t)_grid_size) {
                
                // Convert 2D grid coordinates to 1D bucket index
                // This maps our grid position to the corresponding hash bucket
                uint16_t bucket = grid_y * _grid_size + grid_x;

                // Safety check: ensure bucket index is within allocated array bounds
                if (bucket >= OA_SPATIAL_HASH_BUCKETS) {
                    continue;
                }
                
                // Iterate through all obstacles stored in this grid cell's bucket
                // Each bucket contains a linked list of LocationItem objects
                LocationItem *item = _buckets[bucket];
                while (item != nullptr) {
                    // Calculate straight-line distance to this obstacle
                    float dist = item->location.get_distance(loc);
                    float dist_sq = dist * dist;
                    // Check if obstacle is within the specified collision radius
                    if (dist_sq <= radius_sq) {
                        return true;
                    }
                    item = item->next;
                }
            }
        }
    }
    
    return false;
}

float AP_OASpatialHash::find_closest_obstacle_distance(const Location &loc, float max_search_radius) const
{
    if (loc.is_zero()) {
        return FLT_MAX;
    }
    
    float closest_distance = FLT_MAX;
    
    // Convert search location to grid coordinates
    int16_t center_x, center_y;
    _location_to_grid(loc, center_x, center_y);
    
    // Search in 3x3 grid area around the location
    for (int16_t dx = -1; dx <= 1; dx++) {
        for (int16_t dy = -1; dy <= 1; dy++) {
            int16_t grid_x = center_x + dx;
            int16_t grid_y = center_y + dy;
            
            if (grid_x >= 0 && grid_x < (int16_t)_grid_size && 
                grid_y >= 0 && grid_y < (int16_t)_grid_size) {
                
                uint16_t bucket = grid_y * _grid_size + grid_x;
                if (bucket >= OA_SPATIAL_HASH_BUCKETS) {
                    continue;
                }
                
                // Check all obstacles in this bucket
                LocationItem *item = _buckets[bucket];
                while (item != nullptr) {
                    float dist = item->location.get_distance(loc);
                    if (dist < closest_distance && dist <= max_search_radius) {
                        closest_distance = dist;
                        // printf("found %s dist %.0f\n", item->label, closest_distance);
                    }
                    item = item->next;
                }
            }
        }
    }
    
    return closest_distance;
}

uint16_t AP_OASpatialHash::_calc_hash_bucket(const Location &loc) const
{
    int16_t grid_x, grid_y;
    _location_to_grid(loc, grid_x, grid_y);
    return (grid_y * _grid_size + grid_x) % OA_SPATIAL_HASH_BUCKETS;
}

void AP_OASpatialHash::_location_to_grid(const Location &loc, int16_t &grid_x, int16_t &grid_y) const
{
   // Use the lower bits of lat/lng for grid distribution
    grid_x = (loc.lat & 0xFFFF) % _grid_size;
    grid_y = (loc.lng & 0xFFFF) % _grid_size;
}

AP_OASpatialHash::LocationItem* AP_OASpatialHash::_alloc_item()
{
    for (uint16_t i = 0; i < OA_SPATIAL_HASH_POOL_SIZE; i++) {
        if (_pool[i].update_time_ms == 0) {
            return &_pool[i];
        }
    }
    return nullptr;
}

void AP_OASpatialHash::_free_item(LocationItem *item)
{
    if (item != nullptr) {
        item->label          = (char *)"";
        item->update_time_ms = 0;
    }
}

void AP_OASpatialHash::debug_print_contents() const
{
    ::printf("OA Spatial Hash: %u items in %u buckets\n", _num_items, OA_SPATIAL_HASH_BUCKETS);
    
    for (uint16_t bucket = 0; bucket < OA_SPATIAL_HASH_BUCKETS; bucket++) {
        LocationItem *item = _buckets[bucket];
        if (item != nullptr) {
            ::printf("Bucket[%3u]: ", bucket);
            uint16_t count = 0;
            while (item != nullptr && count < 5) { // Limit to 5 items per bucket
                ::printf("(%s %.4f,%.4f deg %.0f dst %.0f) ", item->label,
                        item->location.lat / 10000000.0f,
                        item->location.lng / 10000000.0f,
                        item->bearing, item->distance);
                item = item->next;
                count++;
            }
            if (item != nullptr) {
                ::printf("... (+%u more)", _count_items_in_bucket(bucket) - 5);
            }
            ::printf("\n");
        }
    }
}

// Helper method to count items in a bucket
uint16_t AP_OASpatialHash::_count_items_in_bucket(uint16_t bucket) const
{
    uint16_t count = 0;
    LocationItem *item = _buckets[bucket];
    while (item != nullptr) {
        count++;
        item = item->next;
    }
    return count;
}
