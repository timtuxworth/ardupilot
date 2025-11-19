#include "AP_OASpatialHash.h"
#include <AP_AHRS/AP_AHRS.h>

// Initialize grid center in constructor
AP_OASpatialHash::AP_OASpatialHash()
{
}

void AP_OASpatialHash::clear()
{
    for (uint16_t i = 0; i < OA_SPATIAL_HASH_BUCKETS; i++) {
        _buckets[i] = nullptr;
    }
    _pool_used = 0;
    _num_obstacles = 0;
    _grid_center_valid = false;
}

AP_OASpatialHash::BucketItem* AP_OASpatialHash::_alloc_bucket_item()
{
    if (_pool_used >= OA_SPATIAL_HASH_POOL_SIZE) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SH: Pool Exhausted");
        return nullptr;
    }
    BucketItem* item = &_pool[_pool_used++];
    item->next = nullptr;  // CRITICAL: Always initialize next pointer

    /*if(_pool_used % 50 == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SH: Pool %d", _pool_used);
    }*/

    return item;
}

AP_OASpatialHash::HashKey AP_OASpatialHash::_neu_to_grid_key(const Vector3f &neu) const
{
    HashKey key;
    
    if (!_grid_center_valid) {
        // Fallback if grid hasn't been positioned
        key.x = (int32_t)(neu.x / GRID_CELL_SIZE);
        key.y = (int32_t)(neu.y / GRID_CELL_SIZE);
    } else {
        // Convert absolute NEU to coordinates relative to grid center
        float grid_rel_x = neu.x - _grid_center_neu.x;
        float grid_rel_y = neu.y - _grid_center_neu.y;
        
        // Convert to grid indices
        key.x = (int32_t)((grid_rel_x + (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f)) / GRID_CELL_SIZE);
        key.y = (int32_t)((grid_rel_y + (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f)) / GRID_CELL_SIZE);
    }
    
    // Wrap to bucket range
    key.x = key.x % OA_SPATIAL_HASH_BUCKETS;
    key.y = key.y % OA_SPATIAL_HASH_BUCKETS;
    if (key.x < 0) key.x += OA_SPATIAL_HASH_BUCKETS;
    if (key.y < 0) key.y += OA_SPATIAL_HASH_BUCKETS;
    
    return key;
}

Vector3f AP_OASpatialHash::_grid_key_to_neu(const HashKey& key) const
{
    Vector3f neu;
    
    if (!_grid_center_valid) {
        // Fallback - treat key as absolute grid cells
        neu.x = key.x * GRID_CELL_SIZE;
        neu.y = key.y * GRID_CELL_SIZE;
        neu.z = 0;
        return neu;
    }
    
    // Convert grid key back to NEU relative to grid center
    float grid_rel_x = (key.x * GRID_CELL_SIZE) - (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f);
    float grid_rel_y = (key.y * GRID_CELL_SIZE) - (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f);
    
    // Convert to absolute NEU by adding grid center
    neu.x = _grid_center_neu.x + grid_rel_x;
    neu.y = _grid_center_neu.y + grid_rel_y;
    neu.z = 0;
    
    return neu;
}

// Set current position and recenter if needed
void AP_OASpatialHash::set_current_position(const Vector3f& current_NEU_m)
{
    _grid_center_neu = current_NEU_m;
    _grid_center_valid = true;
}

uint16_t AP_OASpatialHash::_grid_key_to_bucket(const HashKey &key) const
{
    // Simple hash function that distributes keys across buckets
    uint32_t hash = (uint32_t)(key.x) * 73856093U ^ (uint32_t)(key.y) * 19349663U;
    return hash % OA_SPATIAL_HASH_BUCKETS;
}

bool AP_OASpatialHash::_insert_point_obstacle_loc(const Vector3f &vehicle_NEU_m, const Location &center, float radius, float margin, uint32_t timestamp_ms, ObstacleType type, char *label)
{
    Vector3f center_neu_m;
    if (!center.get_vector_from_origin_NEU_m(center_neu_m)) {
        return false;
    }
    if(!_insert_point_obstacle_neu(vehicle_NEU_m, center_neu_m, radius, margin, timestamp_ms, type, label)) {
        return false;
    }
    return true;
}
bool AP_OASpatialHash::_insert_point_obstacle_neu(const Vector3f &vehicle_NEU_m, const Vector3f &obstacle_NEU_pos, float radius, float margin, uint32_t timestamp_ms, ObstacleType type, char *label)
{
    return _insert_point_obstacle_to_hash(obstacle_NEU_pos, radius, margin, timestamp_ms, type, label);
}

bool AP_OASpatialHash::_insert_point_obstacle_to_hash(const Vector3f &neu_pos, float radius, float margin, uint32_t timestamp_ms, ObstacleType type, char *label)
{
    return _insert_exclusion_point_multicell_fastest(neu_pos, radius, type, label);

    /*HashKey key = _neu_to_grid_key(neu_pos);
    uint16_t bucket_index = _grid_key_to_bucket(key);
    
    // Allocate new bucket item
    BucketItem* new_item = _alloc_bucket_item();
    if (new_item == nullptr) {
        return false; // Pool exhausted
    }
    
    // Initialize the obstacle data
    new_item->obstacle.type = type;
    new_item->obstacle.label = (label == nullptr) ? (char *)"" : label;
    new_item->obstacle.pos = neu_pos;
    new_item->obstacle.radius = radius;
    new_item->obstacle.margin = margin;
    new_item->obstacle.timestamp_ms = timestamp_ms;
    
    // Insert at the head of the bucket's linked list
    new_item->next = _buckets[bucket_index];
    _buckets[bucket_index] = new_item;
    _num_obstacles++;
    
    return true;
    */
}

bool AP_OASpatialHash::_insert_line_obstacle_loc(const Vector3f &vehicle_NEU_m, const Location &start, const Location &end, float margin, ObstacleType type, char *label)
{
    Vector3f start_neu_m, end_neu_m;
    if (!start.get_vector_from_origin_NEU_m(start_neu_m)) {
        return false;
    }
    if (!end.get_vector_from_origin_NEU_m(end_neu_m)) {
        return false;
    }
    if(!_insert_line_obstacle_neu(vehicle_NEU_m, start_neu_m, end_neu_m, margin, type, label)) {
        return false;
    }
    return true;
}

bool AP_OASpatialHash::_insert_line_obstacle_neu(const Vector3f &vehicle_NEU_m, const Vector3f &start_neu, const Vector3f &end_neu, float margin, ObstacleType type, char *label)
{
    //_recenter_grid(vehicle_NEU_m);

    return _insert_line_obstacle_to_hash(start_neu, end_neu, margin, type, label);
}

/*bool AP_OASpatialHash::_insert_line_obstacle_to_hash(const Vector3f &start_neu, const Vector3f &end_neu, float margin, ObstacleType type, char *label)
{
    // Insert both endpoints as point obstacles
    _insert_point_obstacle_to_hash(start_neu, 0.1f, margin, 0, type, label);
    _insert_point_obstacle_to_hash(end_neu, 0.1f, margin, 0, type, label);
    
    // Sample intermediate points for long lines
    float distance = (end_neu - start_neu).xy().length();
    if (distance > GRID_CELL_SIZE) {
        uint8_t samples = MIN(8, (uint8_t)(distance / GRID_CELL_SIZE));
        for (uint8_t i = 1; i < samples; i++) {
            float ratio = (float)i / samples;
            Vector3f intermediate = start_neu + (end_neu - start_neu) * ratio;
            _insert_point_obstacle_to_hash(intermediate, 0.1f, margin, 0, type, label);
        }
    }
    
    // Also store the line segment for accurate distance calculation
    HashKey key = _neu_to_grid_key((start_neu + end_neu) * 0.5f);
    uint16_t bucket_index = _grid_key_to_bucket(key);
    
    BucketItem* new_item = _alloc_bucket_item();
    if (new_item == nullptr) {
        return false;
    }
    
    // Initialize the line obstacle
    new_item->obstacle.type = type;
    new_item->obstacle.label = (label == nullptr) ? (char *)"" : label;
    new_item->obstacle.pos = start_neu;
    new_item->obstacle.pos2 = end_neu;
    new_item->obstacle.radius = 0.1f; // Small radius for line endpoints
    new_item->obstacle.margin = margin;
    new_item->obstacle.timestamp_ms = AP_HAL::millis();
    
    // Insert at the head of the bucket
    new_item->next = _buckets[bucket_index];
    _buckets[bucket_index] = new_item;
    _num_obstacles++;
    
    return true;
}*/

/* Optimizing
bool AP_OASpatialHash::_insert_line_obstacle_to_hash(const Vector3f &start_neu, const Vector3f &end_neu, float margin, ObstacleType type, char *label)
{
    // Insert both endpoints as point obstacles
    _insert_point_obstacle_to_hash(start_neu, 0.1f, margin, 0, type, label);
    _insert_point_obstacle_to_hash(end_neu, 0.1f, margin, 0, type, label);
    
    // Sample intermediate points for long lines
    float distance = (end_neu - start_neu).xy().length();
    if (distance > GRID_CELL_SIZE) {
        uint8_t samples = MIN(8, (uint8_t)(distance / GRID_CELL_SIZE));
        for (uint8_t i = 1; i < samples; i++) {
            float ratio = (float)i / samples;
            Vector3f intermediate = start_neu + (end_neu - start_neu) * ratio;
            _insert_point_obstacle_to_hash(intermediate, 0.1f, margin, 0, type, label);
        }
    }
    
    // Now, use DDA to insert the line segment obstacle in every grid cell the line passes through
    HashKey start_key = _neu_to_grid_key(start_neu);
    HashKey end_key = _neu_to_grid_key(end_neu);
    
    int x0 = start_key.x, y0 = start_key.y;
    int x1 = end_key.x, y1 = end_key.y;
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0;
    int y = y0;
    
    const int max_dda_steps = OA_SPATIAL_HASH_BUCKETS * 3;
    int dda_steps = 0;
    
    while (dda_steps < max_dda_steps) {
        HashKey current_key = {x, y};
        uint16_t bucket_index = _grid_key_to_bucket(current_key);
        
        // Insert the line segment obstacle in this cell
        BucketItem* new_item = _alloc_bucket_item();
        if (new_item != nullptr) {
            new_item->obstacle.type = type;
            new_item->obstacle.label = (label == nullptr) ? (char *)"" : label;
            new_item->obstacle.pos = start_neu;
            new_item->obstacle.pos2 = end_neu;
            new_item->obstacle.radius = 0.1f;
            new_item->obstacle.margin = margin;
            new_item->obstacle.timestamp_ms = AP_HAL::millis();
            
            new_item->next = _buckets[bucket_index];
            _buckets[bucket_index] = new_item;
            _num_obstacles++;
        }
        
        if (x == x1 && y == y1) {
            break;
        }
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
        
        dda_steps++;
    }
    
    return true;
}
*/

bool AP_OASpatialHash::_insert_line_obstacle_to_hash(const Vector3f &start_neu, const Vector3f &end_neu, float margin, ObstacleType type, char *label)
{
    // OPTIMIZATION: Only insert endpoints for short lines, use sampling for long lines
    float distance = (end_neu - start_neu).xy().length();
    
    if (distance <= GRID_CELL_SIZE * 2) {
        // Short line - just insert endpoints
        _insert_point_obstacle_to_hash(start_neu, 0.1f, margin, 0, type, label);
        _insert_point_obstacle_to_hash(end_neu, 0.1f, margin, 0, type, label);
        return true;
    }
    
    // Medium line - use endpoint sampling instead of DDA
    uint8_t samples = MIN(4, (uint8_t)(distance / (GRID_CELL_SIZE * 2)));
    for (uint8_t i = 0; i <= samples; i++) {
        float ratio = (float)i / samples;
        Vector3f point = start_neu + (end_neu - start_neu) * ratio;
        _insert_point_obstacle_to_hash(point, 0.1f, margin, 0, type, label);
    }
    
    return true;
}

// Public insert methods
bool AP_OASpatialHash::insert_database_obstacle(const Vector3f &vehicle_NEU_m, const Vector3f &obstacle_pos, float radius, float margin, uint32_t timestamp_ms, char *label)
{
    // Grid recenter happens in insert_point_obstacle_neu
    if (!_insert_point_obstacle_neu(vehicle_NEU_m, obstacle_pos, radius, margin, timestamp_ms, OBSTACLE_DATABASE, label)) {
        return false;
    }
    return true;
}

bool AP_OASpatialHash::insert_fence_inclusion_polygon(const Vector3f &vehicle_NEU_m, const Vector3f &start_NEU_m, const Vector3f &end_NEU_m, float margin)     
{ 
    // Grid recenter happens in insert_line_obstacle_neu
    if(!_insert_line_obstacle_neu(vehicle_NEU_m, start_NEU_m, end_NEU_m, margin, FENCE_INCLUSION_POLYGON, AP_OASPATIAL_INCLUDE_LABEL)) {
        return false;
    }
    return true;
}

bool AP_OASpatialHash::insert_fence_exclusion_polygon(const Vector3f &vehicle_NEU_m, const Vector3f &start_NEU_m, const Vector3f &end_NEU_m, float margin)     
{ 
    if(!_insert_line_obstacle_neu(vehicle_NEU_m, start_NEU_m, end_NEU_m, margin, FENCE_EXCLUSION_POLYGON, AP_OASPATIAL_EXCLUDE_LABEL)) {
        return false;
    }
    return true;
}


bool AP_OASpatialHash::insert_fence_exclusion_circle(const Vector3f &center_NEU_m, float radius)
{
    if(!_insert_exclusion_circle_multicell(center_NEU_m, radius)) {
        return false;
    }
    return true;
}

/* Optimizing bool AP_OASpatialHash::_insert_exclusion_point_multicell(const Vector3f &center_NEU_m, float radius, AP_OASpatialHash::ObstacleType type, char *label)
{
    HashKey center_key = _neu_to_grid_key(center_NEU_m);
    int cells_radius = (int)ceilf(radius / GRID_CELL_SIZE);
    
    bool success = true;
    
    // Create template obstacle
    Obstacle point_obstacle;
    point_obstacle.type = type;
    point_obstacle.label = label;
    point_obstacle.pos = center_NEU_m;
    point_obstacle.radius = radius;
    point_obstacle.margin = 0;
    point_obstacle.timestamp_ms = AP_HAL::millis();
    
    for (int dx = -cells_radius; dx <= cells_radius; dx++) {
        for (int dy = -cells_radius; dy <= cells_radius; dy++) {
            HashKey cell_key = {center_key.x + dx, center_key.y + dy};
            
            if (cell_key.x >= 0 && cell_key.x < OA_SPATIAL_HASH_BUCKETS &&
                cell_key.y >= 0 && cell_key.y < OA_SPATIAL_HASH_BUCKETS) {
                
                if (_circle_overlaps_cell(center_NEU_m, radius, cell_key)) {
                    uint16_t bucket_index = _grid_key_to_bucket(cell_key);
                    
                    BucketItem* new_item = _alloc_bucket_item();
                    if (new_item == nullptr) {
                        success = false;
                        continue;
                    }
                    
                    // Copy obstacle data only
                    new_item->obstacle = point_obstacle;
                    
                    // Proper linking
                    new_item->next = _buckets[bucket_index];
                    _buckets[bucket_index] = new_item;
                    _num_obstacles++;
                }
            }
        }
    }
    
    return success;
}*/

bool AP_OASpatialHash::_insert_exclusion_point_multicell_fastest(const Vector3f &center_NEU_m, float radius, ObstacleType type, char *label)
{
    HashKey center_key = _neu_to_grid_key(center_NEU_m);
    int cells_radius = (int)ceilf(radius / GRID_CELL_SIZE);
    
    // Create template obstacle ONCE
    Obstacle point_obstacle;
    point_obstacle.type = type;
    point_obstacle.label = label;
    point_obstacle.pos = center_NEU_m;
    point_obstacle.radius = radius;
    point_obstacle.margin = 0;
    point_obstacle.timestamp_ms = AP_HAL::millis();
    
    bool success = true;
    
    int min_x = MAX(0, center_key.x - cells_radius);
    int max_x = MIN(OA_SPATIAL_HASH_BUCKETS - 1, center_key.x + cells_radius);
    int min_y = MAX(0, center_key.y - cells_radius);
    int max_y = MIN(OA_SPATIAL_HASH_BUCKETS - 1, center_key.y + cells_radius);
    
    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {
            HashKey cell_key = {x, y};
            
            // JUST USE AABB TEST - good enough for obstacle avoidance
            if (_circle_cell_aabb_overlap(center_NEU_m, radius, cell_key)) {
                uint16_t bucket_index = _grid_key_to_bucket(cell_key);
                
                BucketItem* new_item = _alloc_bucket_item();
                if (new_item == nullptr) {
                    success = false;
                    continue;
                }
                
                new_item->obstacle = point_obstacle;
                new_item->next = _buckets[bucket_index];
                _buckets[bucket_index] = new_item;
                _num_obstacles++;
            }
        }
    }
    
    return success;
}

bool AP_OASpatialHash::_insert_exclusion_point_multicell(const Vector3f &center_NEU_m, float radius, ObstacleType type, char *label)
{
    HashKey center_key = _neu_to_grid_key(center_NEU_m);
    int cells_radius = (int)ceilf(radius / GRID_CELL_SIZE);
    
    // OPTIMIZATION: Pre-calculate squared radius to avoid sqrt in overlap check
    float radius_sq = radius * radius;
    
    // Create template obstacle ONCE (moved outside loops)
    Obstacle point_obstacle;
    point_obstacle.type = type;
    point_obstacle.label = label;
    point_obstacle.pos = center_NEU_m;
    point_obstacle.radius = radius;
    point_obstacle.margin = 0;
    point_obstacle.timestamp_ms = AP_HAL::millis();
    
    bool success = true;
    
    // OPTIMIZATION: Use integer math for bounds checking
    int min_x = MAX(0, center_key.x - cells_radius);
    int max_x = MIN(OA_SPATIAL_HASH_BUCKETS - 1, center_key.x + cells_radius);
    int min_y = MAX(0, center_key.y - cells_radius);
    int max_y = MIN(OA_SPATIAL_HASH_BUCKETS - 1, center_key.y + cells_radius);
    
    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {
            HashKey cell_key = {x, y};
            
            // FAST OVERLAP CHECK: Use quick AABB test first
            if (!_circle_cell_aabb_overlap(center_NEU_m, radius, cell_key)) {
                continue;
            }
            
            // Only do precise check if AABB passes
            if (_circle_overlaps_cell(center_NEU_m, radius_sq, cell_key)) {
                uint16_t bucket_index = _grid_key_to_bucket(cell_key);
                
                BucketItem* new_item = _alloc_bucket_item();
                if (new_item == nullptr) {
                    success = false;
                    continue;
                }
                
                new_item->obstacle = point_obstacle; // Copy pre-built obstacle
                new_item->next = _buckets[bucket_index];
                _buckets[bucket_index] = new_item;
                _num_obstacles++;
            }
        }
    }
    
    return success;
}

// Add these fast helper methods:
bool AP_OASpatialHash::_circle_cell_aabb_overlap(const Vector3f& center, float radius, const HashKey& cell_key) const
{
    Vector3f cell_min = _grid_key_to_neu(cell_key);
    Vector3f cell_max = cell_min;
    cell_max.x += GRID_CELL_SIZE;
    cell_max.y += GRID_CELL_SIZE;
    
    // Simple AABB test - much faster than precise circle check
    float closest_x = fmaxf(cell_min.x, fminf(center.x, cell_max.x));
    float closest_y = fmaxf(cell_min.y, fminf(center.y, cell_max.y));
    
    float dx = center.x - closest_x;
    float dy = center.y - closest_y;
    
    return (dx * dx + dy * dy) <= (radius * radius);
}

/*bool AP_OASpatialHash::_circle_overlaps_cell_fast(const Vector3f& center, float radius_sq, const HashKey& cell_key) const
{
    Vector3f cell_center = _grid_key_to_neu(cell_key);
    cell_center.x += GRID_CELL_SIZE * 0.5f;
    cell_center.y += GRID_CELL_SIZE * 0.5f;
    
    float dx = center.x - cell_center.x;
    float dy = center.y - cell_center.y;
    float distance_sq = dx * dx + dy * dy;
    
    // Conservative test: cell radius is half the diagonal
    float cell_radius_sq = (GRID_CELL_SIZE * 0.707f) * (GRID_CELL_SIZE * 0.707f);
    
    return distance_sq <= (radius_sq + cell_radius_sq + 2 * radius * (GRID_CELL_SIZE * 0.707f));
}*/

bool AP_OASpatialHash::_insert_exclusion_circle_multicell(const Vector3f &center_NEU_m, float radius)
{
    return _insert_exclusion_point_multicell(center_NEU_m, radius, FENCE_EXCLUSION_CIRCLE, AP_OASPATIAL_EXCLUDE_LABEL);

    /*
    HashKey center_key = _neu_to_grid_key(center_NEU_m);
    int cells_radius = (int)ceilf(radius / GRID_CELL_SIZE);
    
    bool success = true;
    
    // Create template obstacle
    Obstacle circle_obstacle;
    circle_obstacle.type = FENCE_EXCLUSION_CIRCLE;
    circle_obstacle.label = AP_OASPATIAL_EXCLUDE_LABEL;
    circle_obstacle.pos = center_NEU_m;
    circle_obstacle.radius = radius;
    circle_obstacle.timestamp_ms = AP_HAL::millis();
    
    for (int dx = -cells_radius; dx <= cells_radius; dx++) {
        for (int dy = -cells_radius; dy <= cells_radius; dy++) {
            HashKey cell_key = {center_key.x + dx, center_key.y + dy};
            
            if (cell_key.x >= 0 && cell_key.x < OA_SPATIAL_HASH_BUCKETS &&
                cell_key.y >= 0 && cell_key.y < OA_SPATIAL_HASH_BUCKETS) {
                
                if (_circle_overlaps_cell(center_NEU_m, radius, cell_key)) {
                    uint16_t bucket_index = _grid_key_to_bucket(cell_key);
                    
                    BucketItem* new_item = _alloc_bucket_item();
                    if (new_item == nullptr) {
                        success = false;
                        continue;
                    }
                    
                    // Copy obstacle data only
                    new_item->obstacle = circle_obstacle;
                    
                    // Proper linking
                    new_item->next = _buckets[bucket_index];
                    _buckets[bucket_index] = new_item;
                    _num_obstacles++;
                }
            }
        }
    }
    
    return success;
    */

}

bool AP_OASpatialHash::insert_fence_inclusion_circle(const Vector3f &center_NEU_m, float radius)
{
    // Grid recenter happens in insert_inclusion_circle_multicell
    if(!_insert_inclusion_circle_multicell(center_NEU_m, radius)) {
        return false;
    }
    return true;
}

bool AP_OASpatialHash::_insert_inclusion_circle_multicell(const Vector3f &center_NEU_m, float radius)
{
    // For inclusion circles, insert into cells OUTSIDE the circle
    AP_OASpatialHash::HashKey center_key = _neu_to_grid_key(center_NEU_m);
    
    // Calculate the area we need to cover (cells outside the circle)
    int coverage_radius = fmin(OA_SPATIAL_HASH_BUCKETS / 2, (int)ceilf(radius * 3 / GRID_CELL_SIZE));
    
    bool success = true;
    
    // Create a template obstacle (not inserted into any bucket)
    Obstacle circle_obstacle;
    circle_obstacle.type = FENCE_INCLUSION_CIRCLE;
    circle_obstacle.label = AP_OASPATIAL_INCLUDE_LABEL;
    circle_obstacle.pos = center_NEU_m;
    circle_obstacle.radius = radius;
    circle_obstacle.margin = 0;
    circle_obstacle.timestamp_ms = AP_HAL::millis();
    
    // Insert obstacle into cells outside the inclusion circle
    for (int dx = -coverage_radius; dx <= coverage_radius; dx++) {
        for (int dy = -coverage_radius; dy <= coverage_radius; dy++) {
            HashKey cell_key = {center_key.x + dx, center_key.y + dy};
            
            // Check if cell is within grid bounds
            if (cell_key.x >= 0 && cell_key.x < OA_SPATIAL_HASH_BUCKETS &&
                cell_key.y >= 0 && cell_key.y < OA_SPATIAL_HASH_BUCKETS) {
                
                // For inclusion circles, insert if cell is OUTSIDE the circle
                if (!_circle_overlaps_cell(center_NEU_m, radius, cell_key)) {
                    uint16_t bucket_index = _grid_key_to_bucket(cell_key);
                    
                    // Create new bucket item
                    BucketItem* new_item = _alloc_bucket_item();
                    if (new_item == nullptr) {
                        success = false;
                        continue;
                    }
                    
                    // Copy obstacle data (NOT the entire BucketItem!)
                    new_item->obstacle = circle_obstacle;
                    
                    // Properly link into the bucket (insert at head)
                    new_item->next = _buckets[bucket_index];
                    _buckets[bucket_index] = new_item;
                    _num_obstacles++;
                }
            }
        }
    }
    
    return success;
}

bool AP_OASpatialHash::insert_fence_inclusion_polygon(const Vector3f &vehicle_NEU_m, const Vector2f *points, uint16_t point_count, float margin)
{
    return _insert_polygon_obstacle(vehicle_NEU_m, points, point_count, margin, 
                                   FENCE_INCLUSION_POLYGON, AP_OASPATIAL_INCLUDE_LABEL, true);
}

bool AP_OASpatialHash::insert_fence_exclusion_polygon(const Vector3f &vehicle_NEU_m, const Vector2f *points, uint16_t point_count, float margin)
{
    return _insert_polygon_obstacle(vehicle_NEU_m, points, point_count, margin, 
                                   FENCE_EXCLUSION_POLYGON, AP_OASPATIAL_EXCLUDE_LABEL, false);
}

// circle helper methods
// Check if a cell overlaps with a circle (for exclusion circles)
bool AP_OASpatialHash::_cell_overlaps_circle(const HashKey& cell_key, const Vector3f& center, float radius) const
{
    // Calculate cell center in NEU coordinates
    Vector3f cell_center_neu;
    cell_center_neu.x = (cell_key.x + 0.5f) * GRID_CELL_SIZE - (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f);
    cell_center_neu.y = (cell_key.y + 0.5f) * GRID_CELL_SIZE - (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f);
    cell_center_neu.z = 0;
    
    float distance = Vector2f(center.x - cell_center_neu.x, center.y - cell_center_neu.y).length();
    
    // Cell overlaps if distance is less than (radius + half cell diagonal)
    return distance <= radius + (GRID_CELL_SIZE * 0.707f);
}

// Check if a cell is inside a circle (for inclusion circles)
bool AP_OASpatialHash::_cell_inside_circle(const HashKey& cell_key, const Vector3f& center, float radius) const
{
    // Calculate cell center in NEU coordinates
    Vector3f cell_center_neu;
    cell_center_neu.x = (cell_key.x + 0.5f) * GRID_CELL_SIZE - (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f);
    cell_center_neu.y = (cell_key.y + 0.5f) * GRID_CELL_SIZE - (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f);
    cell_center_neu.z = 0;
    
    float distance = Vector2f(center.x - cell_center_neu.x, center.y - cell_center_neu.y).length();
    
    // Cell is inside if all four corners are within the circle
    // Use a conservative estimate: cell center must be within (radius - half cell diagonal)
    return distance <= radius - (GRID_CELL_SIZE * 0.707f);
}

// Distance calculation methods
float AP_OASpatialHash::_distance_to_point(const Vector3f &neu_pos, const Obstacle &obstacle) const
{
    // Use 2D distance for horizontal avoidance (ignore altitude)
    return Vector2f(neu_pos.x - obstacle.pos.x, neu_pos.y - obstacle.pos.y).length() - obstacle.radius;
}

float AP_OASpatialHash::_distance_to_line(const Vector3f &neu_pos, const Obstacle &obstacle) const
{
    // Convert to 2D for line distance (ignore altitude for fence lines)
    Vector2f p(neu_pos.x, neu_pos.y);
    Vector2f a(obstacle.pos.x, obstacle.pos.y);
    Vector2f b(obstacle.pos2.x, obstacle.pos2.y);
    
    Vector2f ab = b - a;
    Vector2f ap = p - a;
    
    float ab_squared = ab.length_squared();
    if (ab_squared < 1.0f) {
        return ap.length(); // Very short line, treat as point
    }
    
    // Project point onto line segment
    float t = (ap * ab) / ab_squared;
    t = constrain_float(t, 0.0f, 1.0f);
    
    // Find closest point on segment
    Vector2f closest_point = a + ab * t;
    
    return (p - closest_point).length() - obstacle.radius;
}

uint8_t AP_OASpatialHash::_calculate_search_radius(float max_search_radius) const
{
    // Calculate desired radius with safety margin
    // Example: For 250m max_search_radius with 40% safety margin:
    // 250m × 140% = 350m
    float desired_radius_m = (max_search_radius * SEARCH_SAFETY_MARGIN_PERCENT) / PERCENT_DIVISOR;
    
    // Convert to grid cells (floating point division)
    // Example: 350m / 50m per cell = 7.0 cells
    float desired_cells_float = desired_radius_m / GRID_CELL_SIZE;
    
    // Convert to integer with ceiling operation
    // Example: ceil(7.0) = 7 cells, ceil(7.1) = 8 cells
    uint8_t desired_cells = (uint8_t)desired_cells_float;
    if (desired_cells_float > desired_cells) {
        desired_cells++;
    }
    
    // Apply minimum bound for basic obstacle awareness
    // Even with small max_search_radius, we want at least 150m coverage (3 cells)
    if (desired_cells < MIN_SEARCH_RADIUS_CELLS) {
        return MIN_SEARCH_RADIUS_CELLS;
    }
    
    // Apply maximum bound for computational performance
    // Searching more than 500m (10 cells) would impact real-time performance
    if (desired_cells > MAX_SEARCH_RADIUS_CELLS) {
        return MAX_SEARCH_RADIUS_CELLS;
    }
    
    return desired_cells;
}

// 
// Find the distance of the closest obstacle to the line (start,end) looking a maximum of lookahead_m
//  also returns the name/label of the obstacle for display purposes
float AP_OASpatialHash::find_closest_obstacle_to_line(const Location& start, const Location& end, float lookahead_m) const
{
    // Convert start and end locations to NEU coordinates
    Vector3f start_NEU_m, end_NEU_m;
    start.get_vector_from_origin_NEU_m(start_NEU_m);
    end.get_vector_from_origin_NEU_m(end_NEU_m);
    Vector2f start_NE_m(start_NEU_m.x, start_NEU_m.y);
    Vector2f end_NE_m(end_NEU_m.x, end_NEU_m.y);
    Vector2f obstacle_NE_m;
    
    Vector2f start_NE_cm(start_NEU_m.x * 100.0, start_NEU_m.y * 100.0);
    Vector2f end_NE_cm(end_NEU_m.x * 100.0, end_NEU_m.y * 100.0);
    //Vector2f obstacle_NE_cm;

    // Get grid keys for start and end using current grid center
    HashKey start_key = _neu_to_grid_key(start_NEU_m);
    HashKey end_key = _neu_to_grid_key(end_NEU_m);
    
    float closest_distance = lookahead_m + 1;
    
    // Use integer DDA for grid traversal
    int x0 = start_key.x, y0 = start_key.y;
    int x1 = end_key.x, y1 = end_key.y;

    // Fudge to look 1 cell out because for some reason I'm not getting the hits
    x0 = MAX(0, x0);
    y0 = MAX(0, y0);
    x1 = MIN((int)OA_SPATIAL_HASH_BUCKETS, x1);
    y1 = MIN((int)OA_SPATIAL_HASH_BUCKETS, y1);

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0;
    int y = y0;
    
    // Safety counter for outer DDA loop
    const int max_dda_steps = OA_SPATIAL_HASH_BUCKETS * 3; // Reasonable upper bound
    int dda_steps = 0;
    
    while (dda_steps < max_dda_steps) {
        HashKey current_key = {x, y};
        uint16_t bucket_index = _grid_key_to_bucket(current_key);
        
        // Check all obstacles in current cell with safety counter
        BucketItem* item = _buckets[bucket_index];
        const uint16_t max_linked_list_items = 10; // Safety limit per bucket
        uint16_t linked_list_items = 0;
        
        while (item != nullptr && linked_list_items < max_linked_list_items) {
            Obstacle& obstacle = item->obstacle;
            // TODO - obvious optimization, flag whether we have visited this obstacle already and skip redoing the calculations
            float distance = FLT_MAX;
            
            switch (obstacle.type) {
                case OBSTACLE_DATABASE:
                    //distance = _point_to_line_segment_distance_sq(start_NEU_m, end_NEU_m, obstacle.pos);
                    //distance = safe_sqrt(distance) - obstacle.radius;
                    //distance = _circle_to_line_segment_distance_quick(start_NEU_m, end_NEU_m, obstacle.pos, obstacle.radius);
                    //distance -= obstacle.margin;



                    // OLD CALCULATION
                    //obstacle_NE_cm.x = obstacle.pos.x * 100.0;
                    //obstacle_NE_cm.y = obstacle.pos.y * 100.0;
                    //distance = Vector2f::closest_distance_between_line_and_point(start_NE_cm, end_NE_cm, obstacle_NE_cm) * 0.01f;
                    obstacle_NE_m.x = obstacle.pos.x;
                    obstacle_NE_m.y = obstacle.pos.y;
                    distance = Vector2f::closest_distance_between_line_and_point(start_NE_m, end_NE_m, obstacle_NE_m);
                    distance = distance - obstacle.radius;

                    // New Calculation doesn't work because the altitude from the obstacle is wierd. There's some kind of Up/Down thing going on so ignore it for now.
                    //distance = Vector3f::closest_distance_between_line_and_point(start_NEU_m, end_NEU_m, obstacle.pos);
                    //distance = distance - obstacle.radius;

                    break;
                    
                case FENCE_INCLUSION_POLYGON:
                case FENCE_EXCLUSION_POLYGON:
                    distance = FLT_MAX;
                    if (item->obstacle.polygon_points_cm != nullptr) {
                        // The units need to match. Because we are pointing at the polygon in the fence library, we need to use cm, then convert the result back to meters
                        distance = 0.01f * Polygon_closest_distance_line(item->obstacle.polygon_points_cm, item->obstacle.polygon_point_count, start_NE_m * 100.0f, end_NE_m * 100.0f);
                        distance -= item->obstacle.margin;
                    }
                    break;
                    
                case FENCE_INCLUSION_CIRCLE:
                    distance = _circle_to_line_segment_distance_quick(start_NEU_m, end_NEU_m, obstacle.pos, obstacle.radius);
                    distance -= obstacle.margin;
                    break;
                case FENCE_EXCLUSION_CIRCLE:
                    distance = _circle_to_line_segment_distance_quick(start_NEU_m, end_NEU_m, obstacle.pos, obstacle.radius);
                    distance += obstacle.margin;
                    break;
                    
                default:
                    distance = FLT_MAX;
                    break;
            }
            
            if (distance < closest_distance) {
                closest_distance = distance;
                
                // Early exit on collision
                if (distance <= 0.0f) {
                    return closest_distance;
                }
            }
            
            item = item->next;
            linked_list_items++;
        }
        
        // Log warning if we hit the linked list safety limit (optional)
        if (linked_list_items >= max_linked_list_items) {
            // Consider adding debug output here
        }
        
        // Break if we've reached the end cell
        if (x == x1 && y == y1) {
            break;
        }
        
        // Move to next cell using DDA
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
        
        dda_steps++;
    }
    
    // Log warning if we hit DDA safety limit (optional)
    if (dda_steps >= max_dda_steps) {
        // Consider adding debug output here
    }
    
    return closest_distance;
}

// Helper method to calculate distance between two line segments
float AP_OASpatialHash::_distance_between_segments(const Vector3f& seg1_start, const Vector3f& seg1_end, 
                                                  const Vector3f& seg2_start, const Vector3f& seg2_end) const
{
    // This calculates the minimum distance between two line segments in 2D
    // Convert to 2D vectors (ignore Z for now)
    Vector2f p1(seg1_start.x, seg1_start.y);
    Vector2f p2(seg1_end.x, seg1_end.y);
    Vector2f p3(seg2_start.x, seg2_start.y);
    Vector2f p4(seg2_end.x, seg2_end.y);
    
    Vector2f u = p2 - p1;
    Vector2f v = p4 - p3;
    Vector2f w = p1 - p3;
    
    float a = u * u; // dot product
    float b = u * v;
    float c = v * v;
    float d = u * w;
    float e = v * w;
    
    float D = a * c - b * b;
    float sc, sN, sD = D;
    float tc, tN, tD = D;
    
    // Compute the line parameters of the two closest points
    if (D < 1e-6f) { // lines are parallel
        sN = 0.0f;
        sD = 1.0f;
        tN = e;
        tD = c;
    } else {
        sN = (b * e - c * d);
        tN = (a * e - b * d);
        if (sN < 0.0f) {
            sN = 0.0f;
            tN = e;
            tD = c;
        } else if (sN > sD) {
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }
    
    if (tN < 0.0f) {
        tN = 0.0f;
        if (-d < 0.0f) {
            sN = 0.0f;
        } else if (-d > a) {
            sN = sD;
        } else {
            sN = -d;
            sD = a;
        }
    } else if (tN > tD) {
        tN = tD;
        if ((-d + b) < 0.0f) {
            sN = 0.0f;
        } else if ((-d + b) > a) {
            sN = sD;
        } else {
            sN = (-d + b);
            sD = a;
        }
    }
    
    // Finally do the division to get sc and tc
    sc = (fabsf(sN) < 1e-6f ? 0.0f : sN / sD);
    tc = (fabsf(tN) < 1e-6f ? 0.0f : tN / tD);
    
    // Get the difference of the two closest points - fix the vector operations
    Vector2f sc_u = u * sc;
    Vector2f tc_v = v * tc;
    Vector2f dP = w + sc_u - tc_v;
    
    return dP.length(); // return the distance
}

// Helper method to calculate distance between a line segment and a circle
float AP_OASpatialHash::_distance_between_line_and_circle(const Vector3f& line_start, const Vector3f& line_end, 
                                                         const Vector3f& circle_center, float circle_radius) const
{
    // Convert to 2D (ignore Z)
    Vector2f A(line_start.x, line_start.y);
    Vector2f B(line_end.x, line_end.y);
    Vector2f C(circle_center.x, circle_center.y);
    
    // Find the closest point on the line segment to the circle center
    Vector2f AB = B - A;
    Vector2f AC = C - A;
    
    float ab_sq = AB * AB; // dot product
    if (ab_sq < 1e-6f) {
        // Line segment is a point
        return (A - C).length() - circle_radius;
    }
    
    // Project C onto AB, but constrain to the line segment
    float t = (AC * AB) / ab_sq;
    t = constrain_float(t, 0.0f, 1.0f);
    
    Vector2f closest_point = A + AB * t;
    float distance_to_center = (closest_point - C).length();
    
    return distance_to_center - circle_radius;
}

// Quick rejection test for obstacles
bool AP_OASpatialHash::_is_obstacle_near_line(const Vector3f& start, const Vector3f& end, const Obstacle& obstacle, float search_radius_sq) const
{
    // For line obstacles, check distance to either endpoint
    float dist1_sq = (obstacle.pos - start).length_squared();
    float dist2_sq = (obstacle.pos - end).length_squared();
    float dist3_sq = (obstacle.pos2 - start).length_squared();
    float dist4_sq = (obstacle.pos2 - end).length_squared();
    
    return (dist1_sq < search_radius_sq || dist2_sq < search_radius_sq || 
            dist3_sq < search_radius_sq || dist4_sq < search_radius_sq);
}

// Fast squared distance from point to line segment
float AP_OASpatialHash::_point_to_line_segment_distance_sq(const Vector3f& start, const Vector3f& end, const Vector3f& point) const
{
    Vector3f segment = end - start;
    Vector3f to_point = point - start;
    
    // Project point onto segment
    float t = to_point.dot(segment) / segment.length_squared();
    t = constrain_float(t, 0.0f, 1.0f);
    
    // Find closest point on segment
    Vector3f projection = start + segment * t;
    
    // Return squared distance
    return (point - projection).length_squared();
}

// Quick rejection test for obstacles near line
bool AP_OASpatialHash::_is_obstacle_near_line_fast(const Vector3f& start, const Vector3f& end, const Obstacle& obstacle, float search_radius_sq) const
{
    // For line obstacles, check if either endpoint is near our line segment
    float dist1_sq = _point_to_line_segment_distance_sq(start, end, obstacle.pos);
    if (dist1_sq <= search_radius_sq) return true;
    
    if (obstacle.type == FENCE_INCLUSION_POLYGON || obstacle.type == FENCE_EXCLUSION_POLYGON) {
        float dist2_sq = _point_to_line_segment_distance_sq(start, end, obstacle.pos2);
        if (dist2_sq <= search_radius_sq) return true;
    }
    
    return false;
}

// Quick circle-line distance approximation
float AP_OASpatialHash::_circle_to_line_segment_distance_quick(const Vector3f& start, const Vector3f& end, const Vector3f& center, float radius) const
{
    float dist_sq = _point_to_line_segment_distance_sq(start, end, center);
    return safe_sqrt(dist_sq) - radius;
}

bool AP_OASpatialHash::_circle_overlaps_cell(const Vector3f& circle_center_NEU_m,
                                            float radius,
                                            const HashKey& cell_key) const
{
    // Get the NEU coordinates of the cell's minimum corner
    Vector3f cell_min = _grid_key_to_neu(cell_key);
    Vector3f cell_max = cell_min;
    cell_max.x += GRID_CELL_SIZE;
    cell_max.y += GRID_CELL_SIZE;
    
    // Find the closest point on the cell to the circle center
    float closest_x = constrain_float(circle_center_NEU_m.x, cell_min.x, cell_max.x);
    float closest_y = constrain_float(circle_center_NEU_m.y, cell_min.y, cell_max.y);
    
    // Calculate squared distance from circle center to closest point on cell
    float dx = circle_center_NEU_m.x - closest_x;
    float dy = circle_center_NEU_m.y - closest_y;
    float distance_sq = dx * dx + dy * dy;
    
    // Circle overlaps cell if distance to closest point is <= radius
    return distance_sq <= (radius * radius);
}

// Check if a point is inside a polygon
bool AP_OASpatialHash::_point_in_polygon(const Vector3f &point_cm, const Vector2f *points_cm, uint16_t point_count) const
{
    if (point_count < 3) return false;
    
    bool inside = false;
    uint16_t j = point_count - 1;
    
    for (uint16_t i = 0; i < point_count; i++) {
        if (((points_cm[i].y > point_cm.y) != (points_cm[j].y > point_cm.y)) &&
            (point_cm.x < (points_cm[j].x - points_cm[i].x) * (point_cm.y - points_cm[i].y) / 
                      (points_cm[j].y - points_cm[i].y) + points_cm[i].x)) {
            inside = !inside;
        }
        j = i;
    }
    
    return inside;
}

// Check if polygon overlaps with grid cell - because the polygon is in PolyLoader it's in cm
bool AP_OASpatialHash::_polygon_overlaps_cell(const HashKey& cell_key, const Vector2f *points_cm, uint16_t point_count) const
{
    // Convert cell to NEU coordinates
    Vector3f cell_min_m = _grid_key_to_neu(cell_key);
    Vector3f cell_max_m = cell_min_m;
    cell_max_m.x += GRID_CELL_SIZE;
    cell_max_m.y += GRID_CELL_SIZE;
    
    // Quick check: if any polygon vertex is inside the cell
    for (uint16_t i = 0; i < point_count; i++) {
        Vector2f point_m = points_cm[i] * 0.01f;
        if (point_m.x * 0.01f >= cell_min_m.x && 
                point_m.x * 0.01f <= cell_max_m.x &&
                point_m.y * 0.01f >= cell_min_m.y && 
                point_m.y * 0.01f <= cell_max_m.y) {
            return true;
        }
    }
    
    // Check if any cell corner is inside the polygon
    Vector3f corners_m[4] = {
        Vector3f(cell_min_m.x, cell_min_m.y, 0),
        Vector3f(cell_max_m.x, cell_min_m.y, 0),
        Vector3f(cell_max_m.x, cell_max_m.y, 0),
        Vector3f(cell_min_m.x, cell_max_m.y, 0)
    };
    
    for (uint8_t i = 0; i < 4; i++) {
        if (_point_in_polygon(corners_m[i] * 100.0f, points_cm, point_count)) {
            return true;
        }
    }
    
    return false; // Conservative approach
}

bool AP_OASpatialHash::_insert_polygon_obstacle(const Vector3f &vehicle_NEU_m, const Vector2f *points_cm, 
                                               uint16_t point_count, float margin, ObstacleType type, 
                                               char *label, bool is_inclusion)
{
    if (point_count < 3) return false;
    
    // OPTIMIZATION: Use much coarser grid for polygon insertion
    const uint8_t GRID_SKIP = 3; // Check every 3rd cell
    
    // Calculate bounding box (convert to meters)
    Vector2f min_point_m = points_cm[0] * 0.01f;
    Vector2f max_point_m = points_cm[0] * 0.01f;
    for (uint16_t i = 1; i < point_count; i++) {
        Vector2f point_m = points_cm[i] * 0.01f;
        min_point_m.x = fminf(min_point_m.x, point_m.x);
        min_point_m.y = fminf(min_point_m.y, point_m.y);
        max_point_m.x = fmaxf(max_point_m.x, point_m.x);
        max_point_m.y = fmaxf(max_point_m.y, point_m.y);
    }
    
    // Convert to grid coordinates with padding
    HashKey min_key = _neu_to_grid_key(Vector3f(min_point_m.x, min_point_m.y, 0));
    HashKey max_key = _neu_to_grid_key(Vector3f(max_point_m.x, max_point_m.y, 0));
    
    // Expand and skip cells
    min_key.x = MAX(0, min_key.x - 2);
    min_key.y = MAX(0, min_key.y - 2);
    max_key.x = MIN(OA_SPATIAL_HASH_BUCKETS - 1, max_key.x + 2);
    max_key.y = MIN(OA_SPATIAL_HASH_BUCKETS - 1, max_key.y + 2);
    
    bool success = true;
    
    // OPTIMIZATION: Skip cells for faster insertion
    for (int x = min_key.x; x <= max_key.x; x += GRID_SKIP) {
        for (int y = min_key.y; y <= max_key.y; y += GRID_SKIP) {
            HashKey cell_key = {x, y};
            
            // FAST POLYGON CHECK: Use bounding box test first
            if (!_polygon_bbox_overlaps_cell(cell_key, min_point_m, max_point_m)) {
                continue;
            }
            
            bool should_insert = false;
            if (is_inclusion) {
                should_insert = !_polygon_overlaps_cell(cell_key, points_cm, point_count);
            } else {
                should_insert = _polygon_overlaps_cell(cell_key, points_cm, point_count);
            }
            
            if (should_insert) {
                uint16_t bucket_index = _grid_key_to_bucket(cell_key);
                BucketItem* new_item = _alloc_bucket_item();
                if (new_item == nullptr) {
                    success = false;
                    continue;
                }
                
                new_item->obstacle.type = type;
                new_item->obstacle.label = label;
                new_item->obstacle.radius = 0;
                new_item->obstacle.margin = margin;
                new_item->obstacle.timestamp_ms = AP_HAL::millis();
                new_item->obstacle.polygon_points_cm = points_cm;
                new_item->obstacle.polygon_point_count = point_count;
                
                new_item->next = _buckets[bucket_index];
                _buckets[bucket_index] = new_item;
                _num_obstacles++;
            }
        }
    }
    
    return success;
}

// Main polygon insertion - simplified!
/* Optimizing
bool AP_OASpatialHash::_insert_polygon_obstacle(const Vector3f &vehicle_NEU_m, const Vector2f *points_cm, 
                                               uint16_t point_count, float margin, ObstacleType type, 
                                               char *label, bool is_inclusion)
{
    if (point_count < 3) return false;
    
    // Calculate polygon bounding box
    Vector2f min_point_cm = points_cm[0];
    Vector2f max_point_cm = points_cm[0];
    for (uint16_t i = 1; i < point_count; i++) {
        min_point_cm.x = fminf(min_point_cm.x, points_cm[i].x);
        min_point_cm.y = fminf(min_point_cm.y, points_cm[i].y);
        max_point_cm.x = fmaxf(max_point_cm.x, points_cm[i].x);
        max_point_cm.y = fmaxf(max_point_cm.y, points_cm[i].y);
    }
    
    // Convert to grid coordinates  - fence polygons are using cm, need to convert to m
    HashKey min_key = _neu_to_grid_key(Vector3f(min_point_cm.x * .01f, min_point_cm.y * .01f, 0));
    HashKey max_key = _neu_to_grid_key(Vector3f(max_point_cm.x * .01f, max_point_cm.y * .01f, 0));
    
    // Expand search area
    int expand_cells = 2;
    min_key.x = MAX(0, min_key.x - expand_cells);
    min_key.y = MAX(0, min_key.y - expand_cells);
    max_key.x = MIN(OA_SPATIAL_HASH_BUCKETS - 1, max_key.x + expand_cells);
    max_key.y = MIN(OA_SPATIAL_HASH_BUCKETS - 1, max_key.y + expand_cells);
    
    bool success = true;
    
    // Insert into relevant grid cells
    for (int x = min_key.x; x <= max_key.x; x++) {
        for (int y = min_key.y; y <= max_key.y; y++) {
            HashKey cell_key = {x, y};
            
            bool should_insert = false;
            if (is_inclusion) {
                // For inclusion polygons, insert into cells OUTSIDE the polygon
                should_insert = !_polygon_overlaps_cell(cell_key, points_cm, point_count);
            } else {
                // For exclusion polygons, insert into cells INSIDE the polygon
                should_insert = _polygon_overlaps_cell(cell_key, points_cm, point_count);
            }
            
            if (should_insert) {
                uint16_t bucket_index = _grid_key_to_bucket(cell_key);
                BucketItem* new_item = _alloc_bucket_item();
                if (new_item == nullptr) {
                    success = false;
                    continue;
                }
                
                // Create polygon obstacle - just store references, no copying!
                new_item->obstacle.type = type;
                new_item->obstacle.label = label;
                new_item->obstacle.radius = 0;
                new_item->obstacle.margin = margin;
                new_item->obstacle.timestamp_ms = AP_HAL::millis();
                
                // Store polygon reference - no vertex copying!
                new_item->obstacle.polygon_points_cm = points_cm;
                new_item->obstacle.polygon_point_count = point_count;
                
                // Insert into bucket
                new_item->next = _buckets[bucket_index];
                _buckets[bucket_index] = new_item;
                _num_obstacles++;
            }
        }
    }
    
    return success;
}*/

// Fast bounding box test
bool AP_OASpatialHash::_polygon_bbox_overlaps_cell(const HashKey& cell_key, const Vector2f& poly_min_m, const Vector2f& poly_max_m) const
{
    Vector3f cell_min_m = _grid_key_to_neu(cell_key);
    Vector3f cell_max_m = cell_min_m;
    cell_max_m.x += GRID_CELL_SIZE;
    cell_max_m.y += GRID_CELL_SIZE;
    
    // Simple AABB overlap test
    return !(cell_max_m.x < poly_min_m.x || cell_min_m.x > poly_max_m.x ||
             cell_max_m.y < poly_min_m.y || cell_min_m.y > poly_max_m.y);
}