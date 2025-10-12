#include "AP_OASpatialHash.h"
#include <AP_AHRS/AP_AHRS.h>

AP_OASpatialHash::AP_OASpatialHash()
{
}

/*void AP_OASpatialHash::set_origin(const Location &origin)
{
    _origin = origin;
    _origin_set = true;
}*/

void AP_OASpatialHash::clear()
{
    _free_all_bucket_items();
    _num_obstacles = 0;
}

/*Vector3f AP_OASpatialHash::_location_to_neu(const Location &loc) const
{
    if (!_origin_set) {
        return Vector3f(0, 0, 0);
    }
    
    Vector3f neu;
    
    // Calculate bearing and distance from origin to location
    float bearing_deg = _origin.get_bearing_to(loc); // Returns degrees
    float distance_cm = _origin.get_distance(loc);   // Returns centimeters
    float distance_m = distance_cm * 0.01f;          // Convert cm to meters
    
    // Convert to NEU components (North-East-Up)
    float bearing_rad = radians(bearing_deg);
    neu.x = cosf(bearing_rad) * distance_m;  // North (meters)
    neu.y = sinf(bearing_rad) * distance_m;  // East (meters)
    neu.z = (loc.alt - _origin.alt) * 0.01f; // Up (meters), convert cm to m
    
    return neu;
}*/

AP_OASpatialHash::HashKey AP_OASpatialHash::_neu_to_grid_key(const Vector3f &neu) const
{
    HashKey key;
    key.x = (int32_t)(neu.x / GRID_CELL_SIZE);
    key.y = (int32_t)(neu.y / GRID_CELL_SIZE);
    return key;
}

uint16_t AP_OASpatialHash::_grid_key_to_bucket(const HashKey &key) const
{
    // Simple hash function that distributes keys across buckets
    uint32_t hash = (uint32_t)(key.x) * 73856093U ^ (uint32_t)(key.y) * 19349663U;
    return hash % OA_SPATIAL_HASH_BUCKETS;
}

AP_OASpatialHash::BucketItem* AP_OASpatialHash::_alloc_bucket_item()
{
    if (_pool_used >= OA_SPATIAL_HASH_POOL_SIZE) {
        return nullptr;
    }
    return &_pool[_pool_used++];
}

void AP_OASpatialHash::_free_all_bucket_items()
{
    for (uint16_t i = 0; i < OA_SPATIAL_HASH_BUCKETS; i++) {
        _buckets[i] = nullptr;
    }
    _pool_used = 0;
}

bool AP_OASpatialHash::_insert_point_obstacle(const Location &center, float radius,  uint32_t timestamp_ms, ObstacleType type)
{
    Vector3f center_neu_m;
    if (!center.get_vector_from_origin_NEU_m(center_neu_m)) {
        return false;
    }
    return _insert_point_obstacle(center_neu_m, radius, timestamp_ms, type);
}

bool AP_OASpatialHash::_insert_point_obstacle(const Vector3f &neu_pos, float radius,  uint32_t timestamp_ms, ObstacleType type)
{
    HashKey key = _neu_to_grid_key(neu_pos);
    uint16_t bucket = _grid_key_to_bucket(key);
    
    BucketItem* new_item = _alloc_bucket_item();
    if (new_item == nullptr) {
        return false; // Pool exhausted
    }
    
    new_item->obstacle.type = type;
    new_item->obstacle.pos = neu_pos;
    new_item->obstacle.radius = radius;
    new_item->obstacle.timestamp_ms = timestamp_ms;
    new_item->next = _buckets[bucket];
    _buckets[bucket] = new_item;
    
    _num_obstacles++;

    return true;
}

bool AP_OASpatialHash::_insert_line_obstacle(const Vector3f &start_neu, const Vector3f &end_neu, ObstacleType type)
{
    // Insert both endpoints
    _insert_point_obstacle(start_neu, 0.1f, 0, type);
    _insert_point_obstacle(end_neu, 0.1f, 0, type);
    
    // Sample intermediate points for long lines
    float distance = (end_neu - start_neu).xy().length();
    if (distance > GRID_CELL_SIZE) {
        uint8_t samples = MIN(8, (uint8_t)(distance / GRID_CELL_SIZE));
        for (uint8_t i = 1; i < samples; i++) {
            float ratio = (float)i / samples;
            Vector3f intermediate = start_neu + (end_neu - start_neu) * ratio;
            _insert_point_obstacle(intermediate, 0.1f, 0, type);
        }
    }
    
    // Store the line segment for accurate distance calculation
    HashKey key = _neu_to_grid_key((start_neu + end_neu) * 0.5f);
    uint16_t bucket = _grid_key_to_bucket(key);
    
    BucketItem* new_item = _alloc_bucket_item();
    if (new_item == nullptr) {
        return false;
    }
    
    new_item->obstacle.type = type;
    new_item->obstacle.pos = start_neu;
    new_item->obstacle.pos2 = end_neu;
    new_item->obstacle.radius = 0.1f;
    new_item->obstacle.timestamp_ms = 0;
    new_item->next = _buckets[bucket];
    _buckets[bucket] = new_item;
    
    _num_obstacles++;

    return true;
}

// Public insert methods
bool AP_OASpatialHash::insert_database_obstacle(const Vector3f &neu_pos, float radius, uint32_t timestamp_ms)
{
    return _insert_point_obstacle(neu_pos, radius, timestamp_ms, OBSTACLE_DATABASE);
}

bool AP_OASpatialHash::_insert_line_obstacle(const Location &start, const Location &end, ObstacleType type)
{
    Vector3f start_neu_m, end_neu_m;
    if (!start.get_vector_from_origin_NEU_m(start_neu_m)) {
        return false;
    }
    if (!end.get_vector_from_origin_NEU_m(end_neu_m)) {
        return false;
    }
    return _insert_line_obstacle(start_neu_m, end_neu_m, type);
}

bool AP_OASpatialHash::insert_fence_inclusion_circle(const Vector3f &center_NEU_m, float radius)
{
    if (radius <= GRID_CELL_SIZE * 2.0f) {
        // Small circles: insert as a single point with radius
        return _insert_point_obstacle(center_NEU_m, radius, 0, FENCE_INCLUSION_CIRCLE);
    } else {
        // Large inclusion circles: treat as boundary - insert multiple rings
        return _insert_large_inclusion_circle(center_NEU_m, radius);
    }
}

bool AP_OASpatialHash::insert_fence_exclusion_circle(const Vector3f &center_NEU_m, float radius)
{
    if (radius <= GRID_CELL_SIZE * 3.0f) {
        // Small exclusion circles: insert as a single point with radius
        // Use slightly larger threshold since these represent physical obstacles
        return _insert_point_obstacle(center_NEU_m, radius, 0, FENCE_EXCLUSION_CIRCLE);
    } else {
        // Large exclusion circles: sample the boundary densely
        return _insert_large_exclusion_circle(center_NEU_m, radius);
    }
}

bool AP_OASpatialHash::_insert_large_inclusion_circle(const Vector3f &center, float radius)
{
    // For large inclusion circles (safe perimeters), we need to ensure
    // the entire boundary is well-represented in the spatial hash
    
    // Calculate appropriate sampling based on circle size
    float circumference = 2 * M_PI * radius;
    uint16_t samples = MIN(48, MAX(24, (uint16_t)(circumference / GRID_CELL_SIZE)));
    
    // Sample the outer boundary
    for (uint16_t i = 0; i < samples; i++) {
        float angle_rad = (2 * M_PI * i) / samples;
        Vector3f point = center;
        point.x += cosf(angle_rad) * radius;
        point.y += sinf(angle_rad) * radius;
        if (!_insert_point_obstacle(point, 0.1f, 0, FENCE_INCLUSION_CIRCLE)) {
            return false;
        }
    }
    
    // For very large circles, also sample an inner ring to improve coverage
    if (radius > GRID_CELL_SIZE * 10.0f) {
        float inner_radius = radius - GRID_CELL_SIZE * 2.0f;
        uint16_t inner_samples = samples / 2;
        for (uint16_t i = 0; i < inner_samples; i++) {
            float angle_rad = (2 * M_PI * i) / inner_samples;
            Vector3f point = center;
            point.x += cosf(angle_rad) * inner_radius;
            point.y += sinf(angle_rad) * inner_radius;
            if(!_insert_point_obstacle(point, 0.1f, 0, FENCE_INCLUSION_CIRCLE)) {
                return false;
            }
        }
    }
    return true;
}

bool AP_OASpatialHash::_insert_large_exclusion_circle(const Vector3f &center, float radius)
{
    // For large exclusion circles (airspace boundaries, etc.),
    // we need dense sampling to ensure accurate avoidance
    
    float circumference = 2 * M_PI * radius;
    uint16_t samples = MIN(64, MAX(32, (uint16_t)(circumference / (GRID_CELL_SIZE * 0.5f))));
    
    // Very dense sampling of the boundary
    for (uint16_t i = 0; i < samples; i++) {
        float angle_rad = (2 * M_PI * i) / samples;
        Vector3f point = center;
        point.x += cosf(angle_rad) * radius;
        point.y += sinf(angle_rad) * radius;
        if (!_insert_point_obstacle(point, 0.1f, 0, FENCE_EXCLUSION_CIRCLE)) {
            return false;
        }
    }
    
    // For regulatory airspace boundaries, also consider adding intermediate rings
    if (radius > GRID_CELL_SIZE * 20.0f) {
        // Add two intermediate rings for very large airspace boundaries
        for (uint8_t ring = 1; ring <= 2; ring++) {
            float ring_radius = radius * (0.2f * ring); // 20% and 40% of radius
            uint16_t ring_samples = samples / 2;
            for (uint16_t i = 0; i < ring_samples; i++) {
                float angle_rad = (2 * M_PI * i) / ring_samples;
                Vector3f point = center;
                point.x += cosf(angle_rad) * ring_radius;
                point.y += sinf(angle_rad) * ring_radius;
                if (!_insert_point_obstacle(point, 0.1f, 0, FENCE_EXCLUSION_CIRCLE)) {
                    return false;
                }
            }
        }
    }
    return true;
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

float AP_OASpatialHash::find_closest_obstacle_distance(const Location &loc, float max_search_radius) const
{
    /*if (!_origin_set) {
        return FLT_MAX;
    }*/
    
    //Vector3f neu_pos = _location_to_neu(loc);
    Vector3f loc_NEU_m;
    loc.get_vector_from_origin_NEU_m(loc_NEU_m);
    HashKey center_key = _neu_to_grid_key(loc_NEU_m);
    
    float closest_distance = FLT_MAX;
    uint8_t search_radius = _calculate_search_radius(max_search_radius);
    
    // Search in dynamic radius around the position
    for (int32_t dx = -search_radius; dx <= search_radius; dx++) {
        for (int32_t dy = -search_radius; dy <= search_radius; dy++) {
            HashKey search_key = {center_key.x + dx, center_key.y + dy};
            uint16_t bucket = _grid_key_to_bucket(search_key);
            
            BucketItem* item = _buckets[bucket];
            while (item != nullptr) {
                float distance;
                
                switch (item->obstacle.type) {
                    case OBSTACLE_DATABASE:
                        distance = _distance_to_point(loc_NEU_m, item->obstacle);
                        break;
                    case FENCE_INCLUSION_CIRCLE:
                    case FENCE_EXCLUSION_CIRCLE:
                        // For circles, use the actual circle distance formula
                        // This handles both small circles (stored as center + radius)
                        // and large circles (stored as boundary points)
                        distance = _distance_to_circle(loc_NEU_m, item->obstacle);
                        break;
                    case FENCE_INCLUSION_POLYGON:
                    case FENCE_EXCLUSION_POLYGON:
                        distance = _distance_to_line(loc_NEU_m, item->obstacle);
                        break;
                    default:
                        distance = FLT_MAX;
                        break;
                }
                
                if (distance < closest_distance && distance <= max_search_radius) {
                    closest_distance = distance;
                }
                item = item->next;
            }
        }
    }
    
    return closest_distance;
}

float AP_OASpatialHash::_distance_to_circle(const Vector3f &neu_pos, const Obstacle &obstacle) const
{
    // Calculate distance to circle center
    float dist_to_center = Vector2f(neu_pos.x - obstacle.pos.x, neu_pos.y - obstacle.pos.y).length();
    
    // If this is a small circle stored as center + radius
    if (obstacle.radius > 0.1f) { // Threshold to distinguish from boundary points
        return dist_to_center - obstacle.radius;
    }
    
    // For large circles stored as boundary points, the "radius" is small (0.1m)
    // but we need to calculate distance to the actual circle boundary
    // Since we've sampled the boundary densely, the closest point should give us
    // a good approximation of the distance to the circle
    return dist_to_center - 0.1f;
}