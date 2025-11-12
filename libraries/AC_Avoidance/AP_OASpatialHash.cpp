#include "AP_OASpatialHash.h"
#include <AP_AHRS/AP_AHRS.h>

//AP_OASpatialHash::AP_OASpatialHash()
//{
//}

// Initialize grid center in constructor
AP_OASpatialHash::AP_OASpatialHash() :
    _grid_center_valid(false),
    _last_recenter_time(0),
    _all_obstacles(OA_SPATIAL_HASH_POOL_SIZE)  // Use pool size as max obstacles
{
    // Initialize buckets
    for (uint16_t i = 0; i < OA_SPATIAL_HASH_BUCKETS; i++) {
        _buckets[i] = nullptr;
    }
    _pool_used = 0;
    _num_obstacles = 0;
}

/*void AP_OASpatialHash::set_origin(const Location &origin)
{
    _origin = origin;
    _origin_set = true;
}

void AP_OASpatialHash::clear()
{
    _free_all_bucket_items();
    _num_obstacles = 0;
}*/

void AP_OASpatialHash::clear()
{
    _free_all_bucket_items();
    _all_obstacles.clear();
    _grid_center_valid = false;
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
}

AP_OASpatialHash::HashKey AP_OASpatialHash::_neu_to_grid_key(const Vector3f &neu) const
{
    HashKey key;
    key.x = (int32_t)(neu.x / GRID_CELL_SIZE);
    key.y = (int32_t)(neu.y / GRID_CELL_SIZE);
    return key;
}*/

// Modified _neu_to_grid_key with dynamic recentering
AP_OASpatialHash::HashKey AP_OASpatialHash::_neu_to_grid_key(const Vector3f &neu) const
{
    HashKey key;
    
    if (!_grid_center_valid) {
        // Fallback to original behavior if grid not centered
        key.x = (int32_t)((neu.x + (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f)) / GRID_CELL_SIZE);
        key.y = (int32_t)((neu.y + (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f)) / GRID_CELL_SIZE);
        return key;
    }
    
    // Convert NEU to grid-relative coordinates
    float grid_rel_x = neu.x - _grid_center_neu.x;
    float grid_rel_y = neu.y - _grid_center_neu.y;
    
    // Convert to grid indices
    key.x = (int32_t)((grid_rel_x + (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f)) / GRID_CELL_SIZE);
    key.y = (int32_t)((grid_rel_y + (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f)) / GRID_CELL_SIZE);
    
    return key;
}

// Set current position and recenter if needed
void AP_OASpatialHash::set_current_position(const Vector3f& current_position_neu)
{
    _recenter_grid(current_position_neu);
}

// Main recentering method
void AP_OASpatialHash::_recenter_grid(const Vector3f& current_position_neu)
{
    uint32_t now = AP_HAL::millis();
    
    // Only recenter periodically
    if (_grid_center_valid && (now - _last_recenter_time) < 5000) { // 5 seconds
        return;
    }
    
    Vector2f new_center(current_position_neu.x, current_position_neu.y);
    
    if (!_grid_center_valid) {
        // First time initialization
        _grid_center_neu = new_center;
        _grid_center_valid = true;
        _last_recenter_time = now;
        return;
    }
    
    // Check if we need to recenter (if we're near the edge)
    Vector2f offset_from_center = new_center - _grid_center_neu;
    float recenter_threshold = OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.3f; // 30% from center
    
    if (offset_from_center.length() > recenter_threshold) {
        _slide_grid(new_center);
    }
    
    _last_recenter_time = now;
}

// Slide the grid to new center and migrate obstacles
void AP_OASpatialHash::_slide_grid(const Vector2f& new_center)
{
    // Calculate the offset in grid cells
    Vector2f offset = new_center - _grid_center_neu;
    int dx = (int)(offset.x / GRID_CELL_SIZE);
    int dy = (int)(offset.y / GRID_CELL_SIZE);
    
    if (dx == 0 && dy == 0) {
        return; // No significant movement
    }
    
    // Update grid center
    _grid_center_neu = new_center;
    
    // Clear the current hash table (but keep the obstacle master list)
    _free_all_bucket_items();
    
    // Re-insert all obstacles with new grid coordinates
    for (uint16_t i = 0; i < _all_obstacles.size(); i++) {
        Obstacle* obs = _all_obstacles[i];
        if (obs == nullptr) {
            continue;
        }
        
        // Re-insert based on obstacle type
        switch (obs->type) {
            case OBSTACLE_DATABASE:
            case FENCE_INCLUSION_CIRCLE:
            case FENCE_EXCLUSION_CIRCLE:
                _insert_point_obstacle_to_hash(obs->pos, obs->radius, obs->margin, obs->timestamp_ms, obs->type);
                break;
                
            case FENCE_INCLUSION_POLYGON:
            case FENCE_EXCLUSION_POLYGON:
                _insert_line_obstacle_to_hash(obs->pos, obs->pos2, obs->margin, obs->type);
                break;
        }
    }
}

// Collect all obstacles from buckets into master list
void AP_OASpatialHash::_update_obstacle_master_list()
{
    _all_obstacles.clear();
    
    for (uint16_t i = 0; i < OA_SPATIAL_HASH_BUCKETS; i++) {
        BucketItem* item = _buckets[i];
        while (item != nullptr) {
            // Check if this obstacle is already in the master list
            // Check if this obstacle is already in the master list
            bool found = false;
            for (uint16_t j = 0; j < _all_obstacles.size(); j++) {
                Obstacle* existing = _all_obstacles[j];
                if (existing != nullptr &&
                    existing->type == item->obstacle.type &&
                    fabsf(existing->pos.x - item->obstacle.pos.x) < 0.1f &&
                    fabsf(existing->pos.y - item->obstacle.pos.y) < 0.1f &&
                    fabsf(existing->radius - item->obstacle.radius) < 0.1f) {
                    found = true;
                    break;
                }
            }
            
            if (!found && _all_obstacles.available() > 0) {
                // Use the correct ObjectArray method
                _all_obstacles.push(item->obstacle);
            }
            
            item = item->next;
        }
    }
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

// Insert a reference to an existing obstacle into a specific cell
bool AP_OASpatialHash::_insert_obstacle_reference(Obstacle& obstacle, const HashKey& cell_key)
{
    uint16_t bucket_index = _grid_key_to_bucket(cell_key);
    BucketItem* item = _alloc_bucket_item();
    if (item == nullptr) {
        return false;
    }
    
    // Store a reference to the existing obstacle
    item->obstacle = obstacle;  // This copies the obstacle data
    item->next = _buckets[bucket_index];
    _buckets[bucket_index] = item;
    _num_obstacles++;
    return true;
}

// Create a new bucket and item and populate them, returning the item
AP_OASpatialHash::BucketItem* AP_OASpatialHash::_new_bucket_item(const Vector3f &neu_pos, float radius,  float margin, uint32_t timestamp_ms, ObstacleType type, uint16_t bucket)
{
    BucketItem* new_item = _alloc_bucket_item();
    if (new_item == nullptr) {
        return nullptr; // Pool exhausted
    }
    
    new_item->obstacle.type = type;
    new_item->obstacle.pos = neu_pos;
    new_item->obstacle.radius = radius;
    new_item->obstacle.margin = margin;
    new_item->obstacle.timestamp_ms = timestamp_ms;
    new_item->next = _buckets[bucket];
    _buckets[bucket] = new_item;
    
    _num_obstacles++;

   return new_item;
}


bool AP_OASpatialHash::_insert_point_obstacle_loc(const Location &center, float radius, float margin, uint32_t timestamp_ms, ObstacleType type)
{
    Vector3f center_neu_m;
    if (!center.get_vector_from_origin_NEU_m(center_neu_m)) {
        return false;
    }
    if(!_insert_point_obstacle_neu(center_neu_m, radius, margin, timestamp_ms, type)) {
        return false;
    }
    _update_obstacle_master_list();
    return true;
}
bool AP_OASpatialHash::_insert_point_obstacle_neu(const Vector3f &neu_pos, float radius, float margin, uint32_t timestamp_ms, ObstacleType type)
{
    _recenter_grid(neu_pos);
    return _insert_point_obstacle_to_hash(neu_pos, radius, margin, timestamp_ms, type);
}

bool AP_OASpatialHash::_insert_point_obstacle_to_hash(const Vector3f &neu_pos, float radius, float margin, uint32_t timestamp_ms, ObstacleType type)
{
    HashKey key = _neu_to_grid_key(neu_pos);
    uint16_t bucket = _grid_key_to_bucket(key);
    
    BucketItem* new_item = _new_bucket_item(neu_pos, radius, margin, timestamp_ms, type, bucket);
    if (new_item == nullptr) {
        return false;
    }
    else {
        return true;
    }

    new_item->obstacle.type = type;
    new_item->obstacle.pos = neu_pos;
    new_item->obstacle.radius = radius;
    new_item->obstacle.margin = margin;
    new_item->obstacle.timestamp_ms = timestamp_ms;
    new_item->next = _buckets[bucket];
    _buckets[bucket] = new_item;
    
    _num_obstacles++;

    return true;
}

bool AP_OASpatialHash::_insert_line_obstacle_loc(const Location &start, const Location &end, float margin, ObstacleType type)
{
    Vector3f start_neu_m, end_neu_m;
    if (!start.get_vector_from_origin_NEU_m(start_neu_m)) {
        return false;
    }
    if (!end.get_vector_from_origin_NEU_m(end_neu_m)) {
        return false;
    }
    if(!_insert_line_obstacle_neu(start_neu_m, end_neu_m, margin, type)) {
        return false;
    }
    _update_obstacle_master_list();
    return true;
}

bool AP_OASpatialHash::_insert_line_obstacle_neu(const Vector3f &start_neu, const Vector3f &end_neu, float margin, ObstacleType type)
{
    // Ensure grid is centered before insertion (use midpoint)
    Vector3f midpoint((start_neu.x + end_neu.x) * 0.5f, 
                     (start_neu.y + end_neu.y) * 0.5f, 0);
    _recenter_grid(midpoint);

    return _insert_line_obstacle_to_hash(start_neu, end_neu, margin, type);
}

bool AP_OASpatialHash::_insert_line_obstacle_to_hash(const Vector3f &start_neu, const Vector3f &end_neu, float margin, ObstacleType type)
{
    // Insert both endpoints
    _insert_point_obstacle_neu(start_neu, 0.1f, 0, margin, type);
    _insert_point_obstacle_neu(end_neu, 0.1f, 0, margin, type);
    
    // Sample intermediate points for long lines
    float distance = (end_neu - start_neu).xy().length();
    if (distance > GRID_CELL_SIZE) {
        uint8_t samples = MIN(8, (uint8_t)(distance / GRID_CELL_SIZE));
        for (uint8_t i = 1; i < samples; i++) {
            float ratio = (float)i / samples;
            Vector3f intermediate = start_neu + (end_neu - start_neu) * ratio;
            _insert_point_obstacle_neu(intermediate, 0.1f, 0, margin, type);
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
    new_item->obstacle.margin = margin;
    new_item->obstacle.timestamp_ms = 0;
    new_item->next = _buckets[bucket];
    _buckets[bucket] = new_item;
    
    _num_obstacles++;

    return true;
}

// Public insert methods
bool AP_OASpatialHash::insert_database_obstacle(const Vector3f &neu_pos, float radius, float margin, uint32_t timestamp_ms)
{
    // Grid recenter happens in insert_point_obstacle_neu
    if (!_insert_point_obstacle_neu(neu_pos, radius, timestamp_ms, margin, OBSTACLE_DATABASE)) {
        return false;
    }
    _update_obstacle_master_list();
    return true;
}

bool AP_OASpatialHash::insert_fence_inclusion_polygon(const Vector3f &start_NEU_m, const Vector3f &end_NEU_m, float margin)     
{ 
    // Grid recenter happens in insert_line_obstacle_neu
    if(!_insert_line_obstacle_neu(start_NEU_m, end_NEU_m, margin, FENCE_INCLUSION_POLYGON)) {
        return false;
    }
    _update_obstacle_master_list();
    return true;
}

bool AP_OASpatialHash::insert_fence_exclusion_polygon(const Vector3f &start_NEU_m, const Vector3f &end_NEU_m, float margin)     
{ 
    // Grid recenter happens in insert_line_obstacle_neu
    if(!_insert_line_obstacle_neu(start_NEU_m, end_NEU_m, margin, FENCE_EXCLUSION_POLYGON)) {
        return false;
    }
    _update_obstacle_master_list();
    return true;
}


bool AP_OASpatialHash::insert_fence_exclusion_circle(const Vector3f &center_NEU_m, float radius)
{
    // Grid recenter happens in insert_exclusion_circle_multicell
    if(!_insert_exclusion_circle_multicell(center_NEU_m, radius)) {
        return false;
    }
    _update_obstacle_master_list();
    return true;
}

bool AP_OASpatialHash::_insert_exclusion_circle_multicell(const Vector3f &center, float radius)
{
    // Ensure grid is centered before insertion
    _recenter_grid(center);

    // For exclusion circles, insert into all cells that the circle overlaps
    HashKey center_key = _neu_to_grid_key(center);
    
    // Calculate number of cells the radius covers
    int cells_radius = (int)ceilf(radius / GRID_CELL_SIZE);
    
    // Create the obstacle once
    BucketItem* obstacle_item = _alloc_bucket_item();
    if (obstacle_item == nullptr) {
        return false;
    }
    
    obstacle_item->obstacle.type = FENCE_EXCLUSION_CIRCLE;
    obstacle_item->obstacle.pos = center;
    obstacle_item->obstacle.radius = radius;
    obstacle_item->obstacle.timestamp_ms = AP_HAL::millis();
    
    bool success = true;
    
    // Insert the SAME obstacle into all cells within the circle's radius
    for (int dx = -cells_radius; dx <= cells_radius; dx++) {
        for (int dy = -cells_radius; dy <= cells_radius; dy++) {
            HashKey cell_key = {center_key.x + dx, center_key.y + dy};
            
            // Check if cell is within grid bounds
            if (cell_key.x >= 0 && cell_key.x < OA_SPATIAL_HASH_BUCKETS &&
                cell_key.y >= 0 && cell_key.y < OA_SPATIAL_HASH_BUCKETS) {
                
                // Create a new bucket item that points to the same obstacle
                BucketItem* cell_item = _alloc_bucket_item();
                if (cell_item == nullptr) {
                    success = false;
                    continue;
                }
                
                // Copy the obstacle data (not reference, but same data)
                cell_item->obstacle = obstacle_item->obstacle;
                cell_item->next = _buckets[_grid_key_to_bucket(cell_key)];
                _buckets[_grid_key_to_bucket(cell_key)] = cell_item;
                _num_obstacles++;
            }
        }
    }
    
    return success;
}

bool AP_OASpatialHash::insert_fence_inclusion_circle(const Vector3f &center_NEU_m, float radius)
{
    // Grid recenter happens in insert_inclusion_circle_multicell
    if(!_insert_inclusion_circle_multicell(center_NEU_m, radius)) {
        return false;
    }
    _update_obstacle_master_list();
    return true;
}

bool AP_OASpatialHash::_insert_inclusion_circle_multicell(const Vector3f &center, float radius)
{
    // Ensure grid is centered before insertion
    _recenter_grid(center);

    // For inclusion circles, insert into cells OUTSIDE the circle
    HashKey center_key = _neu_to_grid_key(center);
    
    // Calculate the area we need to cover (cells outside the circle)
    int coverage_radius = fmin(OA_SPATIAL_HASH_BUCKETS / 2, (int)ceilf(radius * 3 / GRID_CELL_SIZE));
    
    // Create the obstacle once
    BucketItem* obstacle_item = _alloc_bucket_item();
    if (obstacle_item == nullptr) {
        return false;
    }
    
    obstacle_item->obstacle.type = FENCE_INCLUSION_CIRCLE;
    obstacle_item->obstacle.pos = center;
    obstacle_item->obstacle.radius = radius;
    obstacle_item->obstacle.timestamp_ms = AP_HAL::millis();
    
    bool success = true;
    
    // Insert the SAME obstacle into cells outside the inclusion circle
    for (int dx = -coverage_radius; dx <= coverage_radius; dx++) {
        for (int dy = -coverage_radius; dy <= coverage_radius; dy++) {
            HashKey cell_key = {center_key.x + dx, center_key.y + dy};
            
            // Check if cell is within grid bounds
            if (cell_key.x >= 0 && cell_key.x < OA_SPATIAL_HASH_BUCKETS &&
                cell_key.y >= 0 && cell_key.y < OA_SPATIAL_HASH_BUCKETS) {
                
                // Simple check: if cell center is outside the circle, insert obstacle
                Vector3f cell_center_neu;
                cell_center_neu.x = (cell_key.x + 0.5f) * GRID_CELL_SIZE - (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f);
                cell_center_neu.y = (cell_key.y + 0.5f) * GRID_CELL_SIZE - (OA_SPATIAL_HASH_BUCKETS * GRID_CELL_SIZE * 0.5f);
                
                float distance = Vector2f(center.x - cell_center_neu.x, center.y - cell_center_neu.y).length();
                
                if (distance > radius) {
                    // Create a new bucket item that points to the same obstacle data
                    BucketItem* cell_item = _alloc_bucket_item();
                    if (cell_item == nullptr) {
                        success = false;
                        continue;
                    }
                    
                    // Copy the obstacle data
                    cell_item->obstacle = obstacle_item->obstacle;
                    cell_item->next = _buckets[_grid_key_to_bucket(cell_key)];
                    _buckets[_grid_key_to_bucket(cell_key)] = cell_item;
                    _num_obstacles++;
                }
            }
        }
    }
    
    return success;
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

float AP_OASpatialHash::find_closest_obstacle_distance(const Location &loc, float max_search_radius) const
{
    //Vector3f neu_pos = _location_to_neu(loc);
    Vector3f loc_NEU_m;
    loc.get_vector_from_origin_NEU_m(loc_NEU_m);
    // Recenter grid (need to cast away const)
    const_cast<AP_OASpatialHash*>(this)->_recenter_grid(loc_NEU_m);

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
                        distance = _distance_to_circle_boundary(loc_NEU_m, item->obstacle);
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

float AP_OASpatialHash::_distance_to_circle_boundary(const Vector3f &neu_pos, const Obstacle &obstacle) const
{
    // Calculate exact distance to circle boundary
    Vector2f pos_2d(neu_pos.x, neu_pos.y);
    Vector2f center_2d(obstacle.pos.x, obstacle.pos.y);
    
    float dist_to_center = (pos_2d - center_2d).length();
    
    // Return distance to circle boundary (positive outside, negative inside)
    return dist_to_center - obstacle.radius;
}

float AP_OASpatialHash::find_closest_obstacle_to_line(const Location& start, const Location& end, float margin) const
{
    // Convert start and end locations to NEU coordinates
    Vector3f start_NEU_m, end_NEU_m;
    start.get_vector_from_origin_NEU_m(start_NEU_m);
    end.get_vector_from_origin_NEU_m(end_NEU_m);
    
    // Recenter grid if needed (use midpoint of line segment)
    Vector3f midpoint_NEU_m((start_NEU_m.x + end_NEU_m.x) * 0.5f, 
                     (start_NEU_m.y + end_NEU_m.y) * 0.5f, 0);
    const_cast<AP_OASpatialHash*>(this)->_recenter_grid(midpoint_NEU_m);

    // Get grid keys for start and end using existing method
    HashKey start_key = _neu_to_grid_key(start_NEU_m);
    HashKey end_key = _neu_to_grid_key(end_NEU_m);
    
    float closest_distance = margin;
    
    // Use DDA algorithm for grid traversal between the keys
    int x0 = start_key.x;
    int y0 = start_key.y;
    int x1 = end_key.x;
    int y1 = end_key.y;
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    
    int x = x0;
    int y = y0;
    
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    
    int error = dx - dy;
    dx *= 2;
    dy *= 2;
    
    // Maximum number of cells to check
    const uint16_t max_cells = dx + dy + 1;
    uint16_t cells_checked = 0;
    
    while (cells_checked < max_cells) {
        // Create hash key for current cell
        HashKey current_key = {x, y};
        
        // Get bucket index for this key
        uint16_t bucket_index = _grid_key_to_bucket(current_key);
        
        // Check all obstacles in this bucket
        BucketItem* item = _buckets[bucket_index];
        while (item != nullptr) {
            Obstacle& obstacle = item->obstacle;
            
            // Calculate distance based on obstacle type
            float distance = FLT_MAX;
            
            switch (obstacle.type) {
                case OBSTACLE_DATABASE:
                    // Point obstacle with radius
                    distance = _distance_to_point(start_NEU_m, obstacle) - obstacle.radius;
                    break;
                    
                case FENCE_INCLUSION_POLYGON:
                case FENCE_EXCLUSION_POLYGON:
                    // Line segment obstacle - we need segment-to-segment distance, subtract the margin to give the distance to the margin
                    distance = _distance_between_segments(start_NEU_m, end_NEU_m, obstacle.pos, obstacle.pos2) - obstacle.margin;
                    break;
                    
                case FENCE_INCLUSION_CIRCLE:
                    // Circle obstacle - find closest distance between line segment and circle, subtract the margin as we need to be that much further inside to be ok
                    distance = _distance_between_line_and_circle(start_NEU_m, end_NEU_m, obstacle.pos, obstacle.radius - obstacle.margin);
                    break;
                case FENCE_EXCLUSION_CIRCLE:
                    // Circle obstacle - find closest distance between line segment and circle, add the margin as we have to be that much further away from the circle to be ok
                    distance = _distance_between_line_and_circle(start_NEU_m, end_NEU_m, obstacle.pos, obstacle.radius + obstacle.margin);
                    break;
            }
            
            if (distance < closest_distance) {
                closest_distance = distance;
                
                // If we hit an obstacle (distance <= 0), return early
                if (distance <= 0.0f) {
                    return closest_distance;
                }
            }
            item = item->next;
        }
        
        // Break if we've reached the end cell
        if (x == x1 && y == y1) {
            break;
        }
        
        // Move to next cell using DDA
        if (error > 0) {
            x += x_inc;
            error -= dy;
        } else {
            y += y_inc;
            error += dx;
        }
        
        cells_checked++;
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