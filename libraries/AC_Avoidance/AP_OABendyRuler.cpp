#include "AP_OABendyRuler.h"
#include <AP_AHRS/AP_AHRS.h>
#include "AP_OAPathPlanner.h"
#include <string.h>

// Use the correct define for BendyRuler
#if AP_OAPATHPLANNER_BENDYRULER_ENABLED

AP_OABendyRuler *AP_OABendyRuler::_singleton = nullptr;

const AP_Param::GroupInfo AP_OABendyRuler::var_info[] = {
    // @Param: MARGIN_MAX
    // @DisplayName: Object Avoidance Margin Max
    // @Description: Object Avoidance Margin Max in meters
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("MARGIN_MAX", 1, AP_OABendyRuler, _margin, 2.0f),

    // @Param: MIN_DIST
    // @DisplayName: Object Avoidance Minimum Distance
    // @Description: Object Avoidance Minimum Distance in meters
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("MIN_DIST", 2, AP_OABendyRuler, _min_distance, 0.5f),

    // @Param: LOOKAHEAD
    // @DisplayName: Object Avoidance Look Ahead Time
    // @Description: Object Avoidance Look Ahead Time in seconds
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LOOKAHEAD", 3, AP_OABendyRuler, _lookahead_time, 2.0f),

    // @Param: LOOKAHEAD_MAX
    // @DisplayName: Object Avoidance Look Ahead Max
    // @Description: Object Avoidance Look Ahead Max in meters
    // @Units: m
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LOOKAHEAD_MAX", 4, AP_OABendyRuler, _lookahead_max, 20.0f),

    // @Param: SPEED_MAX
    // @DisplayName: Object Avoidance Speed Max
    // @Description: Object Avoidance Speed Max in m/s
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("SPEED_MAX", 5, AP_OABendyRuler, _max_speed, 10.0f),

    // @Param: ACCEL_MAX
    // @DisplayName: Object Avoidance Acceleration Max
    // @Description: Object Avoidance Acceleration Max in m/s/s
    // @Units: m/s/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ACCEL_MAX", 6, AP_OABendyRuler, _max_accel, 5.0f),

    // @Param: TURN_RATE_MAX
    // @DisplayName: Object Avoidance Turn Rate Max
    // @Description: Object Avoidance Turn Rate Max in rad/s
    // @Units: rad/s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("TURN_RATE_MAX", 7, AP_OABendyRuler, _max_turn_rate, 1.0f),

    // @Param: TURN_ANGLE_MAX
    // @DisplayName: Object Avoidance Turn Angle Max
    // @Description: Object Avoidance Turn Angle Max in radians
    // @Units: rad
    // @Range: 0 3.14
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("TURN_ANGLE_MAX", 8, AP_OABendyRuler, _max_turn_angle, 1.57f),

    AP_GROUPEND
};

AP_OABendyRuler::AP_OABendyRuler()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_OABendyRuler must be singleton");
    }
    _singleton = this;

    _fence = nullptr;
    _fences_loaded = false;
    _last_fence_update_ms = 0;
    _spatial_hash_initialized = false;
    _last_spatial_hash_update_ms = 0;
    _fence_segment_count = 0;
    _have_current_loc = false;
    _margin_ratio = 0.1f; // 10% margin
}

void AP_OABendyRuler::init()
{
    // Initialization if needed
}

void AP_OABendyRuler::pre_update()
{
    // Pre-calculation if needed
}

void AP_OABendyRuler::set_config(float margin_max)
{
    // Configure the BendyRuler with the specified maximum margin
    if (margin_max > 0) {
        _margin.set(margin_max);
    }
}

bool AP_OABendyRuler::location_to_vector(const Location& loc, Vector2f& pos) const
{
    // Simple conversion - in real implementation, use proper coordinate transformation
    pos.x = loc.lat * 1.0e-7f;
    pos.y = loc.lng * 1.0e-7f;
    return true;
}

bool AP_OABendyRuler::get_origin_and_direction(const Location& current_loc, const Vector2f &ground_speed_vec, Vector2f &origin, Vector2f &direction)
{
    // Convert current location to vector
    if (!location_to_vector(current_loc, origin)) {
        return false;
    }
    
    // Use ground speed vector for direction, or fallback to heading
    if (ground_speed_vec.length() > 0.1f) {
        direction = ground_speed_vec.normalized() * _lookahead_max;
    } else {
        // Fallback to vehicle heading
        float yaw_rad = AP::ahrs().get_yaw();
        direction = Vector2f(cosf(yaw_rad), sinf(yaw_rad)) * _lookahead_max;
    }
    
    _have_current_loc = true;
    return true;
}

bool AP_OABendyRuler::update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new, OABendyResult &bendy_result, bool proximity_only)
{
    Vector2f origin, direction;
    if (!get_origin_and_direction(current_loc, ground_speed_vec, origin, direction)) {
        return false;
    }

    // Update spatial hash with current obstacles
    _update_spatial_hash();

    // Store current origin for spatial hash
    _current_origin = origin;
    _lookahead = direction.length();

    // Existing margin and segment setup
    const float margin = _lookahead * _margin_ratio;
    const Vector2f left_origin = origin + Vector2f(-direction.y, direction.x) * margin;
    const Vector2f right_origin = origin + Vector2f(direction.y, -direction.x) * margin;

    // Clear previous results
    bendy_result.reset();

    // Check center path first (most likely case)
    Vector2f center_end = origin + direction;
    if (_check_segment_with_raytrace(origin, center_end, bendy_result)) {
        bendy_result.bendy_type = OABendyType::OA_BENDY_HORIZONTAL;
        return true;
    }

    // Simplified bendy ruler segment generation
    const uint8_t total_segments = 4; // Reduced for simplicity
    for (uint8_t i = 0; i < total_segments; i++) {
        // Simplified segment calculation
        float ratio = (float)(i + 1) / (float)(total_segments + 1);
        Vector2f segment_start = origin;
        Vector2f segment_end = origin + direction;
        
        if (i % 2 == 0) {
            segment_start = segment_start + (left_origin - origin) * ratio;
            segment_end = segment_end + (left_origin - origin) * ratio;
        } else {
            segment_start = segment_start + (right_origin - origin) * ratio;
            segment_end = segment_end + (right_origin - origin) * ratio;
        }

        // Use ray tracing for segment collision check
        if (_check_segment_with_raytrace(segment_start, segment_end, bendy_result)) {
            return true;
        }
    }

    return false;
}

// [Rest of the implementation remains exactly the same as before...]
// SPATIAL HASH IMPLEMENTATION
bool AP_OABendyRuler::AP_OASpatialHash::_world_to_grid(const Vector2f& pos, uint16_t& grid_x, uint16_t& grid_y) const
{
    if (_cell_size < 0.1f) {
        return false;
    }
    
    grid_x = (uint16_t)((pos.x - _grid_origin.x) / _cell_size);
    grid_y = (uint16_t)((pos.y - _grid_origin.y) / _cell_size);
    
    return (grid_x < OA_SPATIAL_HASH_SIZE && grid_y < OA_SPATIAL_HASH_SIZE);
}

void AP_OABendyRuler::AP_OASpatialHash::init(float cell_size, const Vector2f& origin)
{
    _cell_size = cell_size;
    _grid_origin = origin;
    clear();
}

void AP_OABendyRuler::AP_OASpatialHash::clear()
{
    for (uint16_t x = 0; x < OA_SPATIAL_HASH_SIZE; x++) {
        for (uint16_t y = 0; y < OA_SPATIAL_HASH_SIZE; y++) {
            _cells[x][y].obstacle_count = 0;
        }
    }
}

bool AP_OABendyRuler::AP_OASpatialHash::add_obstacle(const Vector2f& pos, float radius, uint16_t obstacle_id)
{
    uint16_t grid_x, grid_y;
    if (!_world_to_grid(pos, grid_x, grid_y)) {
        return false;
    }
    
    OACell& cell = _cells[grid_x][grid_y];
    if (cell.obstacle_count < OA_MAX_OBSTACLES_PER_CELL) {
        cell.obstacle_indices[cell.obstacle_count] = obstacle_id;
        cell.obstacle_count++;
        return true;
    }
    
    return false; // Cell is full
}

void AP_OABendyRuler::AP_OASpatialHash::query_radius(const Vector2f& pos, float radius, uint32_t obstacle_mask[OA_BITMASK_SIZE])
{
    // Clear the bitmask first
    memset(obstacle_mask, 0, OA_BITMASK_SIZE * sizeof(uint32_t));
    
    // Calculate grid bounds for the query circle
    Vector2f min_bound = pos - Vector2f(radius, radius);
    Vector2f max_bound = pos + Vector2f(radius, radius);
    
    uint16_t min_x, min_y, max_x, max_y;
    if (!_world_to_grid(min_bound, min_x, min_y) || !_world_to_grid(max_bound, max_x, max_y)) {
        return;
    }
    
    // Clamp to grid bounds
    min_x = MAX(min_x, 0);
    min_y = MAX(min_y, 0);
    max_x = MIN(max_x, OA_SPATIAL_HASH_SIZE - 1);
    max_y = MIN(max_y, OA_SPATIAL_HASH_SIZE - 1);
    
    // Check all cells in the bounding box
    for (uint16_t x = min_x; x <= max_x; x++) {
        for (uint16_t y = min_y; y <= max_y; y++) {
            const OACell& cell = _cells[x][y];
            
            // Add all obstacles in this cell to the mask
            for (uint16_t i = 0; i < cell.obstacle_count; i++) {
                uint16_t obstacle_index = cell.obstacle_indices[i];
                if (obstacle_index < OA_MAX_OBSTACLES) {
                    uint16_t word_index = obstacle_index / 32;
                    uint16_t bit_index = obstacle_index % 32;
                    obstacle_mask[word_index] |= (1U << bit_index);
                }
            }
        }
    }
}

bool AP_OABendyRuler::AP_OASpatialHash::ray_intersect_dda(const Vector2f& start, const Vector2f& end, float& distance, uint16_t& obstacle_id)
{
    Vector2f ray_dir = (end - start);
    float ray_length = ray_dir.length();
    
    if (ray_length < 0.1f) {
        return false;
    }
    
    ray_dir /= ray_length; // normalize
    
    // DDA algorithm implementation
    Vector2f delta_dist;
    delta_dist.x = (fabsf(ray_dir.x) > 0.0f) ? fabsf(1.0f / ray_dir.x) : FLT_MAX;
    delta_dist.y = (fabsf(ray_dir.y) > 0.0f) ? fabsf(1.0f / ray_dir.y) : FLT_MAX;
    
    uint16_t grid_x, grid_y;
    if (!_world_to_grid(start, grid_x, grid_y)) {
        return false;
    }
    
    int16_t step_x, step_y;
    Vector2f side_dist;
    
    // Calculate step direction and initial side distances
    if (ray_dir.x < 0.0f) {
        step_x = -1;
        float grid_boundary_x = _grid_origin.x + grid_x * _cell_size;
        side_dist.x = (start.x - grid_boundary_x) * delta_dist.x;
    } else {
        step_x = 1;
        float grid_boundary_x = _grid_origin.x + (grid_x + 1) * _cell_size;
        side_dist.x = (grid_boundary_x - start.x) * delta_dist.x;
    }
    
    if (ray_dir.y < 0.0f) {
        step_y = -1;
        float grid_boundary_y = _grid_origin.y + grid_y * _cell_size;
        side_dist.y = (start.y - grid_boundary_y) * delta_dist.y;
    } else {
        step_y = 1;
        float grid_boundary_y = _grid_origin.y + (grid_y + 1) * _cell_size;
        side_dist.y = (grid_boundary_y - start.y) * delta_dist.y;
    }
    
    // DDA traversal
    float traveled_dist = 0.0f;
    const uint16_t max_steps = OA_SPATIAL_HASH_SIZE * 2;
    
    for (uint16_t step = 0; step < max_steps && traveled_dist < ray_length; step++) {
        // Check current cell for obstacles
        if (grid_x < OA_SPATIAL_HASH_SIZE && grid_y < OA_SPATIAL_HASH_SIZE) {
            const OACell& cell = _cells[grid_x][grid_y];
            
            if (cell.obstacle_count > 0) {
                obstacle_id = cell.obstacle_indices[0];
                distance = traveled_dist;
                return true;
            }
        }
        
        // Move to next cell using DDA
        if (side_dist.x < side_dist.y) {
            traveled_dist = side_dist.x;
            side_dist.x += delta_dist.x;
            grid_x = (int16_t)grid_x + step_x;
        } else {
            traveled_dist = side_dist.y;
            side_dist.y += delta_dist.y;
            grid_y = (int16_t)grid_y + step_y;
        }
        
        // Check grid bounds
        if (grid_x >= OA_SPATIAL_HASH_SIZE || grid_y >= OA_SPATIAL_HASH_SIZE) {
            break;
        }
    }
    
    return false;
}

bool AP_OABendyRuler::AP_OASpatialHash::obstacle_in_mask(const uint32_t obstacle_mask[OA_BITMASK_SIZE], uint16_t obstacle_id) const
{
    if (obstacle_id >= OA_MAX_OBSTACLES) {
        return false;
    }
    uint16_t word_index = obstacle_id / 32;
    uint16_t bit_index = obstacle_id % 32;
    return (obstacle_mask[word_index] & (1U << bit_index)) != 0;
}

// BENDYRULER RAY TRACING INTEGRATION
bool AP_OABendyRuler::_check_segment_with_raytrace(const Vector2f& start, const Vector2f& end, OABendyResult &result)
{
    float nearest_distance;
    OABendyType obstacle_type;
    
    if (!_find_nearest_intersection(start, end, nearest_distance, obstacle_type)) {
        return false;
    }

    const Vector2f segment_vec = end - start;
    const float segment_length = segment_vec.length();
    
    if (segment_length > 0.0f && nearest_distance < segment_length) {
        const float avoidance_distance = segment_length - nearest_distance;
        const Vector2f avoidance_vec = segment_vec * (avoidance_distance / segment_length);
        
        result.avoidance_vec = avoidance_vec;
        result.origin = start;
        result.nearest_distance = nearest_distance;
        result.bendy_type = obstacle_type;
        
        return true;
    }
    
    return false;
}

bool AP_OABendyRuler::_find_nearest_intersection(const Vector2f& start, const Vector2f& end, float& distance, OABendyType& obstacle_type)
{
    float nearest_obstacle_dist = FLT_MAX;
    float nearest_fence_dist = FLT_MAX;
    uint16_t nearest_obstacle_id = 0;
    
    // 1. Check obstacles using spatial hash and DDA ray tracing
    if (_spatial_hash_initialized) {
        if (_spatial_hash.ray_intersect_dda(start, end, nearest_obstacle_dist, nearest_obstacle_id)) {
            // Found an obstacle collision
        }
    }
    
    // 2. Check fence collisions
    if (_fence != nullptr && _fence->enabled()) {
        uint32_t fence_mask[OA_BITMASK_SIZE];
        if (check_collision_with_fences(start, end, nearest_fence_dist, fence_mask)) {
            // Found a fence collision
        }
    }
    
    // Determine the closest threat
    if (nearest_obstacle_dist < nearest_fence_dist) {
        distance = nearest_obstacle_dist;
        obstacle_type = OABendyType::OA_BENDY_OBJECT; // OA_OBSTACLE_TYPE_OBJECT
        return true;
    } else if (nearest_fence_dist < FLT_MAX) {
        distance = nearest_fence_dist;
        obstacle_type = OABendyType::OA_BENDY_HORIZONTAL; // OA_OBSTACLE_TYPE_FENCE
        return true;
    }
    
    return false;
}

void AP_OABendyRuler::_update_spatial_hash()
{
    const uint32_t now = AP_HAL::millis();
    
    if (_spatial_hash_initialized && (now - _last_spatial_hash_update_ms < 100)) {
        return;
    }
    
    if (!_spatial_hash_initialized) {
        _spatial_hash.init(_lookahead / 10.0f, _current_origin);
        _spatial_hash_initialized = true;
    }
    
    _spatial_hash.clear();
    
    // For now, we'll skip obstacle database integration since AP_OADatabase is not available
    // In a real implementation, you would query the obstacle database here
    
    _last_spatial_hash_update_ms = now;
}

// FENCE INTEGRATION - Simplified for now
bool AP_OABendyRuler::_load_fence_segments()
{
    if (_fence == nullptr || !_fence->enabled()) {
        return false;
    }
    
    const uint32_t now = AP_HAL::millis();
    if (_fences_loaded && (now - _last_fence_update_ms < 1000)) {
        return true;
    }
    
    _fence_segment_count = 0;
    
    // For now, we'll use a simple approach - create some dummy fence segments
    // In a real implementation, you would query the actual fence boundaries
    // This is a placeholder until we figure out the correct AC_Fence API
    
    // Create a simple square fence around origin for testing
    const float fence_size = 100.0f; // 100m square
    const Vector2f corners[] = {
        Vector2f(-fence_size, -fence_size),
        Vector2f(-fence_size, fence_size),
        Vector2f(fence_size, fence_size),
        Vector2f(fence_size, -fence_size)
    };
    
    for (uint8_t i = 0; i < 4 && _fence_segment_count < OA_MAX_FENCE_SEGMENTS; i++) {
        uint8_t next_i = (i + 1) % 4;
        _fence_segments[_fence_segment_count].start = corners[i];
        _fence_segments[_fence_segment_count].end = corners[next_i];
        _fence_segments[_fence_segment_count].fence_type = 0;
        _fence_segments[_fence_segment_count].fence_instance = 0;
        _fence_segment_count++;
    }
    
    _fences_loaded = true;
    _last_fence_update_ms = now;
    return true;
}

bool AP_OABendyRuler::check_collision_with_fences(const Vector2f& start, const Vector2f& end, float& intersection_dist, uint32_t fence_mask[OA_BITMASK_SIZE])
{
    if (!_load_fence_segments()) {
        return false;
    }
    
    memset(fence_mask, 0, OA_BITMASK_SIZE * sizeof(uint32_t));
    
    float closest_dist = FLT_MAX;
    uint16_t closest_segment = 0;
    
    for (uint16_t i = 0; i < _fence_segment_count; i++) {
        const OAFenceSegment& segment = _fence_segments[i];
        Vector2f intersection;
        
        if (_ray_intersects_fence_segment(start, end, segment, intersection)) {
            const float dist = (intersection - start).length();
            if (dist < closest_dist) {
                closest_dist = dist;
                closest_segment = i;
            }
        }
    }
    
    if (closest_dist < FLT_MAX) {
        intersection_dist = closest_dist;
        
        if (closest_segment < OA_MAX_FENCE_SEGMENTS) {
            uint16_t word_index = closest_segment / 32;
            uint16_t bit_index = closest_segment % 32;
            fence_mask[word_index] |= (1U << bit_index);
        }
        
        return true;
    }
    
    return false;
}

bool AP_OABendyRuler::_ray_intersects_fence_segment(const Vector2f& start, const Vector2f& end,
                                                   const OAFenceSegment& segment, Vector2f& intersection) const
{
    const Vector2f p = start;
    const Vector2f r = end - start;
    const Vector2f q = segment.start;
    const Vector2f s = segment.end - segment.start;
    
    const float r_cross_s = r.x * s.y - r.y * s.x;
    
    if (fabsf(r_cross_s) < 1e-6f) {
        return false;
    }
    
    const Vector2f q_minus_p = q - p;
    const float t = (q_minus_p.x * s.y - q_minus_p.y * s.x) / r_cross_s;
    const float u = (q_minus_p.x * r.y - q_minus_p.y * r.x) / r_cross_s;
    
    if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) {
        intersection = p + r * t;
        return true;
    }
    
    return false;
}

#endif // AP_OAPATHPLANNER_BENDYRULER_ENABLED
