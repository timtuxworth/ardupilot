# Plane DAA

This script implements DAA (Detect, Alert, Avoid) for fixed wing and VTOL/quadplanes
using the "Bendy Ruler" algorithm. 

Rather than using the Bendy Ruler implementation for Copter enshrined in the AC_Avoidance library,
this implementation tries to make minimial changes to the c++ core libraries required to surface
the required data to Lua and then allows the Lua to implement most of the Alert and Avoid logic
in order to allow for the maximum implementation flexibility in the face of varying regulatory
environments across the globe.

This script requires the AC_AVOID library to be available. This requires a custom build.

## Parameters

The script adds the following `DAA_` parameters to control its behaviour. It also
reuses the existing core `AVD_` parameters (`AVD_WCLR_XY`, `AVD_WCLR_Z`,
`AVD_NMAC_XY`, `AVD_NMAC_Z`, etc.) that define the Well Clear and Near Mid-Air
Collision volumes, and the `FENCE_*` parameters for the altitude/geo fences.

| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `DAA_ACT_FN` | 308 | | RC option / scripting function used to activate the DAA capability. |
| `DAA_MARGIN_FENCE` | 50 | m | Avoidance margin for the geofence. |
| `DAA_MARGIN_DYN` | 20 | m | Avoidance margin for dynamic objects. |
| `DAA_MARGIN_EXCL` | 20 | m | Avoidance margin for exclusion zones. |
| `DAA_MARGIN_WIDE` | 30 | m | Avoidance margin for wide avoidance. |
| `DAA_MARGIN_HGT` | 60 | m | Avoidance margin for height avoidance. |
| `DAA_LKAHD` | 1000 | m | Avoidance lookahead distance. |
| `DAA_UPDATE_RATE` | 10 | Hz | Rate at which avoidance is processed. |
| `DAA_HEIGHT_USE` | 0 | | Whether to consider height differences when calculating collisions (0: use height, 1: ignore height). |
| `DAA_MARGIN_GA` | 50 | m | Avoidance margin for fixed-wing/General Aviation aircraft, over and above the Well Clear margin `AVD_WCLR_XY`. |
| `DAA_MARGIN_WTH` | 173 | m | Avoidance radius for weather/clouds/rain. |
| `DAA_MARGIN_BIRD` | 100 | m | Avoidance margin for migratory birds. |
| `DAA_MARGIN_PREY` | 200 | m | Avoidance radius for birds of prey. |
| `DAA_MARGIN_UAV` | 50 | m | Avoidance radius for UAVs/drones (MAVLink sourced). |
| `DAA_MARGIN_AIS` | 50 | m | Avoidance radius for AIS ship contacts (MAVLink sourced). |
| `DAA_MARGIN_PRX` | 50 | m | Avoidance radius for obstacles detected by proximity sensors. |
| `DAA_BR_RATIO` | 1.5 | | BendyRuler will avoid changing bearing unless the ratio of the previous margin to the newly calculated margin is at least this much. |
| `DAA_BR_ANGLE` | 45 | deg | BendyRuler resists changing the current bearing if the change exceeds this angle. |
| `DAA_AVD_ALT` | 50 | m | Altitude to loiter/descend to when avoiding a crude aircraft contact. Ignored if zero. |
| `DAA_AVD_ALT_TP` | 3 | | Frame of `DAA_AVD_ALT` (0: absolute, 1: above home, 2: above origin, 3: above terrain). |
| `DAA_AVD_ALERT` | 1 | | Whether to alert on avoidance (0: none, 1: alert). |
| `DAA_AVD_ACTION` | 1 | | Whether to act on avoidance (0: none, 1: avoid). |
| `DAA_MARGIN_ALT` | 20 | m | Proactive buffer inside the safe altitude-fence limits at which DAA starts clamping the commanded altitude. |
| `DAA_ALT_HYST_M` | 10 | m | Hysteresis band for altitude-fence avoidance, preventing chatter as the plane levels off. |
| `DAA_ALT_COOL_S` | 15 | s | Minimum time between altitude-fence "levelling off" notices, so brief re-engagements do not re-spam the GCS. |
| `DAA_HEADING_INC` | 1.5 | deg | Angular step used when searching candidate headings around the target bearing for a collision-free path. The search sweeps a full circle in increments of this size, alternating left and right. Smaller values search more finely but cost more CPU per update. |

## Logging

The script writes the following messages to the dataflash log to record its
DAA (Detect, Alert, Avoid) decisions. All location fields (`TLat`/`TLng`) are in
degrees, altitudes (`TAlt`) in metres, and distances in metres. The altitude
frame field `TFra` is `0` = AMSL, `1` = home-relative, `3` = terrain-relative.

### DAAD — Detect

Written once per avoidance cycle when an obstacle has been detected and a new
target to dodge it has been computed.

| Field | Description |
|-------|-------------|
| `Obs`  | Obstacle found (1/0) |
| `DstF` | Distance to the detected obstacle (m) |
| `DstT` | Distance to the proposed new target that avoids the obstacle (m) |
| `HdgB` | Best bearing found to avoid the obstacle (deg) |
| `Tfnd` | Avoidance target found (1/0) |
| `TLat` | Latitude of the proposed new target (deg) |
| `TLng` | Longitude of the proposed new target (deg) |
| `TAlt` | Altitude of the proposed new target (m) |
| `TFra` | Altitude frame of `TAlt` |
| `ObjT` | `OBSTACLE_TYPE` of the detected object (see table below) |

### DAAV — aVoid

Written when the script commands an avoidance manoeuvre towards a DAA target.

| Field | Description |
|-------|-------------|
| `DstO` | Distance to the obstacle being avoided (m) |
| `TLat` | Latitude of the DAA target (deg) |
| `TLng` | Longitude of the DAA target (deg) |
| `TAlt` | Altitude of the DAA target (m) |
| `TFra` | Altitude frame of `TAlt` |
| `DstH` | Horizontal distance to the obstacle (m) |
| `DstZ` | Vertical distance to the obstacle (+ve is up) (m) |
| `ObjT` | `OBSTACLE_TYPE` of the obstacle (see table below) |

### DAAG — General Aviation aircraft

Written when an aircraft (typically with an ICAO/ADSB identifier) is detected.

| Field | Description |
|-------|-------------|
| `DstF` | Distance to the detected aircraft (m) |
| `TLat` | Latitude of the aircraft (deg) |
| `TLng` | Longitude of the aircraft (deg) |
| `TAlt` | Altitude of the aircraft (m) |
| `TFra` | Altitude frame of `TAlt` |
| `DstH` | Horizontal distance to the aircraft (m) |
| `DstZ` | Vertical distance to the aircraft (+ve is up) (m) |
| `ICAO` | Integer value of the aircraft's ICAO code, if available |

### OBSTACLE_TYPE values

The `ObjT` field uses the following `OBSTACLE_TYPE` enumeration:

| Value | Name | Description |
|-------|------|-------------|
| 0  | GENERAL | Generic obstacle of unknown type |
| 1  | MAV_SYSID | Another MAVLink drone with a MAV_SYSID |
| 2  | GENERAL_AVIATION | Aircraft, usually with an ICAO ADSB identifier |
| 3  | WEATHER | Weather |
| 4  | BIRD_MIGRATORY | Migratory bird(s), e.g. Canada Geese |
| 5  | BIRD_OF_PREY | A bird that might attack the vehicle |
| 6  | FENCE_HOME | All fixed/unmovable fences |
| 7  | FENCE_CIRCLE_INCLUSION | Circular inclusion fence |
| 8  | FENCE_CIRCLE_EXCLUSION | Circular exclusion fence |
| 9  | FENCE_POLYGON_INCLUSION | Polygon inclusion fence |
| 10 | FENCE_POLYGON_EXCLUSION | Polygon exclusion fence |
| 11 | FENCE_LUA | Fence defined in Lua |
| 12 | PROXIMITY | Detected by a proximity sensor, typically close |
| 13 | AIS | AIS-tracked maritime (ship) vehicle |
| 14 | FENCE_ALT_MAX | Max altitude fence (FENCE_TYPE bit 0) |
| 15 | FENCE_ALT_MIN | Min altitude fence (FENCE_TYPE bit 3) |
