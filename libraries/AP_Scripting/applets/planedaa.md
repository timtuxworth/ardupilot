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

The script adds the following parameters to control it's behaviour. It uses
the existing AVD parameters that are used for the Copter FOLLOW mode. In addition
the following "FOLLP" parameters are added.

## FOLLP_FAIL_MODE

This is the mode the plane will change to if following fails. Failure happens
if the following plane loses telemetry from the target, or the distance exceeds
FOLL_DIST_MAX.

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
