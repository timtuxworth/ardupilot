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
