# motor_recover

Attempts to recover from a VTOL motor failure by switching to forward flight (LOITER mode) if 
a heuristic detects the likely failure of one of the VTOL motors. There are two 
heuristics:-

1. Uncommanded yaw, detected when if
   - if the vehicle is not trying to rotate (yaw_target_rate is small)
   - if the actual yaw is diverging from the target (yaw_error is large)
   - then we accumulate that error in yaw_error_integral
   - if the yaw_error_integral gets too high it assumes motor failure (this is very much a heuristic)

2. Single VTOL motor saturation
If a single pwm output becomes very high (ZMF_PWM_HIGH) while all other motors are 
not saturated, then the script assumes this motor has failed.

The script will NOT switch to forward flight if the vehicle is below Q_ASSIST_ALT.

# Parameters

# AMF_YAW_DEG 

Yaw rate sensitivity (in degrees): If yaw rate exceeded + yaw error exceeded + yaw integral exceeded = motor failure

# AMF_YAW_ERR_DEG

Yaw error sensitivity (in degrees): If yaw rate exceeded + yaw error exceeded + yaw integral exceeded = motor failure

#ZMF_YAW_INT_DEG

Yaw integral sensitivity (in degrees): If yaw rate exceeded + yaw error exceeded + yaw integral exceeded = motor failure

# Operation

Copy this file to the APM/scripts folder on your SD card and make sure 

SCR_ENABLE = 1
SCR_HEAPSIZE = 250000 (minimum)
SCR_VM_. = 200000 (minimum)

The ZMF_* parameters can be changed at runtime to tune the heuristics. It's probably a good idea to test extensively in SITL before trying this on a real vehicle.