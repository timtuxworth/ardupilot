#pragma once

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

/*
 * Common Configuration for AC_Avoidance library code, to avoid different configuration for each vehicle
 */

#ifndef AC_AVOID_ENABLED
 #define AC_AVOID_ENABLED   ENABLED
#endif

#ifndef AC_OAPATHPLANNER_ENABLED
 #define AC_OAPATHPLANNER_ENABLED   !HAL_MINIMIZE_FEATURES
#endif
