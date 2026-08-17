#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#ifndef AP_ADSB_AVOIDANCE_ENABLED
#define AP_ADSB_AVOIDANCE_ENABLED HAL_ADSB_ENABLED
#endif  // AP_ADSB_AVOIDANCE_ENABLED

// The DAA standoff parameters and the distance queries behind them exist only to
// serve AP_OAScripting, which is itself ArduPlane-only, so keep them off the other
// vehicles and off boards built without scripting.
//
// This lives in a config header, rather than in AP_Avoidance.h, because waf refuses
// vehicle-dependent macros in library headers unless the header is whitelisted in
// Tools/ardupilotwaf/ap_library.py - this file is.  Note that AP_Avoidance.cpp must
// keep spelling APM_BUILD_TYPE out in full: waf decides whether to compile a source
// per-vehicle by searching the .cpp text for that token, and it does not follow
// macros through headers.
#ifndef AP_AVOIDANCE_SCRIPTING_ENABLED
#define AP_AVOIDANCE_SCRIPTING_ENABLED (AP_SCRIPTING_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane))
#endif  // AP_AVOIDANCE_SCRIPTING_ENABLED
