/*
  SkyDroid gimbal driver using custom serial protocol (usually run over UDP)

  Packet format (courtesy of SkyDroid's "TOP" protocol document).  This is
  the same framing used by SkyDroid's OEM supplier for the Topotek driver
  (see AP_Mount_Topotek) but the address bytes, command identifiers and
  units used by SkyDroid's own firmware differ, so this is a separate,
  independent implementation rather than a subclass.

  -------------------------------------------------------------------------------------------
  Field                 Index   Bytes       Description
  -------------------------------------------------------------------------------------------
  Frame Header          0       3           #TP (fixed length) or #tp (variable length)
  Address Bit           3       2           source address first, destination address second
  Data_Len              5       1           data length (hex nibble, max 0x0F)
  Control Bit           6       1           r -> query   w -> set/control
  Identification Bit    7       3           3 character command identifier
  Data                  10      Data_Len
  Check Bit                     2           sum of all preceding bytes, output as 2 ASCII hex
                                            characters (high nibble first)

  This one driver covers every model in SkyDroid's "TOP protocol" gimbal camera
  family (they all share the same wire protocol; only axis ranges and a few
  optional features differ per model):
  - control is over UDP only (no direct UART), source address is always 'U'
  - axis ranges vary by model, e.g. the C11 has only pitch (-90 to +10 deg) and
    yaw (-90 to +90 deg), no roll axis; the C13 additionally has roll (-45 to
    +45 deg).  Configure MNT1_PITCH/YAW/ROLL_MIN/MAX to match your hardware -
    this driver does not hardcode any model's limits itself
  - SkyDroid's documented sign convention is yaw-right-positive, pitch-up-positive, which
    matches AP_Mount's own convention (no sign flip needed, unlike Topotek's protocol)
  - the connected model (e.g. "C11", "C13") is queried at runtime via the "MOD"
    command and reported through CAMERA_INFORMATION
  - confirmed on real C11 hardware: the combined/absolute-angle commands (GAM, GSM,
    GAY, GAP) are silently ignored - only the individual-axis speed commands (GSY,
    GSP) actually move that model, so both rate and (closed-loop) angle control for
    the C11 are driven through those instead - see uses_individual_axis_speed_commands()
  - confirmed on real C13 hardware: GAM/GSM/GSY/GSP/GAY and PTZ left/right are ALL
    silently ignored (this model does not share the C11's fix) - the only thing that
    moves yaw/roll at all is the "PTZ" command's fine-tune nudge codes (0x10-0x13),
    normally documented as small trim adjustments rather than primary controls, sent
    repeatedly - see uses_finetune_nudge_commands().  Pitch still uses the ordinary
    PTZ up/down jog (0x01/0x02), same as every other model
 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_SKYDROID_ENABLED

#include "AP_Mount_Backend_Serial.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

#define AP_MOUNT_SKYDROID_PACKETLEN_MAX     28      // maximum number of bytes in a packet sent to or received from the gimbal
#define AP_MOUNT_SKYDROID_CMD_CATEGORIES_NUM 5       // number of gimbal message types we parse

class AP_Mount_SkyDroid : public AP_Mount_Backend_Serial
{

public:
    // Constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    // Do not allow copies
    CLASS_NO_COPY(AP_Mount_SkyDroid);

    // update mount position - should be called periodically
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); };

    //
    // camera controls
    //

    // take a picture.  returns true on success
    bool take_picture() override;

    // start or stop video recording
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;

    // set zoom specified as a rate.  SkyDroid's zoom is stepped (not continuous) so
    // each non-zero call sends a single zoom-in/zoom-out pulse
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    bool has_camera_information() const override { return true; }
    // return camera vendor name
    void get_camera_vendor_name(char *buf, uint8_t buflen) const override { strncpy(buf, "SkyDroid", buflen); }
    // return camera model name (e.g. "C11", "C13"), queried from the gimbal via the "MOD" command.
    // this same driver supports every model in SkyDroid's "TOP protocol" gimbal camera family;
    // the model name lets the GCS show which one is actually connected
    void get_camera_model_name(char *buf, uint8_t buflen) const override {
        if (!_got_model_name) {
            return;
        }
        strncpy(buf, _model_name, buflen);
    }
    // return camera firmware version
    uint32_t get_camera_firmware_version() const override { return _firmware_ver; }
    // return camera capability flags
    uint32_t get_camera_cap_flags() const override {
        return (CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM);
    }

    // send camera settings message to GCS
    void send_camera_settings(mavlink_channel_t chan) const override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

    // SkyDroid can send either rates or angles
    uint8_t natively_supported_mount_target_types() const override {
        return NATIVE_ANGLES_AND_RATES_ONLY;
    };

private:

    // header type (fixed or variable length)
    // first three bytes of packet determined by this value
    enum class HeaderType : uint8_t {
        FIXED_LEN = 0x00,       // #TP will be sent
        VARIABLE_LEN = 0x01,    // #tp will be sent
    };

    // address (2nd and 3rd bytes of packet)
    // first byte is always U (external/UDP controller) for our outgoing packets
    enum class AddressByte : uint8_t {
        SYSTEM_AND_IMAGE = 68,      // 'D'
        GIMBAL = 71,                // 'G'
        LENS = 77,                  // 'M'
        UDP = 85,                   // 'U'
    };

    // control byte (read or write)
    // sent as 7th byte of packet
    enum class ControlByte : uint8_t {
        READ = 114,     // 'r'
        WRITE = 119,    // 'w'
    };

    // parsing state
    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER1 = 0,// #
        WAITING_FOR_HEADER2,    // T or t
        WAITING_FOR_HEADER3,    // P or p
        WAITING_FOR_ADDR1,      // normally U
        WAITING_FOR_ADDR2,      // M, D, G
        WAITING_FOR_DATALEN,
        WAITING_FOR_CONTROL,    // r or w
        WAITING_FOR_ID1,        // e.g. 'G'
        WAITING_FOR_ID2,        // e.g. 'A'
        WAITING_FOR_ID3,        // e.g. 'C'
        WAITING_FOR_DATA,       // normally hex numbers in char form (e.g. '0A')
        WAITING_FOR_CRC_LOW,
        WAITING_FOR_CRC_HIGH,
    };

    // identifier bytes
    typedef char Identifier[3];

    // send text prefix string
    static const char* send_message_prefix;

    // reading incoming packets from gimbal and confirm they are of the correct format
    void read_incoming_packets();

    // request gimbal to start sending attitude at 10hz
    void request_gimbal_attitude();

    // request gimbal memory card information
    void request_gimbal_sdcard_info();

    // request gimbal version
    void request_gimbal_version();

    // request gimbal model name (e.g. "C11", "C13")
    void request_gimbal_model();

    // send current UTC date/time to the gimbal (TIM command) so it can correctly
    // timestamp photos/videos - the camera has no RTC of its own and defaults to
    // 1970-01-01 without this (confirmed on real hardware).  Resent periodically,
    // same as request_gimbal_attitude()/send_attitude_enable(), both as a guard
    // against UDP packet loss and to recover if the camera reboots independently
    // of the flight controller (also confirmed to happen on real hardware)
    bool send_time_sync();

    // enable the gimbal to receive our attitude (FAE) and send it to us (GAA)
    bool send_attitude_enable();

    // send our current attitude to the gimbal (FAI)
    bool send_attitude_to_gimbal();

    // send angle target in radians to gimbal
    void send_target_angles(const MountAngleTarget& angle_rad) override;

    // send rate target in rad/s to gimbal
    void send_target_rates(const MountRateTarget& rate_rads) override;

    // true if the connected model is known to only respond to the individual-axis
    // GSY/GSP speed commands (confirmed on the C11; GAM/GSM/GAY/GAP - i.e. every
    // combined or absolute-angle command - are silently ignored on that model).
    // Unknown models (including before the "MOD" response has arrived) default to
    // false and use the normal GAM/GSM angle/rate commands, since that's the
    // documented behaviour and is confirmed working on at least the C10/C10Pro/C12/C20
    // family
    bool uses_individual_axis_speed_commands() const {
        return _got_model_name && strncmp(_model_name, "C11", 3) == 0;
    }

    // send angle target by closing the loop ourselves (P-controller) using GAC
    // attitude feedback and driving GSY/GSP as the rate actuator - there is no
    // absolute-angle command that works on this model
    void send_target_angles_individual_axis(const MountAngleTarget& angle_rad);

    // send rate target directly via GSY/GSP, the only commands confirmed to move
    // this model
    void send_target_rates_individual_axis(const MountRateTarget& rate_rads);

    // send a single-axis rate command (GSY or GSP) for rate_dps, converted to the
    // wire's signed 8bit LSB units using the real-world calibrated scale (see
    // AP_MOUNT_SKYDROID_INDIVIDUAL_AXIS_DPS_PER_LSB).  Caller is responsible for any
    // axis-specific sign compensation (GSY's sign is inverted vs AP_Mount's convention
    // - see send_target_rates_individual_axis())
    void send_individual_axis_rate(const Identifier id, float rate_dps);

    // PTZ "fine tune" nudge codes (data byte of the "PTZ" command, 0x10-0x13).
    // Confirmed on real C13 hardware to be the only commands that move yaw/roll on
    // that model - GAM/GSM/GSY/GSP/GAY/PTZ-left-right are all confirmed silently
    // ignored.  Normally documented as small trim adjustments rather than primary
    // jog/speed controls; a single packet did not produce clearly visible movement
    // in testing, only a rapid burst did (~15 packets at 0.2s intervals) - exact
    // degrees-per-nudge, and whether there is a saturating trim range needing
    // periodic "Clear Fine Tune" (0x14), are NOT YET CHARACTERIZED on real hardware
    enum class FineTuneCode : uint8_t {
        YAW_LEFT = 0x10,
        YAW_RIGHT = 0x11,
        // roll direction convention vs AP_Mount's roll-right-positive is UNCONFIRMED
        // on real hardware - only that these two codes move roll in opposite
        // directions to each other, not which one is "positive" - verify and fix the
        // mapping in send_target_angles_finetune()/send_target_rates_finetune() on
        // the next real hardware round if it turns out backwards
        ROLL_A = 0x12,
        ROLL_B = 0x13,
    };

    // true if the connected model is known to only move yaw/roll via PTZ fine-tune
    // nudges (confirmed on the C13; GAM/GSM/GSY/GSP/GAY/PTZ-left-right are all
    // silently ignored on that model).  Pitch uses the ordinary PTZ up/down jog,
    // same as every other model, so isn't gated by this - see
    // send_target_angles_finetune()/send_target_rates_finetune()
    bool uses_finetune_nudge_commands() const {
        return _got_model_name && strncmp(_model_name, "C13", 3) == 0;
    }

    // send angle target for models needing PTZ fine-tune nudges for yaw/roll (e.g.
    // C13).  Pitch closes the loop with the ordinary PTZ up/down jog, yaw/roll with
    // repeated fine-tune nudges - both duty-cycled (PWM-style) as the error shrinks
    // rather than run at full speed right up to a hard deadzone, to approximate
    // proportional control from an actuator that has none - see duty_from_error()/
    // duty_cycle_active() in the .cpp
    void send_target_angles_finetune(const MountAngleTarget& angle_rad);

    // send rate target for models needing PTZ fine-tune nudges for yaw/roll (e.g.
    // C13) - duty-cycled the same way, from duty_from_rate()
    void send_target_rates_finetune(const MountRateTarget& rate_rads);

    // send a single PTZ pitch jog direction (-1=down, 0=stop, +1=up), deduped
    // against the last direction sent - same "press and hold" mechanism used by
    // every model's pitch control.  Callers duty-cycle by toggling direction
    // on/off between calls (see send_target_angles_finetune()/
    // send_target_rates_finetune()) rather than this function knowing about duty
    // cycling itself
    void send_ptz_pitch_direction(int8_t direction);

    // send a fine-tune nudge code at up to AP_MOUNT_SKYDROID_FINETUNE_NUDGE_INTERVAL_MS
    // (a hard floor - never exceed the empirically-tested rate), further gated by
    // duty (0-1, see duty_cycle_active()) so the effective average nudge rate tapers
    // off as duty shrinks.  Unlike PTZ's jog codes, repeated identical fine-tune
    // sends are NOT deduped - each packet is believed to be a small discrete step,
    // not a "hold to keep moving" command, so sending the same code repeatedly (at
    // whatever the duty-gated rate works out to) is the intended way to keep moving.
    // last_send_ms is the caller's own per-axis timestamp (yaw and roll are
    // throttled independently)
    void send_finetune_nudge(FineTuneCode code, uint32_t &last_send_ms, float duty);

    // attitude information analysis of gimbal (response to GAA, arrives as "GAC")
    void gimbal_angle_analyse();

    // gimbal video information analysis
    void gimbal_record_analyse();

    // information analysis of gimbal storage card
    void gimbal_sdcard_analyse();

    // gimbal basic information analysis
    void gimbal_version_analyse();

    // gimbal model name analysis (raw ASCII text, e.g. "C13")
    void gimbal_model_analyse();

    // calculate checksum
    uint8_t calculate_crc(const uint8_t *cmd, uint8_t len) const;

    // hexadecimal to character conversion
    uint8_t hex2char(uint8_t data) const;

    // send a fixed length packet to gimbal
    // returns true on success, false if serial port initialization failed
    bool send_fixedlen_packet(AddressByte address, const Identifier id, bool write, uint8_t value);

    // send a variable length packet to gimbal
    // returns true on success, false if serial port initialization failed
    bool send_variablelen_packet(HeaderType header, AddressByte address, const Identifier id, bool write, const uint8_t* databuff, uint8_t databuff_len);

    // set gimbal's lock vs follow mode
    // lock should be true if gimbal should maintain an earth-frame target
    // lock is false to follow / maintain a body-frame target
    bool set_gimbal_lock(bool lock);

    // members
    bool _recording;                                            // recording status, tracked locally from commands we've sent
    bool _sdcard_status;                                        // memory card status (received from gimbal)
    bool _last_lock;                                            // last lock mode sent to gimbal
    bool _got_gimbal_version;                                   // true if gimbal's version has been received
    bool _got_model_name;                                       // true if gimbal's model name has been received
    bool _announced_connected;                                  // true once we've told the user the gimbal is connected
    uint32_t _firmware_ver;                                     // firmware version
    char _model_name[8];                                        // gimbal model name (e.g. "C11", "C13"), always null-terminated
    Vector3f _current_angle_rad;                                // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw).  roll is always 0 on models with no roll axis
    uint32_t _last_current_angle_ms;                            // system time (in milliseconds) that angle information received from the gimbal
    uint32_t _last_req_current_info_ms;                         // system time that this driver last requested current gimbal information
    uint32_t _last_model_request_ms;                            // system time this driver last requested the model name (retried fast, independent of the 1hz loop below, until _got_model_name is true)
    uint8_t _last_ptz_pitch_code;                               // last PTZ pitch jog code sent (0=stop, 1=up, 2=down), deduped like every other model's PTZ pitch control.  Zero-initialised like the rest of this class's members, which correctly matches the real "not moving" state before anything has been commanded
    uint32_t _last_yaw_nudge_ms;                                // system time of the last yaw fine-tune nudge sent (models using uses_finetune_nudge_commands() only)
    uint32_t _last_roll_nudge_ms;                               // system time of the last roll fine-tune nudge sent (models using uses_finetune_nudge_commands() only)
    uint8_t _last_req_step;                                     // 10hz request loop step (different requests are sent at various steps)
    uint8_t _msg_buff[AP_MOUNT_SKYDROID_PACKETLEN_MAX];         // buffer holding bytes from latest packet received.  only used to calculate crc
    uint8_t _msg_buff_len;                                      // number of bytes in the msg buffer
    struct {
        ParseState state;                                       // parser state
        uint8_t data_len;                                       // expected number of data bytes
    } _parser;

    // mapping from received message key to member function pointer to consume the message
    typedef struct {
        uint8_t uart_cmd_key[4];                                // gimbal message key
        void (AP_Mount_SkyDroid::*func)(void);                  // member function to consume message
    } UartCmdFunctionHandler;

    // stores command ID and corresponding member functions that are compared with the command received by the gimbal
    UartCmdFunctionHandler uart_recv_cmd_compare_list[AP_MOUNT_SKYDROID_CMD_CATEGORIES_NUM] = {
        {{"GAC"}, &AP_Mount_SkyDroid::gimbal_angle_analyse},
        {{"REC"}, &AP_Mount_SkyDroid::gimbal_record_analyse},
        {{"SDC"}, &AP_Mount_SkyDroid::gimbal_sdcard_analyse},
        {{"VER"}, &AP_Mount_SkyDroid::gimbal_version_analyse},
        {{"MOD"}, &AP_Mount_SkyDroid::gimbal_model_analyse},
    };
};

#endif // HAL_MOUNT_SKYDROID_ENABLED
