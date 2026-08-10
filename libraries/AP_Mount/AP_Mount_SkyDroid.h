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

  Notes specific to the C11:
  - control is over UDP only (no direct UART), source address is always 'U'
  - the C11 has only two controllable axes: pitch (-90 to +10 deg) and yaw (-90 to +90 deg).
    There is no roll axis, so roll is never sent or expected in received packets.
  - SkyDroid's documented sign convention is yaw-right-positive, pitch-up-positive, which
    matches AP_Mount's own convention (no sign flip needed, unlike Topotek's protocol)
 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_SKYDROID_ENABLED

#include "AP_Mount_Backend_Serial.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

#define AP_MOUNT_SKYDROID_PACKETLEN_MAX     28      // maximum number of bytes in a packet sent to or received from the gimbal
#define AP_MOUNT_SKYDROID_CMD_CATEGORIES_NUM 4       // number of gimbal message types we parse

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
    // return camera model name.  hardcoded rather than parsed from the "MOD" command since
    // the protocol doc marks MOD as still under development for this model
    void get_camera_model_name(char *buf, uint8_t buflen) const override { strncpy(buf, "C11", buflen); }
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

    // enable the gimbal to receive our attitude (FAE) and send it to us (GAA)
    bool send_attitude_enable();

    // send our current attitude to the gimbal (FAI)
    bool send_attitude_to_gimbal();

    // send angle target in radians to gimbal
    void send_target_angles(const MountAngleTarget& angle_rad) override;

    // send rate target in rad/s to gimbal
    void send_target_rates(const MountRateTarget& rate_rads) override;

    // attitude information analysis of gimbal (response to GAA, arrives as "GAC")
    void gimbal_angle_analyse();

    // gimbal video information analysis
    void gimbal_record_analyse();

    // information analysis of gimbal storage card
    void gimbal_sdcard_analyse();

    // gimbal basic information analysis
    void gimbal_version_analyse();

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
    uint32_t _firmware_ver;                                     // firmware version
    Vector3f _current_angle_rad;                                // current angles in radians received from gimbal (x=roll (unused), y=pitch, z=yaw)
    uint32_t _last_current_angle_ms;                            // system time (in milliseconds) that angle information received from the gimbal
    uint32_t _last_req_current_info_ms;                         // system time that this driver last requested current gimbal information
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
    };
};

#endif // HAL_MOUNT_SKYDROID_ENABLED
