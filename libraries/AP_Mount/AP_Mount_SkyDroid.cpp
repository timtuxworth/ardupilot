#include "AP_Mount_config.h"

#if HAL_MOUNT_SKYDROID_ENABLED

#include "AP_Mount_SkyDroid.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_SKYDROID_UPDATE_INTERVAL_MS 100                 // resend angle or rate targets, and push our attitude, at this interval
#define AP_MOUNT_SKYDROID_HEALTH_TIMEOUT_MS  1000                // timeout for health (based on attitude reports from gimbal)
#define AP_MOUNT_SKYDROID_PACKETLEN_MIN      12                  // packet length not including the data segment
#define AP_MOUNT_SKYDROID_DATALEN_MAX        (AP_MOUNT_SKYDROID_PACKETLEN_MAX - AP_MOUNT_SKYDROID_PACKETLEN_MIN) // data segment len can be no more than this
#define AP_MOUNT_SKYDROID_ATTITUDE_RATE_HZ   10                  // rate we ask the gimbal to stream its attitude to us

// 3 character identifiers
#define AP_MOUNT_SKYDROID_ID3CHAR_GIMBAL_MODE       "PTZ"        // discrete gimbal control, data bytes: 00:stop, 01:up, 02:down, 03:left, 04:right, 05:center, 06:follow, 07:lock head
#define AP_MOUNT_SKYDROID_ID3CHAR_SPEED_YAW_PITCH   "GSM"        // rate control, data bytes: yaw speed then pitch speed, each signed 8bit hex, units of 0.5deg/s
#define AP_MOUNT_SKYDROID_ID3CHAR_ANGLE_YAW_PITCH   "GAM"        // angle control, data bytes: yaw angle(4hex,0.01deg)+yaw speed(2hex)+pitch angle(4hex,0.01deg)+pitch speed(2hex)
#define AP_MOUNT_SKYDROID_ID3CHAR_ATTITUDE_ENABLE   "GAA"        // enable/disable gimbal->us attitude streaming, data bytes: 00:off, 01-64:rate in Hz
#define AP_MOUNT_SKYDROID_ID3CHAR_ATTITUDE_DATA     "GAC"        // unsolicited attitude data from gimbal, data bytes: yaw+pitch+roll, each 4hex 0.01deg
#define AP_MOUNT_SKYDROID_ID3CHAR_FC_ATTITUDE_ENABLE "FAE"       // enable/disable us->gimbal attitude streaming, data bytes: 00:off, 01:on
#define AP_MOUNT_SKYDROID_ID3CHAR_FC_ATTITUDE_DATA   "FAI"       // our attitude sent to gimbal, data bytes: yaw+pitch+roll (4hex each, 0.01deg) + mode (1:fixed-wing, 0:hover)
#define AP_MOUNT_SKYDROID_ID3CHAR_RECORD_VIDEO      "REC"        // record video, data bytes: 00:stop, 01:start
#define AP_MOUNT_SKYDROID_ID3CHAR_CAPTURE           "CAP"        // take picture, data bytes: 01
#define AP_MOUNT_SKYDROID_ID3CHAR_SD_CARD           "SDC"        // get SD card state, data bytes: 00 to query
#define AP_MOUNT_SKYDROID_ID3CHAR_GET_VERSION       "VER"        // get firmware version, data bytes always 00
#define AP_MOUNT_SKYDROID_ID3CHAR_DIGITAL_ZOOM      "DZM"        // digital zoom, data bytes: 0A:zoom+ (single step), 0B:zoom- (single step)

#define AP_MOUNT_SKYDROID_DEBUG 0
#define debug(fmt, args ...) do { if (AP_MOUNT_SKYDROID_DEBUG) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SkyDroid: " fmt, ## args); } } while (0)

const char* AP_Mount_SkyDroid::send_message_prefix = "Mount: SkyDroid";

// update mount position - should be called periodically
void AP_Mount_SkyDroid::update()
{
    AP_Mount_Backend::update();

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // reading incoming packets from gimbal
    read_incoming_packets();

    // everything below updates at 10hz
    uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _last_req_current_info_ms) < AP_MOUNT_SKYDROID_UPDATE_INTERVAL_MS) {
        return;
    }
    _last_req_current_info_ms = now_ms;

    // push our own attitude to the gimbal
    send_attitude_to_gimbal();

    // calls below here called at 1hz
    _last_req_step++;
    if (_last_req_step >= 10) {
        _last_req_step = 0;
    }
    switch (_last_req_step) {
    case 0:
        // get gimbal version
        if (!_got_gimbal_version) {
            request_gimbal_version();
        }
        break;
    case 2:
        // (re)request gimbal attitude streaming.  harmless to resend if already enabled,
        // and guards against the enable packet being lost over UDP
        request_gimbal_attitude();
        break;
    case 4:
        // request memory card information
        request_gimbal_sdcard_info();
        break;
    case 6:
        // (re)enable gimbal to accept our attitude pushes
        send_attitude_enable();
        break;
    }

    // update based on mount mode
    update_mnt_target();

    // send target angles or rates depending on the target type
    send_target_to_gimbal();
}

// return true if healthy
bool AP_Mount_SkyDroid::healthy() const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // unhealthy if attitude information not received recently
    const uint32_t last_current_angle_ms = _last_current_angle_ms;
    return (AP_HAL::millis() - last_current_angle_ms < AP_MOUNT_SKYDROID_HEALTH_TIMEOUT_MS);
}

// take a picture.  returns true on success
bool AP_Mount_SkyDroid::take_picture()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // exit immediately if the memory card is abnormal
    if (!_sdcard_status) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s SD card error", send_message_prefix);
        return false;
    }

    // sample command: #TPUD2wCAP01
    return send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_SKYDROID_ID3CHAR_CAPTURE, true, 1);
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount_SkyDroid::record_video(bool start_recording)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // exit immediately if the memory card is abnormal
    if (!_sdcard_status) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s SD card error", send_message_prefix);
        return false;
    }

    // sample command: #TPUD2wREC01
    if (send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_SKYDROID_ID3CHAR_RECORD_VIDEO, true, start_recording ? 1 : 0)) {
        // SkyDroid does not push unsolicited recording-state changes to us so track our own request locally
        _recording = start_recording;
        return true;
    }
    return false;
}

// set zoom specified as a rate.  SkyDroid's digital zoom is stepped, not continuous:
// there is no "stop" data value, only single-shot zoom-in/zoom-out pulses
bool AP_Mount_SkyDroid::set_zoom(ZoomType zoom_type, float zoom_value)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // only rate based zoom is supported
    if (zoom_type != ZoomType::RATE) {
        return false;
    }

    // zero rate has no corresponding command so treat as a successful no-op
    if (is_zero(zoom_value)) {
        return true;
    }

    // sample command: #TPUM2wDZM0A65
    const uint8_t zoom_cmd = (zoom_value < 0) ? 0x0B : 0x0A;  // 0x0B: zoom-, 0x0A: zoom+
    return send_fixedlen_packet(AddressByte::LENS, AP_MOUNT_SKYDROID_ID3CHAR_DIGITAL_ZOOM, true, zoom_cmd);
}

// send camera settings message to GCS
void AP_Mount_SkyDroid::send_camera_settings(mavlink_channel_t chan) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // send CAMERA_SETTINGS message
    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),   // time_boot_ms
        _recording ? CAMERA_MODE_VIDEO : CAMERA_MODE_IMAGE, // camera mode (0:image, 1:video, 2:image survey)
        NaNf,               // zoomLevel float, percentage from 0 to 100, NaN if unknown
        NaNf);              // focusLevel float, percentage from 0 to 100, NaN if unknown
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_SkyDroid::get_attitude_quaternion(Quaternion& att_quat)
{
    // x=roll (always zero, C11 has no roll axis), y=pitch, z=yaw
    att_quat.from_euler(_current_angle_rad.x, _current_angle_rad.y, _current_angle_rad.z);
    return true;
}

// reading incoming packets from gimbal and confirm they are of the correct format
void AP_Mount_SkyDroid::read_incoming_packets()
{
    // check for bytes on the serial port
    int16_t nbytes = MIN(_uart->available(), 1024U);
    if (nbytes <= 0) {
        return;
    }

    // flag to allow cases below to reset parser state
    bool reset_parser = false;

    // process bytes received
    for (int16_t i = 0; i < nbytes; i++) {
        uint8_t b;
        if (!_uart->read(b)) {
            continue;
        }

        // add latest byte to buffer
        _msg_buff[_msg_buff_len++] = b;

        // protect against overly long messages
        if (_msg_buff_len >= AP_MOUNT_SKYDROID_PACKETLEN_MAX) {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (_parser.state) {

        case ParseState::WAITING_FOR_HEADER1:
            if (b == '#') {
                _parser.state = ParseState::WAITING_FOR_HEADER2;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_HEADER2:
            if (b == 't' || b == 'T') {
                _parser.state = ParseState::WAITING_FOR_HEADER3;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_HEADER3:
            if (b == 'p' || b == 'P') {
                _parser.state = ParseState::WAITING_FOR_ADDR1;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_ADDR1:
        case ParseState::WAITING_FOR_ADDR2:
            if (b == 'U' || b == 'M' || b == 'D' || b == 'G') {
                // advance to next state
                _parser.state = (ParseState)((uint8_t)_parser.state+1);
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_DATALEN: {
            // sanity check data length
            uint8_t data_len;
            if (hex_char_to_nibble(b, data_len) && data_len <= AP_MOUNT_SKYDROID_DATALEN_MAX) {
                _parser.data_len = data_len;
                _parser.state = ParseState::WAITING_FOR_CONTROL;
                break;
            }
            reset_parser = true;
            break;
        }

        case ParseState::WAITING_FOR_CONTROL:
            // r or w
            if (b == 'r' || b == 'w') {
                _parser.state = ParseState::WAITING_FOR_ID1;
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_ID1:
        case ParseState::WAITING_FOR_ID2:
        case ParseState::WAITING_FOR_ID3:
            // check all uppercase letters and numbers.  eg 'GAC'
            if ((b >= 'A' && b <= 'Z') || (b >= '0' && b <= '9')) {
                // advance to next state
                _parser.state = (ParseState)((uint8_t)_parser.state+1);
                break;
            }
            reset_parser = true;
            break;

        case ParseState::WAITING_FOR_DATA: {
            // normally hex numbers in char form (e.g. '0A')
            const uint8_t data_bytes_received = _msg_buff_len - (AP_MOUNT_SKYDROID_PACKETLEN_MIN - 2);

            // sanity check to protect against programming errors
            if (data_bytes_received > AP_MOUNT_SKYDROID_DATALEN_MAX) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                reset_parser = true;
                break;
            }

            // advance parser state once expected number of bytes have been received
            if (data_bytes_received == _parser.data_len) {
                _parser.state = ParseState::WAITING_FOR_CRC_LOW;
            }
            break;
        }

        case ParseState::WAITING_FOR_CRC_LOW:
            _parser.state = ParseState::WAITING_FOR_CRC_HIGH;
            break;

        case ParseState::WAITING_FOR_CRC_HIGH:
            // this is the last byte in the message so reset the parser
            reset_parser = true;

            // sanity check to protect against programming errors
            if (_msg_buff_len < AP_MOUNT_SKYDROID_PACKETLEN_MIN) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                break;
            }

            // calculate and check CRC
            const uint8_t crc_value = calculate_crc(_msg_buff, _msg_buff_len - 2);
            const char crc_char1 = hex2char((crc_value >> 4) & 0x0f);
            const char crc_char2 = hex2char((crc_value) & 0x0f);
            if (crc_char1 != _msg_buff[_msg_buff_len - 2] || crc_char2 != _msg_buff[_msg_buff_len-1]) {
                debug("CRC expected:%x got:%c%c", (int)crc_value, crc_char1, crc_char2);
                break;
            }

            // CRC is OK, call function to process the message
            for (uint8_t count = 0; count < AP_MOUNT_SKYDROID_CMD_CATEGORIES_NUM; count++) {
                if (strncmp((const char*)_msg_buff + 7, (const char*)(uart_recv_cmd_compare_list[count].uart_cmd_key), 3) == 0) {
                    (this->*(uart_recv_cmd_compare_list[count].func))();
                    break;
                }
            }
        }

        // handle reset of parser
        if (reset_parser) {
            _parser.state = ParseState::WAITING_FOR_HEADER1;
            _msg_buff_len = 0;
            reset_parser = false;
        }
    }
}

// request gimbal to (re)start sending us attitude at 10hz
void AP_Mount_SkyDroid::request_gimbal_attitude()
{
    // sample command: #TPUG2wGAA0A
    send_fixedlen_packet(AddressByte::GIMBAL, AP_MOUNT_SKYDROID_ID3CHAR_ATTITUDE_ENABLE, true, AP_MOUNT_SKYDROID_ATTITUDE_RATE_HZ);
}

// request gimbal memory card information
void AP_Mount_SkyDroid::request_gimbal_sdcard_info()
{
    // sample command: #TPUD2rSDC00
    send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_SKYDROID_ID3CHAR_SD_CARD, false, 0);
}

// request gimbal version
void AP_Mount_SkyDroid::request_gimbal_version()
{
    // sample command: #TPUD2rVER00
    send_fixedlen_packet(AddressByte::SYSTEM_AND_IMAGE, AP_MOUNT_SKYDROID_ID3CHAR_GET_VERSION, false, 0);
}

// (re)enable the gimbal to accept our attitude pushes
bool AP_Mount_SkyDroid::send_attitude_enable()
{
    // sample command: #TPUG2wFAE01
    return send_fixedlen_packet(AddressByte::GIMBAL, AP_MOUNT_SKYDROID_ID3CHAR_FC_ATTITUDE_ENABLE, true, 1);
}

// send our current attitude to the gimbal
bool AP_Mount_SkyDroid::send_attitude_to_gimbal()
{
    const int16_t yaw_cd = wrap_180_cd((int32_t)(AP::ahrs().get_yaw_deg() * 100));
    const int16_t pitch_cd = (int16_t)(AP::ahrs().get_pitch_deg() * 100);
    const int16_t roll_cd = (int16_t)(AP::ahrs().get_roll_deg() * 100);

    // 1: fixed-wing, 0: copter/hover.  fixed at compile time because a single firmware
    // build only ever targets one vehicle type
    const bool fixed_wing = APM_BUILD_TYPE(APM_BUILD_ArduPlane);

    // sample command: #tpUG0EwFAI
    uint8_t databuff[15];
    hal.util->snprintf((char*)databuff, ARRAY_SIZE(databuff), "%04X%04X%04X%02X",
                        (uint16_t)yaw_cd, (uint16_t)pitch_cd, (uint16_t)roll_cd, fixed_wing ? 1 : 0);
    return send_variablelen_packet(HeaderType::VARIABLE_LEN, AddressByte::GIMBAL, AP_MOUNT_SKYDROID_ID3CHAR_FC_ATTITUDE_DATA, true, databuff, ARRAY_SIZE(databuff)-1);
}

// send angle target in radians to gimbal
void AP_Mount_SkyDroid::send_target_angles(const MountAngleTarget& angle_rad)
{
    // set gimbal's lock state (follow the body-frame target)
    if (!set_gimbal_lock(false)) {
        return;
    }

    // clamp to the configured MNT1_YAW/PITCH_MIN/MAX range.  Note: the C11's real
    // hardware limit is -90..+10 deg pitch, -90..+90 deg yaw - set MNT1_PITCH_MAX=10
    // (and leave yaw at the -90/+90 default) to match the physical hardware; this
    // driver does not hardcode that limit itself so it stays correct if a future
    // firmware/model widens the range
    const int16_t yaw_cd = constrain_int16(degrees(angle_rad.get_bf_yaw()) * 100,
                                            _params.yaw_angle_min * 100,
                                            _params.yaw_angle_max * 100);
    const int16_t pitch_cd = constrain_int16(degrees(angle_rad.pitch) * 100,
                                              _params.pitch_angle_min * 100,
                                              _params.pitch_angle_max * 100);

    // sample command: #TPUGCwGAM
    const uint8_t speed = 99;  // GAM's documented max speed sub-field value (0.5deg/s units)
    uint8_t databuff[13];
    hal.util->snprintf((char*)databuff, ARRAY_SIZE(databuff), "%04X%02X%04X%02X",
                        (uint16_t)yaw_cd, speed, (uint16_t)pitch_cd, speed);
    send_variablelen_packet(HeaderType::VARIABLE_LEN, AddressByte::GIMBAL, AP_MOUNT_SKYDROID_ID3CHAR_ANGLE_YAW_PITCH, true, databuff, ARRAY_SIZE(databuff)-1);
}

// send rate target in rad/s to gimbal
void AP_Mount_SkyDroid::send_target_rates(const MountRateTarget& rate_rads)
{
    // set gimbal's lock state if it has changed
    if (!set_gimbal_lock(rate_rads.yaw_is_ef)) {
        return;
    }

    // convert rad/s to SkyDroid's signed 8-bit units of 0.5deg/s
    const int8_t yaw_speed = constrain_int16(degrees(rate_rads.yaw) * 2, -127, 127);
    const int8_t pitch_speed = constrain_int16(degrees(rate_rads.pitch) * 2, -127, 127);

    // sample command: #TPUG4wGSM
    uint8_t databuff[5];
    hal.util->snprintf((char*)databuff, ARRAY_SIZE(databuff), "%02X%02X", (uint8_t)yaw_speed, (uint8_t)pitch_speed);
    send_variablelen_packet(HeaderType::VARIABLE_LEN, AddressByte::GIMBAL, AP_MOUNT_SKYDROID_ID3CHAR_SPEED_YAW_PITCH, true, databuff, ARRAY_SIZE(databuff)-1);
}

// attitude information analysis of gimbal (arrives as "GAC" in response to our "GAA" enable request)
void AP_Mount_SkyDroid::gimbal_angle_analyse()
{
    // consume current angles.  data is yaw, pitch, roll in that order, each 4 hex chars, 0.01deg units
    uint32_t yaw_raw, pitch_raw, roll_raw;
    if (!hex_chars_to_uint32((const char*)&_msg_buff[10], 4, yaw_raw) ||
        !hex_chars_to_uint32((const char*)&_msg_buff[14], 4, pitch_raw) ||
        !hex_chars_to_uint32((const char*)&_msg_buff[18], 4, roll_raw)) {
        return;
    }
    const int16_t yaw_angle_cd = wrap_180_cd((int16_t)yaw_raw);
    const int16_t pitch_angle_cd = (int16_t)pitch_raw;
    const int16_t roll_angle_cd = (int16_t)roll_raw;    // always 0 on C11, no roll axis

    // convert cd to radians
    _current_angle_rad.x = cd_to_rad(roll_angle_cd);
    _current_angle_rad.y = cd_to_rad(pitch_angle_cd);
    _current_angle_rad.z = cd_to_rad(yaw_angle_cd);
    _last_current_angle_ms = AP_HAL::millis();
}

// gimbal video information analysis
void AP_Mount_SkyDroid::gimbal_record_analyse()
{
    _recording = (_msg_buff[11] == '1');
}

// information analysis of gimbal storage card
void AP_Mount_SkyDroid::gimbal_sdcard_analyse()
{
    // data is 10 hex chars: 5 for remaining capacity, 5 for total capacity (units MB)
    // all zeros means no card inserted
    bool all_zero = true;
    for (uint8_t i = 0; i < 10 && (10U + i) < _msg_buff_len; i++) {
        if (_msg_buff[10 + i] != '0') {
            all_zero = false;
            break;
        }
    }
    _sdcard_status = !all_zero;
}

// gimbal basic information analysis.  response data is of the form "VX.X.X" (e.g. "V1.0.78")
void AP_Mount_SkyDroid::gimbal_version_analyse()
{
    uint8_t data_buf_len;
    if (!hex_char_to_nibble(_msg_buff[5], data_buf_len) || data_buf_len == 0 || _msg_buff[10] != 'V') {
        return;
    }

    // version array with index 0=major, 1=minor, 2=patch
    uint8_t version[3] {};
    uint8_t ver_count = 0;
    uint32_t ver_num = 0;
    for (uint8_t i = 1; i < data_buf_len && ver_count < ARRAY_SIZE(version); i++) {
        const uint8_t c = _msg_buff[10 + i];
        if (c == '.') {
            version[ver_count++] = ver_num;
            ver_num = 0;
            continue;
        }
        uint8_t digit;
        if (!hex_char_to_nibble(c, digit)) {
            return;
        }
        ver_num = ver_num * 10 + digit;
    }
    if (ver_count < ARRAY_SIZE(version)) {
        version[ver_count] = ver_num;
    }
    _firmware_ver = (version[2] << 16) | (version[1] << 8) | (version[0]);

    // display gimbal firmware version to user
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s v%u.%u.%u",
        send_message_prefix,
        version[0],     // major version
        version[1],     // minor version
        version[2]);    // patch version

    _got_gimbal_version = true;
}

// calculate checksum
uint8_t AP_Mount_SkyDroid::calculate_crc(const uint8_t *cmd, uint8_t len) const
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i<len; i++) {
        crc += cmd[i];
    }
    return(crc);
}

// hexadecimal to character conversion
uint8_t AP_Mount_SkyDroid::hex2char(uint8_t data) const
{
    if ((9 >= data)) {
        return (data + '0');
    } else {
        return (data - 10 + 'A');
    }
}

// send a fixed length packet
bool AP_Mount_SkyDroid::send_fixedlen_packet(AddressByte address, const Identifier id, bool write, uint8_t value)
{
    uint8_t databuff[3];
    hal.util->snprintf((char *)databuff, ARRAY_SIZE(databuff), "%02X", value);
    return send_variablelen_packet(HeaderType::FIXED_LEN, address, id, write, databuff, ARRAY_SIZE(databuff)-1);
}

// send variable length packet
bool AP_Mount_SkyDroid::send_variablelen_packet(HeaderType header, AddressByte address, const Identifier id, bool write, const uint8_t* databuff, uint8_t databuff_len)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return false;
    }

    // calculate and sanity check packet size
    const uint16_t packet_size = AP_MOUNT_SKYDROID_PACKETLEN_MIN + databuff_len;
    if (packet_size > AP_MOUNT_SKYDROID_PACKETLEN_MAX) {
        debug("send_packet data buff too large");
        return false;
    }

    // check for sufficient space in outgoing buffer
    if (_uart->txspace() < packet_size) {
        debug("tx buffer full");
        return false;
    }

    // create buffer for holding outgoing packet
    uint8_t send_buff[packet_size];
    uint8_t send_buff_ofs = 0;

    // packet header (bytes 0 ~ 2)
    send_buff[send_buff_ofs++] = '#';
    send_buff[send_buff_ofs++] = (header == HeaderType::FIXED_LEN) ? 'T' : 't';
    send_buff[send_buff_ofs++] = (header == HeaderType::FIXED_LEN) ? 'P' : 'p';

    // address (bytes 3, 4).  source is always the UDP/external-control address for this gimbal
    send_buff[send_buff_ofs++] = (uint8_t)AddressByte::UDP;
    send_buff[send_buff_ofs++] = (uint8_t)address;

    // data length (byte 5)
    send_buff[send_buff_ofs++] = hex2char(databuff_len);

    // control byte (byte 6)
    send_buff[send_buff_ofs++] = write ? (uint8_t)ControlByte::WRITE : (uint8_t)ControlByte::READ;

    // identifier (bytes 7 ~ 9)
    send_buff[send_buff_ofs++] = id[0];
    send_buff[send_buff_ofs++] = id[1];
    send_buff[send_buff_ofs++] = id[2];

    // data
    if (databuff_len != 0) {
        memcpy(&send_buff[send_buff_ofs], databuff, databuff_len);
        send_buff_ofs += databuff_len;
    }

    // crc
    uint8_t crc = calculate_crc(send_buff, send_buff_ofs);
    send_buff[send_buff_ofs++] = hex2char((crc >> 4) & 0x0f);
    send_buff[send_buff_ofs++] = hex2char(crc & 0x0f);

    // send packet
    _uart->write(send_buff, send_buff_ofs);
    return true;
}

// set gimbal's lock vs follow mode
// lock should be true if gimbal should maintain an earth-frame target
// lock is false to follow / maintain a body-frame target
bool AP_Mount_SkyDroid::set_gimbal_lock(bool lock)
{
    if (_last_lock == lock) {
        return true;
    }

    // send message and update lock state.  PTZ data: 0x06 = follow, 0x07 = lock head
    if (send_fixedlen_packet(AddressByte::GIMBAL, AP_MOUNT_SKYDROID_ID3CHAR_GIMBAL_MODE, true, lock ? 0x07 : 0x06)) {
        _last_lock = lock;
        return true;
    }
    return false;
}

#endif // HAL_MOUNT_SKYDROID_ENABLED
