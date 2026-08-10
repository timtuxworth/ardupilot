/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Simulator for SkyDroid gimbal.  This one class simulates every model in
  SkyDroid's "TOP protocol" gimbal camera family (they all share the same
  wire protocol); which model is being simulated is selected by the
  constructor arguments, and registered under separate device names (see
  SITL_State_common.cpp) since each has different capabilities

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:skydroid --speedup=1

param set MNT1_TYPE 15       # skydroid
param set SERIAL5_PROTOCOL 8 # gimbal
reboot

To simulate a C13 (which has a roll axis, unlike the C11) use
--serial5=sim:skydroid_c13 instead and set MNT1_ROLL_MIN/MAX to -45/45.

*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_SKYDROID_ENABLED

#include "SIM_Mount.h"
#include "SIM_Gimbal.h"

namespace SITL {

class SkyDroid : public Mount {
public:

    // model_name is returned verbatim in response to the "MOD" command (e.g. "C11",
    // "C13").  has_roll_axis selects whether GAR/GSR (roll angle/rate) commands are
    // handled; models without a roll axis (e.g. the C11) ignore them
    SkyDroid(const char *model_name, bool has_roll_axis) :
        _model_name(model_name), _has_roll_axis(has_roll_axis) {}

    void update(const Aircraft &aircraft) override;

private:

    // the physical gimbal:
    Gimbal gimbal;

    const char *_model_name;
    const bool _has_roll_axis;

    // input accumulation buffer; also used as working buffer by handle_packet()
    static constexpr uint8_t PACKETLEN_MAX = 28;
    uint8_t _buf[PACKETLEN_MAX];
    uint8_t _buflen;

    uint32_t _last_attitude_ms;     // time of last attitude packet sent

    // last commanded angles from GAM/GAR packets (wire centidegrees, same sign as sent
    // by driver i.e. no sign flip: SkyDroid's own protocol is pitch-up-positive same as
    // AP_Mount).  _commanded_roll_cd is only meaningful if _has_roll_axis
    int16_t _commanded_pitch_cd;
    int16_t _commanded_yaw_cd;
    int16_t _commanded_roll_cd;

    // read and dispatch incoming packets from autopilot
    void update_input();

    // scan forward from search_start_pos for '#' and move it to _buf[0]
    void move_preamble_in_buffer(uint8_t search_start_pos);

    // send gimbal attitude packet to the driver
    void send_attitude();

    // dispatch a complete packet beginning at _buf[0], data_len data bytes
    void handle_packet(uint8_t data_len);

    // build and send a response packet.  SkyDroid's control address is always 'U'
    // (UDP-only device, no separate UART/network interface ambiguity)
    void send_packet(char addr2, const char id[3], bool write, const uint8_t *data, uint8_t len);

    // encode a uint16 as 4 uppercase ASCII hex chars
    static void uint16_to_hex4(uint16_t val, uint8_t buf[4]);

    // convert a nibble (0-15) to an uppercase ASCII hex character
    static uint8_t hex2char(uint8_t nibble) {
        return nibble < 10 ? ('0' + nibble) : ('A' + nibble - 10);
    }
};

}  // namespace SITL

#endif  // AP_SIM_SKYDROID_ENABLED
