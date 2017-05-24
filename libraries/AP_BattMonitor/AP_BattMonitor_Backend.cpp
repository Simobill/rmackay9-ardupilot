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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

/*
  base class constructor.
  This incorporates initialisation as well.
*/
AP_BattMonitor_Backend::AP_BattMonitor_Backend(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state) :
        _mon(mon),
        _state(mon_state)
{
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor_Backend::capacity_remaining_pct() const
{
    float mah_remaining = _mon._pack_capacity[_state.instance] - _state.current_total_mah;
    if ( _mon._pack_capacity[_state.instance] > 10 ) { // a very very small battery
        return (100 * (mah_remaining) / _mon._pack_capacity[_state.instance]);
    } else {
        return 0;
    }
}

/// get capacity for this instance
int32_t AP_BattMonitor_Backend::get_capacity() const
{
    return _mon.pack_capacity_mah(_state.instance);
}

// update battery resistance estimate
// faster rates of change of the current and voltage readings cause faster updates to the resistance estimate
// the battery resistance is calculated by comparing the latest current and voltage readings to a low-pass filtered current and voltage
// high current steps are integrated into the resistance estimate by varying the time constant of the resistance filter
void AP_BattMonitor_Backend::update_resistance_estimate()
{
    // return immediately if no current
    if (!has_current() || _state.current_amps <= 0.0f) {
        return;
    }

    // update maximum current seen since startup and protect against divide by zero
    _state.current_max_amps = MAX(_state.current_max_amps, _state.current_amps);
    float current_delta = _state.current_amps - _state.current_filt_amps;
    if (is_zero(_state.current_max_amps) || is_zero(current_delta)) {
        return;
    }

    // calculate time since last update
    uint32_t now = AP_HAL::millis();
    float loop_interval = (now - _state.resistance_timer_ms) / 1000.0f;
    _state.resistance_timer_ms = now;

    // estimate short-term resistance
    float filt_alpha = constrain_float(loop_interval/(loop_interval + AP_BATT_MONITOR_RES_EST_TC_1), 0.0f, 0.5f);
    float resistance_alpha = MIN(1, AP_BATT_MONITOR_RES_EST_TC_2*abs((_state.current_amps-_state.current_filt_amps)/_state.current_max_amps));
    float resistance_estimate = -(_state.voltage-_state.voltage_filt)/current_delta;
    if (resistance_estimate > 0){
        _state.resistance = _state.resistance*(1-resistance_alpha) + resistance_estimate*resistance_alpha;
    }

    // update the filtered voltage and currents
    _state.voltage_filt = _state.voltage_filt*(1-filt_alpha) + _state.voltage*filt_alpha;
    _state.current_filt_amps = _state.current_filt_amps*(1-filt_alpha) + _state.current_amps*filt_alpha;

    // update estimated voltage without sag
    _state.voltage_resting_estimate = _state.voltage + _state.current_amps * _state.resistance;
}
