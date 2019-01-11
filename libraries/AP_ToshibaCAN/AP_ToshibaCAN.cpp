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
 * AP_ToshibaCAN.cpp
 *
 *      Author: Francisco Ferreira
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#include <AP_Common/AP_Common.h>

#include <AP_HAL/utility/sparse-endian.h>

#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>

#include "AP_ToshibaCAN.h"

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(_driver_index)) { printf(fmt, ##args); }} while (0)

#define DEFAULT_NUM_POLES 14

#define TOSHIBACAN_OUTPUT_MIN 6300
#define TOSHIBACAN_OUTPUT_MAX 32000

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_ToshibaCAN::var_info[] = {
    // @Param: NPOLE
    // @DisplayName: Number of motor poles
    // @Description: Sets the number of motor poles to calculate the correct RPM value
    AP_GROUPINFO("NPOLE", 1, AP_ToshibaCAN, _num_poles, DEFAULT_NUM_POLES),

    AP_GROUPEND
};

AP_ToshibaCAN::AP_ToshibaCAN()
{
    AP_Param::setup_object_defaults(this, var_info);

    debug_can(2, "ToshibaCAN: constructed\n\r");
}

AP_ToshibaCAN *AP_ToshibaCAN::get_tcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_protocol_type(driver_index) != AP_BoardConfig_CAN::Protocol_Type_ToshibaCAN) {
        return nullptr;
    }
    return static_cast<AP_ToshibaCAN*>(AP::can().get_driver(driver_index));
}

// initialise ToshibaCAN bus
void AP_ToshibaCAN::init(uint8_t driver_index)
{
    _driver_index = driver_index;

    debug_can(2, "ToshibaCAN: starting init\n\r");

    if (_initialized) {
        debug_can(1, "ToshibaCAN: already initialized\n\r");
        return;
    }

    AP_HAL::CANManager* can_mgr = hal.can_mgr[driver_index];

    if (can_mgr == nullptr) {
        debug_can(1, "ToshibaCAN: no mgr for this driver\n\r");
        return;
    }

    if (!can_mgr->is_initialized()) {
        debug_can(1, "ToshibaCAN: mgr not initialized\n\r");
        return;
    }

    _can_driver = can_mgr->get_driver();

    if (_can_driver == nullptr) {
        debug_can(1, "ToshibaCAN: no CAN driver\n\r");
        return;
    }

    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ToshibaCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        debug_can(1, "ToshibaCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(2, "ToshibaCAN: init done\n\r");

    return;
}

// loop to send output to ESCs in background thread
void AP_ToshibaCAN::loop()
{
    uavcan::MonotonicTime timeout;
    uavcan::CanFrame empty_frame { (0 | uavcan::CanFrame::MaskStdID), nullptr, 0 };
    const uavcan::CanFrame* select_frames[uavcan::MaxCanIfaces] { };
    select_frames[CAN_IFACE_INDEX] = &empty_frame;

    const uint32_t loop_interval_us = MIN(AP::scheduler().get_loop_period_us(), SET_PWM_MIN_INTERVAL_US);

    while (true) {
        if (!_initialized) {
            // if not initialised wait 2ms
            debug_can(2, "ToshibaCAN: not initialized\n\r");
            hal.scheduler->delay_microseconds(2000);
            continue;
        }

        if (!_esc_present_bitmask) {
            // if no ESCs wait 1ms and try again
            debug_can(1, "ToshibaCAN: no valid ESC present");
            hal.scheduler->delay(1000);
            continue;
        }

        // prepare message to lock or unlock motors
        motor_lock_cmd_t unlock_cmd = {};
        unlock_cmd.motor1 = (_scaled_output[0] == 0) ? 2 : 1;
        unlock_cmd.motor2 = (_scaled_output[1] == 0) ? 2 : 1;
        unlock_cmd.motor3 = (_scaled_output[2] == 0) ? 2 : 1;
        unlock_cmd.motor4 = (_scaled_output[3] == 0) ? 2 : 1;
        uavcan::CanFrame unlock_frame {(uint8_t)COMMAND_LOCK, unlock_cmd.data, sizeof(unlock_cmd.data)};

        // wait for space in buffer to send
        uint32_t now = AP_HAL::micros64();
        uavcan::CanSelectMasks lock_inout_mask;
        lock_inout_mask.write = 1 << CAN_IFACE_INDEX;
        select_frames[CAN_IFACE_INDEX] = &unlock_frame;
        timeout = uavcan::MonotonicTime::fromUSec(now + loop_interval_us);
        _can_driver->select(lock_inout_mask, select_frames, timeout);

        // double check space to send is available
        if (!lock_inout_mask.write) {
            continue;
        }

        // send unlock command
        now = AP_HAL::micros64();
        timeout = uavcan::MonotonicTime::fromUSec(now + SET_PWM_TIMEOUT_US);
        int8_t res = _can_driver->getIface(CAN_IFACE_INDEX)->send(unlock_frame, timeout, 0);

        // check results
        if (res != 1) {
            // skip updating output to motors
            continue;
        }

        // prepare message to spin motors
        motor_rotation_cmd_t mot_rot_cmd;
        mot_rot_cmd.motor1 = htobe16(_scaled_output[0]);
        mot_rot_cmd.motor2 = htobe16(_scaled_output[1]);
        mot_rot_cmd.motor3 = htobe16(_scaled_output[2]);
        mot_rot_cmd.motor4 = htobe16(_scaled_output[3]);
        uavcan::CanFrame mot_rot_frame {((uint8_t)COMMAND_MOTOR1 & uavcan::CanFrame::MaskStdID), mot_rot_cmd.data, sizeof(mot_rot_cmd.data)};

        // wait for space in buffer to send
        now = AP_HAL::micros64();
        uavcan::CanSelectMasks mot_inout_mask;
        mot_inout_mask.write = 1 << CAN_IFACE_INDEX;
        select_frames[CAN_IFACE_INDEX] = &mot_rot_frame;
        timeout = uavcan::MonotonicTime::fromUSec(now + loop_interval_us);
        _can_driver->select(mot_inout_mask, select_frames, timeout);

        // double check space to send is available
        if (!mot_inout_mask.write) {
            continue;
        }

        // send motor rotation command
        now = AP_HAL::micros64();
        timeout = uavcan::MonotonicTime::fromUSec(now + SET_PWM_TIMEOUT_US);
        res = _can_driver->getIface(CAN_IFACE_INDEX)->send(mot_rot_frame, timeout, 0);

        // check results
        if (res != 1) {
            // ignore failures
        }
    }
}

// called from SRV_Channels
void AP_ToshibaCAN::update()
{
    uint16_t pwm1 = 0;
    uint16_t pwm2 = 0;
    if (_rc_out_sem.take(1)) {
        for (uint8_t i = 0; i < TOSHIBACAN_MAX_NUM_ESCS; i++) {
            SRV_Channel::Aux_servo_function_t motor_function = SRV_Channels::get_motor_function(i);

            uint16_t pwm_out;
            if (SRV_Channels::get_output_pwm(motor_function, pwm_out)) {
                if (i == 0) {
                    pwm1 = pwm_out;
                }
                if (i == 1) {
                    pwm2 = pwm_out;
                }
                if (pwm_out <= 1000) {
                    _scaled_output[i] = 0;
                } else if (pwm_out >= 2000) {
                    _scaled_output[i] = TOSHIBACAN_OUTPUT_MAX;
                } else {
                    _scaled_output[i] = TOSHIBACAN_OUTPUT_MIN + (pwm_out - 1000) * 0.001f * (TOSHIBACAN_OUTPUT_MAX - TOSHIBACAN_OUTPUT_MIN);
                }
            } else {
                _scaled_output[i] = 0;
            }
        }

        _rc_out_sem.give();
        _new_output.store(true, std::memory_order_release);
    } else {
        debug_can(2, "ToshibaCAN: failed to get PWM semaphore on write\n\r");
    }
    static uint16_t counter = 0;
    counter++;
    if (counter > 400) {
        counter = 0;
        gcs().send_text(MAV_SEVERITY_CRITICAL,"1:%u/%u 2:%u/%u",
                        (unsigned)pwm1, (unsigned)_scaled_output[0],
                        (unsigned)pwm2, (unsigned)_scaled_output[1]
                       );
    }
}

#endif // HAL_WITH_UAVCAN
