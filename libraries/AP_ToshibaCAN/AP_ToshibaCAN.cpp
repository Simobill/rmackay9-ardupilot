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

    static uint8_t stage = 0;  // 0 = locking, 1 = unlocking, 2 = sending motor updates

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

        // lock motors
        if (stage == 0) {
            // prepare message to LOCK motors
            motor_lock_cmd_t lock_cmd = {{2,2,2,2,2,2,2,2,2,2,2,2}};
            uavcan::CanFrame lock_frame {((uint8_t)COMMAND_LOCK | uavcan::CanFrame::MaskStdID), lock_cmd.data, ARRAY_SIZE(lock_cmd.data)};

            // wait for space in buffer to send
            uint64_t now = AP_HAL::micros64();
            uavcan::CanSelectMasks inout_mask;
            inout_mask.write = 1 << CAN_IFACE_INDEX;
            select_frames[CAN_IFACE_INDEX] = &lock_frame;
            timeout = uavcan::MonotonicTime::fromUSec(now + loop_interval_us);
            _can_driver->select(inout_mask, select_frames, timeout);

            // double check space to send is available
            if (!inout_mask.write) {
                continue;
            }

            // send lock command
            now = AP_HAL::micros64();
            timeout = uavcan::MonotonicTime::fromUSec(now + SET_PWM_TIMEOUT_US);
            int8_t res = _can_driver->getIface(CAN_IFACE_INDEX)->send(lock_frame, timeout, 0);

            // check results
            if (res == 1) {
                // success
                gcs().send_text(MAV_SEVERITY_CRITICAL,"lok!");
                // move to next stage
                stage = 1;
            } else if (res == 0) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"f1!");
                continue;
            } else {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"f1!");
                continue;
            }
        }

        // unlock motors
        if (stage == 1) {
            // prepare message to unlock motors
            motor_lock_cmd_t unlock_cmd = {{1,1,1,1,1,1,1,1,1,1,1,1}};
            uavcan::CanFrame unlock_frame { ((uint8_t)COMMAND_LOCK | uavcan::CanFrame::MaskStdID), unlock_cmd.data, ARRAY_SIZE(unlock_cmd.data) };

            // wait for space in buffer to send
            uint32_t now = AP_HAL::micros64();
            uavcan::CanSelectMasks inout_mask;
            inout_mask.write = 1 << CAN_IFACE_INDEX;
            select_frames[CAN_IFACE_INDEX] = &unlock_frame;
            timeout = uavcan::MonotonicTime::fromUSec(now + loop_interval_us);
            _can_driver->select(inout_mask, select_frames, timeout);

            // double check space to send is available
            if (!inout_mask.write) {
                continue;
            }

            // send unlock command
            now = AP_HAL::micros64();
            timeout = uavcan::MonotonicTime::fromUSec(now + SET_PWM_TIMEOUT_US);
            int8_t res = _can_driver->getIface(CAN_IFACE_INDEX)->send(unlock_frame, timeout, 0);

            // check results
            if (res == 1) {
                // success
                gcs().send_text(MAV_SEVERITY_CRITICAL,"ulok!");
                // move to next stage
                stage = 2;
            } else if (res == 0) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"f2!");
                continue;
            } else {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"f3!");
                continue;
            }
        }

        // send motor commands
        if (stage == 2) {
            // prepare message to spin motors
            motor_rotation_cmd_t mot_rot_cmd; // = {{.motor1 = 6300, .motor2 = 6300, .motor3 = 6300, .motor4 = 6300}};
            mot_rot_cmd.motor1 = htobe16((uint16_t)6300);
            mot_rot_cmd.motor2 = htobe16((uint16_t)6300);
            mot_rot_cmd.motor3 = htobe16((uint16_t)6300);
            mot_rot_cmd.motor4 = htobe16((uint16_t)6300);

            uavcan::CanFrame mot_rot_frame {((uint8_t)COMMAND_MOTOR1 | uavcan::CanFrame::MaskStdID), mot_rot_cmd.data, ARRAY_SIZE(mot_rot_cmd.data)};

            // wait for space in buffer to send
            uint32_t now = AP_HAL::micros64();
            uavcan::CanSelectMasks inout_mask;
            inout_mask.write = 1 << CAN_IFACE_INDEX;
            select_frames[CAN_IFACE_INDEX] = &mot_rot_frame;
            timeout = uavcan::MonotonicTime::fromUSec(now + loop_interval_us);
            _can_driver->select(inout_mask, select_frames, timeout);

            // double check space to send is available
            if (!inout_mask.write) {
                continue;
            }

            // send motor rotation command
            now = AP_HAL::micros64();
            timeout = uavcan::MonotonicTime::fromUSec(now + SET_PWM_TIMEOUT_US);
            int8_t res = _can_driver->getIface(CAN_IFACE_INDEX)->send(mot_rot_frame, timeout, 0);

            // check results
            if (res == 1) {
                // success
                gcs().send_text(MAV_SEVERITY_CRITICAL,"mok!");
            } else if (res == 0) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"m1!");
                continue;
            } else {
                gcs().send_text(MAV_SEVERITY_CRITICAL,"m1!");
                continue;
            }
        }

        /*id.id2 = COMMAND_MOTOR1;

        frame = { (id.value | uavcan::CanFrame::MaskStdID), mot_rot_data.data, ARRAY_SIZE(mot_rot_data.data) };
        select_frames[CAN_IFACE_INDEX] = &frame;
        _can_driver->select(inout_mask, select_frames, timeout);
        */

        /*
        frame_id_t id = { { .object_address = ENUM_OBJ_ADDR,
                          .destination_id = BROADCAST_NODE_ID,
                          .source_id = AUTOPILOT_NODE_ID,
                          .priority = 0,
                          .unused = 0 } };
        be16_t data = htobe16((uint16_t) ENUMERATION_TIMEOUT_MS);
        uavcan::CanFrame frame { (id.value | uavcan::CanFrame::FlagEFF), (uint8_t*) &data, sizeof(data) };

        uavcan::CanSelectMasks in_mask = inout_mask;
        select_frames[CAN_IFACE_INDEX] = &frame;

        _can_driver->select(inout_mask, select_frames, timeout);
        */

/*
        return;

        //uavcan::CanSelectMasks in_mask = inout_mask;
        _can_driver->select(inout_mask, select_frames, timeout);

        //return;

        if (in_mask.write & inout_mask.write) {
            bool new_output = _new_output.load(std::memory_order_acquire);

            // check for timeout sending to ESC
            now = AP_HAL::micros64();
            if (sending_esc_num > 0) {
                if ((now - pwm_last_sent) > SET_PWM_TIMEOUT_US) {
                    debug_can(2, "ToshibaCAN: timed-out after sending frame to ESC with ID %d\n\r", sending_esc_num - 1);
                    sending_esc_num = 0;
                }
            }

            // copy desired pwm outputs to output_buffer
            if (sending_esc_num == 0 && new_output) {
                if (!_rc_out_sem.take(1)) {
                    debug_can(2, "ToshibaCAN: failed to get PWM semaphore on read\n\r");
                    continue;
                }
                memcpy(output_buffer, _scaled_output, TOSHIBACAN_MAX_NUM_ESCS * sizeof(uint16_t));
                _rc_out_sem.give();
            }

            if ((sending_esc_num > 0) ||
                (new_output && (pwm_last_sent == 0 || (now - pwm_last_sent > SET_PWM_TIMEOUT_US))) ||
                ((pwm_last_sent != 0) && (now - pwm_last_sent > SET_PWM_MIN_INTERVAL_US))) {

                for (uint8_t esc_num = sending_esc_num; esc_num < _esc_max_node_id; esc_num++) {

                    if ((_esc_present_bitmask & (1 << esc_num)) == 0) {
                        continue;
                    }

                    be16_t tcan_pwm = htobe16(output_buffer[esc_num]);

                    frame_id_t id = { { .object_address = SET_PWM_OBJ_ADDR,
                                      .destination_id = uint8_t(esc_num + ESC_NODE_ID_FIRST),
                                      .source_id = AUTOPILOT_NODE_ID,
                                      .priority = 0,
                                      .unused = 0 } };

                    uavcan::CanFrame frame { (id.value | uavcan::CanFrame::FlagEFF), (uint8_t*) &tcan_pwm, sizeof(tcan_pwm) };

                    if (esc_num == 0) {
                        timeout = uavcan::MonotonicTime::fromUSec(now + SET_PWM_TIMEOUT_US);
                    } else {
                        timeout = uavcan::MonotonicTime::fromUSec(pwm_last_sent + SET_PWM_TIMEOUT_US);
                    }

                    // send frame to ESCs
                    int8_t res = _can_driver->getIface(CAN_IFACE_INDEX)->send(frame, timeout, 0);

                    // check if send was successful
                    if (res == 1) {
                        if (esc_num == 0) {
                            pwm_last_sent = now;

                            if (new_output) {
                                _new_output.store(false, std::memory_order_release);
                            }
                        }

                        sending_esc_num = (esc_num + 1) % _esc_max_node_id;
                    } else if (res == 0) {
                        debug_can(1, "ToshibaCAN: strange buffer full when sending message to ESC with ID %d\n\r", esc_num + ESC_NODE_ID_FIRST);
                    } else {
                        debug_can(1, "ToshibaCAN: error sending message to ESC with ID %d, result %d\n\r", esc_num + ESC_NODE_ID_FIRST, res);
                    }

                    break;
                }
            }
        }
    */
    }
}

// called from SRV_Channels
void AP_ToshibaCAN::update()
{
    if (_rc_out_sem.take(1)) {
        for (uint8_t i = 0; i < TOSHIBACAN_MAX_NUM_ESCS; i++) {
            if ((_esc_present_bitmask & (1 << i)) == 0) {
                continue;
            }

            SRV_Channel::Aux_servo_function_t motor_function = SRV_Channels::get_motor_function(i);

            if (SRV_Channels::function_assigned(motor_function)) {
                float norm_output = SRV_Channels::get_output_norm(motor_function);
                _scaled_output[i] = uint16_t((norm_output + 1.0f) / 2.0f * 2000.0f);
            } else {
                _scaled_output[i] = 0;
            }
        }

        _rc_out_sem.give();
        _new_output.store(true, std::memory_order_release);
    } else {
        debug_can(2, "ToshibaCAN: failed to get PWM semaphore on write\n\r");
    }
}

#endif // HAL_WITH_UAVCAN
