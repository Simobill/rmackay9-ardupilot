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

#pragma once

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <atomic>

// there are 12 motor functions in SRV_Channel but CAN driver can't keep up
#define TOSHIBACAN_MAX_NUM_ESCS 8

class AP_ToshibaCAN : public AP_HAL::CANProtocol {
public:
    AP_ToshibaCAN();
    ~AP_ToshibaCAN();

    /* Do not allow copies */
    AP_ToshibaCAN(const AP_ToshibaCAN &other) = delete;
    AP_ToshibaCAN &operator=(const AP_ToshibaCAN&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // Return ToshibaCAN from @driver_index or nullptr if it's not ready or doesn't exist
    static AP_ToshibaCAN *get_tcan(uint8_t driver_index);

    // initialise ToshibaCAN bus
    void init(uint8_t driver_index) override;

    // called from SRV_Channels
    void update();

private:

    // loop to send output to ESCs in background thread
    void loop();

    bool _initialized;
    char _thread_name[9];
    uint8_t _driver_index;
    uavcan::ICanDriver* _can_driver;

    AP_Int8 _num_poles;

    // ESC detected information
    uint16_t _esc_present_bitmask = 0x15;
    uint8_t _esc_max_node_id;

    // PWM output
    HAL_Semaphore _rc_out_sem;
    std::atomic<bool> _new_output;
    uint16_t _scaled_output[TOSHIBACAN_MAX_NUM_ESCS];

    // structure for sending motor lock command to ESC
    union motor_lock_cmd_t {
        struct PACKED {
            uint8_t motor2:4;
            uint8_t motor1:4;
            uint8_t motor4:4;
            uint8_t motor3:4;
            uint8_t motor5:4;
            uint8_t motor6:4;
            uint8_t motor7:4;
            uint8_t motor8:4;
            uint8_t motor9:4;
            uint8_t motor10:4;
            uint8_t motor11:4;
            uint8_t motor12:4;
        };
        uint8_t data[6];
    };

    // structure for sending turn rate command to ESC
    union motor_rotation_cmd_t {
        struct PACKED {
            uint16_t motor4;
            uint16_t motor3;
            uint16_t motor2;
            uint16_t motor1;
        };
        uint8_t data[8];
    };

    // data format for messages from flight controller
    static const uint8_t COMMAND_STOP = 0x0;
    static const uint8_t COMMAND_LOCK = 0x10;
    static const uint8_t COMMAND_REQUEST_DATA = 0x20;
    static const uint8_t COMMAND_MOTOR3 = 0x3B;
    static const uint8_t COMMAND_MOTOR2 = 0x3D;
    static const uint8_t COMMAND_MOTOR1 = 0x3F;

    // data format for messages from ESC
    static const uint8_t MOTOR_DATA1 = 0x40;
    static const uint8_t MOTOR_DATA2 = 0x50;
    static const uint8_t MOTOR_DATA3 = 0x60;
    static const uint8_t MOTOR_DATA5 = 0x80;

    static const uint8_t AUTOPILOT_NODE_ID = 0;
    static const uint8_t BROADCAST_NODE_ID = 1;
    static const uint8_t ESC_NODE_ID_FIRST = 2;

    static const uint8_t ESC_INFO_OBJ_ADDR = 0;
    static const uint8_t SET_PWM_OBJ_ADDR = 1;
    static const uint8_t VOLTAGE_OBJ_ADDR = 2;
    static const uint8_t CURRENT_OBJ_ADDR = 3;
    static const uint8_t RPM_OBJ_ADDR = 4;
    static const uint8_t TEMPERATURE_OBJ_ADDR = 5;
    static const uint8_t GET_PWM_INPUT_OBJ_ADDR = 6;
    static const uint8_t GET_PWM_OUTPUT_OBJ_ADDR = 7;
    static const uint8_t MCU_ID_OBJ_ADDR = 8;
    static const uint8_t UPDATE_NODE_ID_OBJ_ADDR = 9;
    static const uint8_t ENUM_OBJ_ADDR = 10;
    static const uint8_t TELEMETRY_OBJ_ADDR = 11;

    static const uint16_t SET_PWM_MIN_INTERVAL_US = 2500;
    static const uint32_t TELEMETRY_INTERVAL_US = 100000;

    static const uint32_t SET_PWM_TIMEOUT_US = 2000;
    static const uint16_t TELEMETRY_TIMEOUT_US = 500;
    static const uint16_t ENUMERATION_TIMEOUT_MS = 30000;

    static const uint8_t CAN_IFACE_INDEX = 0;
};
