// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty
// Copyright (C) 2026 Luo1imasi

#pragma once

#include <atomic>
#include <limits>
#include <shared_mutex>
#include <string>

#include "motor_driver.hpp"
#include "protocol/canfd_iso.hpp"
// #include "protocol/ethercat_iso.hpp"
#include "utils.hpp"

// LRO V1.03 error codes (Byte0 bits 0-2). Query code 36 returns the detailed alarm.
enum LROError : uint8_t {
    LRO_NO_ERROR                     = 0x00,
    LRO_MOTOR_MOS_OVERHEAT            = 0x01,  // A.861 / A.7A0
    LRO_OVER_CURRENT                 = 0x02,  // A.100
    LRO_UNDER_VOLTAGE                = 0x03,  // A.410
    LRO_ENCODER_ERROR                = 0x04,  // A.C90 / A.C91 / A.830 / A.840 / A.CF1 / A.8A2
    LRO_PHASE_LOSS_OR_CURRENT_OFFSET = 0x05,  // UVW phase loss or excessive current sensing offset: A.B31 / A.B32
    LRO_OVER_VOLTAGE                 = 0x06,  // A.400
    LRO_OTHER_ERROR                  = 0x07,
};

enum LRO_Motor_Model {
    LRO_5550,
    LRO_6562,
    LRO_8462,
    LRO_10062,
    LRO_Num_Of_Motor
};

// LRO motor mode byte (upper 3 bits of Byte0 in control frames)
enum LROMotorMode : uint8_t {
    LRO_MODE_MIT     = 0x00,  // MIT impedance control
    LRO_MODE_POS     = 0x01,  // Servo position control
    LRO_MODE_SPD     = 0x02,  // Servo speed control
    LRO_MODE_CUR     = 0x03,  // Current/torque control
    LRO_MODE_CONFIG  = 0x06,  // Parameter configuration
    LRO_MODE_QUERY   = 0x07,  // Parameter query
};

// LRO 0x7FF setup command codes (Byte3)
enum LROSetupCmd : uint8_t {
    LRO_CMD_SET_ZERO   = 0x03,  // Set zero position
    LRO_CMD_SET_ID     = 0x04,  // Change motor ID
    LRO_CMD_RESET_ID   = 0x05,  // Reset ID to 0x01
    LRO_CMD_ENABLE     = 0x06,  // Enable motor
    LRO_CMD_DISABLE    = 0x07,  // Disable motor
    LRO_CMD_CLEAR_ERROR = 0x08, // Clear recoverable alarms
    LRO_CMD_QUERY_MODE = 0x81,  // Query communication mode
    LRO_CMD_QUERY_ID   = 0x82,  // Query motor ID
};

// LRO feedback message type (upper 3 bits of Byte0 in response)
enum LROFeedbackType : uint8_t {
    LRO_FB_TYPE1 = 0x01,  // pos(u16) + spd(u12) + iq(u12) + temp(u8) + mos_temp(u8) + dcvol(u16) + dccur(u16)
    LRO_FB_TYPE2 = 0x02,  // pos(float) + cur(i16) + temp(u8)
    LRO_FB_TYPE3 = 0x03,  // spd(float) + cur(i16) + temp(u8)
    LRO_FB_TYPE4 = 0x04,  // config result: code(u8) + status(u8)
    LRO_FB_TYPE5 = 0x05,  // query result: code(u8) + data(variable)
};

// LRO config codes (used with mode 0x06)
enum LROConfigCode : uint8_t {
    LRO_CFG_ACCEL       = 0x01,  // Set acceleration (rad/s^2)
    LRO_CFG_DECEL       = 0x02,  // Set deceleration (rad/s^2)
    LRO_CFG_MAX_SPD     = 0x03,  // Set maximum operating speed (rad/s)
    LRO_CFG_TORQUE_SENS = 0x04,  // Set torque constant (Nm/A)
    LRO_CFG_KP_MAX      = 0x05,  // Set maximum Kp limit
    LRO_CFG_KD_MAX      = 0x06,  // Set maximum Kd limit
    LRO_CFG_POS_MAX     = 0x07,  // Set maximum position limit (rad)
    LRO_CFG_SPD_MAX     = 0x08,  // Set maximum velocity limit (rad/s)
    LRO_CFG_TOR_MAX     = 0x09,  // Set maximum torque limit (Nm)
    LRO_CFG_CUR_MAX     = 0x0A,  // Set maximum current limit (A)
    LRO_CFG_TIMEOUT     = 0x0B,  // Set CAN communication timeout threshold (ms)
    LRO_CFG_CUR_PI      = 0x0C,  // Set current loop PI parameters
    LRO_CFG_SPD_PI      = 0x0D,  // Set speed loop PI parameters
    LRO_CFG_POS_PD      = 0x0E,  // Set position loop PD parameters
    LRO_CFG_KT_CALIB    = 0x0F  // Set torque constant calibration
};

// Query codes for Mode 0x07. Used to retrieve real-time internal information from the motor.
enum LROQueryCode : uint8_t {
    LRO_QRY_MANUFACTURER = 0x00,  // Manufacturer info (LRO=3)
    LRO_QRY_POS          = 0x01,  // Current position (degrees, float)
    LRO_QRY_SPD          = 0x02,  // Current speed (RPM, float)
    LRO_QRY_CUR          = 0x03,  // Current phase current (float)
    LRO_QRY_POWER        = 0x04,  // Current power (float)
    LRO_QRY_ACCEL        = 0x05,  // Current acceleration (uint16)
    LRO_QRY_FLUX_GAIN    = 0x06,  // Magnetic flux observation gain
    LRO_QRY_DIST_COMP    = 0x07,  // Disturbance compensation coefficient
    LRO_QRY_FB_COMP      = 0x08,  // Feedback compensation coefficient
    LRO_QRY_DAMPING      = 0x09,  // Damping coefficient
    LRO_QRY_KT           = 0x16,  // Torque constant (uint16 * 100)
    LRO_QRY_KP_RANGE     = 0x17,  // KP range (uint16 min + uint16 max)
    LRO_QRY_KD_RANGE     = 0x18,  // KD range (uint16 min + uint16 max)
    LRO_QRY_POS_RANGE    = 0x19,  // POS range (int16*100 + int16*100)
    LRO_QRY_SPD_RANGE    = 0x1A,  // SPD range (int16*100 + int16*100)
    LRO_QRY_TOR_RANGE    = 0x1B,  // TOR range (int16*10 + int16*10)
    LRO_QRY_CUR_RANGE    = 0x1C,  // CUR range (int16*10 + int16*10)
    LRO_QRY_MCU_UUID     = 0x1D,  // MCU UUID (3 frames)
    LRO_QRY_VERSION      = 0x1E,  // Software & hardware version (12 bytes)
    LRO_QRY_CAN_TIMEOUT  = 0x1F,  // CAN timeout (uint16 ms)
    LRO_QRY_CUR_PI       = 0x20,  // Current loop KP/KI
    LRO_QRY_SPD_PI       = 0x21,  // Speed loop KP/KI
    LRO_QRY_POS_PD       = 0x22,  // Position loop KP/KD
    LRO_QRY_KT_CALIB     = 0x23,  // Torque constant calibration enable
};

// Parameter ranges for MIT mode (configurable per motor)
typedef struct {
    float PosMax;   // Maximum position (rad), default 12.5
    float SpdMax;   // Maximum velocity (rad/s), default 45
    float TauMax;   // Maximum torque (Nm), default 40
    float CurMax;   // Iq feedback encoding limit (A), independent of torque
    float OKpMax;    // Default 500
    float OKdMax;    // Default 5
} LRO_Limit_Param;

class LroMotorDriver : public MotorDriver {
   public:
    LroMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                    LRO_Motor_Model motor_model, double motor_zero_offset = 0.0);
    ~LroMotorDriver();

    virtual void lock_motor() override;
    virtual void unlock_motor() override;
    virtual uint8_t init_motor() override;
    virtual void deinit_motor() override;
    virtual void set_motor_zero() override;
    virtual bool write_motor_flash() override;
    virtual void get_motor_param(uint8_t param_cmd) override;

    virtual void motor_pos_cmd(float pos, float spd, bool ignore_limit) override;
    virtual void motor_spd_cmd(float spd) override;
    virtual void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
    virtual void motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) override;
    virtual void set_motor_control_mode(uint8_t motor_control_mode) override;
    virtual int get_response_count() const override { 
        return response_count_; 
    }
    virtual void set_motor_id(uint8_t old_id, uint8_t new_id) override;
    virtual void reset_motor_id() override;
    virtual void refresh_motor_status() override;
    virtual void clear_motor_error() override;

    // NaN until a 12-byte type-1 frame arrives, or when feedback is 8 bytes.
    float get_dc_bus_voltage() const { return dc_bus_voltage_.load(); }  // V
    float get_dc_bus_current() const { return dc_bus_current_.load(); }  // A

   private:
    uint8_t motor_index_{0};

    std::atomic<float> dc_bus_voltage_{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<float> dc_bus_current_{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<int> response_count_{0};
    LRO_Motor_Model motor_model_;
    LRO_Limit_Param limit_param_;
    std::atomic<uint8_t> mos_temperature_{0};
    void set_motor_zero_lro();
    void clear_motor_error_lro();
    void write_register_lro(uint8_t rid, float value);
    void write_register_lro(uint8_t index, int32_t value);
    void save_register_lro();

    virtual void canfd_rx_cbk(const canfd_frame& rx_frame);
    // virtual void ethercat_rx_cbk(const ethercat_frame& rx_frame);
    std::shared_ptr<MotorsCANFD> canfd_;
    // std::shared_ptr<MotorsEthercat> ethercat_;

    inline static std::shared_mutex bus_registry_mutex_;
    inline static std::unordered_map<std::string, std::vector<LroMotorDriver*>> bus_registry_;
};
