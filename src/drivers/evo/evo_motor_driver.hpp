// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2025-2026 Luo1imasi
// Copyright (C) 2025-2026 wentywenty

#pragma once

#include <atomic>
#include <string>

#include "motor_driver.hpp"
#include "protocol/can_iso.hpp"
#include "protocol/canfd_iso.hpp"
#include "utils.hpp"

enum EVOError {
    EVO_NO_ERROR = 0x00,
    EVO_OVER_VOLTAGE = 0x01,
    EVO_UNDER_VOLTAGE = 0x02,
    EVO_ELECTRIC_ANGLE_ERROR = 0x03,
    EVO_DRV_ERROR = 0x04,
    EVO_BUS_OVER_CURRENT = 0x05,
    EVO_PHASE_A_OVER_CURRENT = 0x06,
    EVO_PHASE_B_OVER_CURRENT = 0x07,
    EVO_PHASE_C_OVER_CURRENT = 0x08,
    EVO_PCB_OVER_TEMP = 0x09,
    EVO_COIL_OVER_TEMP = 0x0A,
    EVO_ENCODER_ERROR = 0x0B,
    EVO_CURRENT_SENSOR_ERROR = 0x0C,
    EVO_POSITION_OVER_LIMIT = 0x0D,
    EVO_OVER_SPEED = 0x0E,
    EVO_STALL_PROTECTION = 0x0F,
    EVO_CAN_COMM_LOST = 0x10,

    EVO_ENCODER_FRAME_HEAD_CHECK_ERROR = 0x81,
    EVO_ENCODER_MULTI_TURN_ERROR = 0x82,
    EVO_ENCODER_MULTI_TURN_INTERFACE_ERROR = 0x83,
    EVO_ENCODER_EXTERNAL_INPUT_ERROR = 0x84,
    EVO_ENCODER_READ_ERROR = 0x85,
    EVO_ENCODER_ERR_OFFS = 0x86,
    EVO_ENCODER_ERR_CFG = 0x87,
    EVO_ILLEGAL_FIRMWARE = 0x88,
    EVO_GATE_DRIVER_INIT_ERROR = 0x89,

    // Compatibility aliases for older code using the previous enum names.
    EVO_OVER_CURRENT = EVO_PHASE_A_OVER_CURRENT,
    EVO_MOS_OVER_TEMP = EVO_PCB_OVER_TEMP,
    EVO_OVERLOAD = EVO_STALL_PROTECTION,
    EVO_COMM_LOST = EVO_CAN_COMM_LOST,
    EVO_POS_OVER_LIMIT = EVO_POSITION_OVER_LIMIT,

    EVO_UNKNOWN_ERROR = 0xFF
};

enum EVO_Motor_Model { 
    EVO431040,
    EVO811825,
    EVO811832,
    EVO_Num_Of_Motor 
};

enum EVO_REG : uint8_t {
    // --- System & Core Information ---
    EVO_REG_FIRMWARE_VER    = 10,  // Firmware Version (Int32)
    EVO_REG_MOTOR_ID        = 36,  // Motor CAN ID (Int32)
    EVO_REG_NPP             = 44,  // Number of Pole Pairs (Int32)
    EVO_REG_GEAR_RATIO      = 45,  // Gear Reduction Ratio (Float32)

    // --- Current Loop Control (FOC) ---
    EVO_REG_CUR_KP_D        = 12,  // D-axis Current Loop Kp (Float32)
    EVO_REG_CUR_KI_D        = 13,  // D-axis Current Loop Ki (Float32)
    EVO_REG_CUR_KP_Q        = 14,  // Q-axis Current Loop Kp (Float32)
    EVO_REG_CUR_KI_Q        = 15,  // Q-axis Current Loop Ki (Float32)

    // --- Control Deadzones ---
    EVO_REG_DEADZONE_CUR    = 16,  // Current Deadzone (Float32)
    EVO_REG_DEADZONE_VEL    = 17,  // Velocity Deadzone (Float32)
    EVO_REG_DEADZONE_POS    = 18,  // Position Deadzone (Float32)

    // --- Motion Dynamics & Limits ---
    EVO_REG_TORQUE_LIMIT    = 21,  // Torque Protection Limit (Float32)
    EVO_REG_ZERO_OFFSET     = 23,  // Mechanical Zero Offset (Float32)
    EVO_REG_ACCELERATION    = 42,  // Servo Mode Acceleration (Float32)
    EVO_REG_TORQUE_SLOPE    = 43,  // Torque Rise Rate / Slope (Float32)

    // --- CAN Communication Range Config (CRITICAL for range_map alignment) ---
    EVO_REG_CAN_THETA_MIN   = 24,  // Position Mapping Minimum (rad)
    EVO_REG_CAN_THETA_MAX   = 25,  // Position Mapping Maximum (rad)
    EVO_REG_CAN_VEL_MIN     = 26,  // Velocity Mapping Minimum (rad/s)
    EVO_REG_CAN_VEL_MAX     = 27,  // Velocity Mapping Maximum (rad/s)
    EVO_REG_CAN_KP_MAX      = 29,  // Kp Mapping Maximum Value
    EVO_REG_CAN_KD_MAX      = 31,  // Kd Mapping Maximum Value
    EVO_REG_CAN_TORQUE_MIN  = 34,  // Torque Mapping Minimum (Nm)
    EVO_REG_CAN_TORQUE_MAX  = 35,  // Torque Mapping Maximum (Nm)
    EVO_REG_CAN_TIMEOUT     = 37,  // CAN Communication Timeout Threshold (Int32/ms)

    // --- Safety & Protection Thresholds ---
    EVO_REG_OV_LOCK         = 49,  // Over-Voltage (Bus) Lockout (Float32)
    EVO_REG_UV_LOCK         = 50,  // Under-Voltage (Bus) Lockout (Float32)
    EVO_REG_OC_LOCK         = 51,  // Over-Current (Phase) Lockout (Float32)
    EVO_REG_OT_LOCK         = 52,  // Over-Temperature (Board) Lockout (Float32)
    EVO_REG_STUCK_TIME      = 55,  // Stall/Stuck Protection Time (Float32)
    EVO_REG_PROTECT_SWITCH  = 56  // Protection Feature Switch Bits (Int32)
};

enum EVO_CMD {
    EVO_CMD_CLEAR_ERROR = 0xFB, ///< Clear recoverable errors (CAN-FD module protocol)
    EVO_CMD_ENABLE = 0xFC,      ///< Enable motor mode and reset motor state
    EVO_CMD_DISABLE = 0xFD,      ///< Reset motor and clear errors
    EVO_CMD_SET_ZERO = 0xFE,        ///< Set current position as zero point
    EVO_CMD_START_FLASH = 0x67,      ///< Start flash operation
    EVO_CMD_END_FLASH = 0x76,      ///< End flash operation
    EVO_CMD_READ_FLASH = 0x04,      ///< Read flash operation
    EVO_CMD_WRITE_FLASH = 0x15,      ///< Write flash operation
    EVO_CMD_SAVE_FLASH = 0x00      ///< Save flash operation
};

enum EVO_Flash_Param {
    EVO_PARAM_Q_MAX = 0x00,         ///< Maximum position limit
    EVO_PARAM_Q_MIN = 0x01,         ///< Minimum position limit
    EVO_PARAM_DQ_MAX = 0x02,        ///< Maximum velocity limit
    EVO_PARAM_DQ_MIN = 0x03,        ///< Minimum velocity limit
    EVO_PARAM_TAU_MAX = 0x04,       ///< Maximum torque/current limit
    EVO_PARAM_TAU_MIN = 0x05,       ///< Minimum torque/current limit
    EVO_PARAM_OKP_MAX = 0x06,       ///< Maximum outer Kp
    EVO_PARAM_OKP_MIN = 0x07,       ///< Minimum outer Kp
    EVO_PARAM_OKD_MAX = 0x08,       ///< Maximum outer Kd
    EVO_PARAM_OKD_MIN = 0x09,       ///< Minimum outer Kd
    EVO_PARAM_IKP_MAX = 0x0A,       ///< Maximum inner Kp
    EVO_PARAM_IKP_MIN = 0x0B,       ///< Minimum inner Kp
    EVO_PARAM_IKI_MAX = 0x0C,       ///< Maximum inner Ki
    EVO_PARAM_IKI_MIN = 0x0D,       ///< Minimum inner Ki
    EVO_PARAM_CUR_MAX = 0x0E,       ///< Maximum current
    EVO_PARAM_CUR_MIN = 0x0F        ///< Minimum current
};

// EVO-FD specific
enum EVOFD_CMD : uint16_t {
    EVOFD_CMD_ID = 0x10,
    EVOFD_MIT_ID = 0x20
};

typedef struct {
    float PosMax;       ///< Maximum position limit (rad)
    float SpdMax;       ///< Maximum velocity limit (rad/s)
    float TauMax;       ///< Maximum torque limit (N·m)
    float OKpMax;       ///< Maximum outer-loop proportional gain
    float OKdMax;       ///< Maximum outer-loop derivative gain
} EVO_Limit_Param;

class EvoMotorDriver : public MotorDriver {
   public:
    EvoMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                   EVO_Motor_Model motor_model, double motor_zero_offset = 0.0);
    ~EvoMotorDriver();

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
    virtual int get_response_count() const { 
        return response_count_; 
    }
    virtual void set_motor_id(uint8_t old_id, uint8_t new_id) override;
    virtual void reset_motor_id() override;
    virtual void refresh_motor_status() override;
    virtual void clear_motor_error() override;

   private:
    uint8_t motor_index_{0};

    std::atomic<int> response_count_{0};
    EVO_Motor_Model motor_model_;
    EVO_Limit_Param limit_param_;
    std::atomic<uint8_t> mos_temperature_{0};
    void set_motor_zero_evo();
    void clear_motor_error_evo();
    void write_register_evo(uint8_t rid, float value);
    void write_register_evo(uint8_t index, int32_t value);
    void save_register_evo();

    virtual void can_rx_cbk(const can_frame& rx_frame);
    virtual void canfd_rx_cbk(const canfd_frame& rx_frame);
    std::shared_ptr<MotorsCAN> can_;
    std::shared_ptr<MotorsCANFD> canfd_;

    inline static std::mutex bus_registry_mutex_;
    inline static std::unordered_map<std::string, std::vector<EvoMotorDriver*>> bus_registry_; 
};
