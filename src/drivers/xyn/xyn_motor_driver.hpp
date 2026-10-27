// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#pragma once

#include <atomic>
#include <shared_mutex>
#include <string>

#include "motor_driver.hpp"
#include "protocol/canfd/socket_canfd.hpp"
// #include "protocol/ethercat_iso.hpp"
#include "utils.hpp"

// XYN error codes (reported via 0x00F)
enum XYNError {
    XYN_NO_ERROR            = 0x00,
    XYN_OVER_VOLTAGE        = 0x01,
    XYN_OVER_CURRENT        = 0x02,
    XYN_MOTOR_OVER_TEMP     = 0x03,
    XYN_BOARD_OVER_TEMP     = 0x04,
    XYN_UNDER_VOLTAGE       = 0x05,
    XYN_ENCODER_FAULT       = 0x06,
    XYN_COMM_FAULT          = 0x07,
    XYN_WARN_MOTOR_OVER_TEMP = 0x08,
    XYN_WARN_BOARD_OVER_TEMP = 0x09,
};

enum XYN_Motor_Model {
    XYN_5550,
    XYN_5757,
    XYN_6562,
    XYN_8462,
    XYN_10062,
    XYN_Num_Of_Model
};

// XYN motor modes (written to 0x08)
enum XYNMotorMode : uint8_t {
    XYN_MODE_POSITION = 0x01,
    XYN_MODE_SPEED    = 0x02,
    XYN_MODE_TORQUE   = 0x03,
    XYN_MODE_MIT      = 0x08,
};

// 0x004 start/stop command values
enum XYNStartStop : uint8_t {
    XYN_START = 0x01,
    XYN_STOP  = 0x00,
};

// 0x004 return values (per Xynova protocol)
enum XYNStartStopAck : uint8_t {
    XYN_ACK_FAIL           = 0x00,
    XYN_ACK_SUCCESS        = 0x01,
    XYN_ACK_ZERO_EFFECTIVE = 0x02,  // software zero took effect (returned on stop)
};

// Xynova CAN-FD Message IDs
enum XYNMsgId : uint16_t {
    XYN_MSG_ENABLE       = 0x001,  // Enable/Disable motor
    XYN_MSG_MODE         = 0x002,  // Set control mode
    XYN_MSG_START        = 0x004,  // Start/Stop motor
    XYN_MSG_SET_ZERO     = 0x006,  // Set current position as zero
    XYN_MSG_RESTORE_ZERO = 0x007,  // Restore zero to factory value
    XYN_MSG_SAVE_BASIC   = 0x0009, // Save basic params to FLASH
    XYN_MSG_SAVE_CTRL    = 0x00A,  // Save control params to FLASH
    XYN_MSG_SAVE_LIMIT   = 0x00B,  // Save limit params to FLASH
    XYN_MSG_ERROR        = 0x00F,  // Fault code (motor pushes on fault); write 0 to clear
    XYN_MSG_DEVICE_ID    = 0x011,  // Read/Write device ID
    XYN_MSG_SAVE_ZERO    = 0x012,  // Save software zero to FLASH
    XYN_MSG_POS_TARGET   = 0x131,  // Position target (float, degrees)
    XYN_MSG_SPD_TARGET   = 0x133,  // Speed target (float, rpm)
    XYN_MSG_TRQ_TARGET   = 0x134,  // Torque target (float, Nm)
    XYN_MSG_MIT_TEST     = 0x138,  // MIT test mode command (24 bytes)
    XYN_MSG_MIT_STD      = 0x139,  // MIT standard mode command (20 bytes: pos+spd+trq+kp+kd)
    XYN_MSG_SPD_LIMIT    = 0x141,  // Speed limit (float, rpm)
    XYN_MSG_DECEL_LIMIT  = 0x142,  // Deceleration limit (float, rpm/s)
    XYN_MSG_TRQ_SLOPE    = 0x151,  // Torque slope (float, Nm/s)
    XYN_MSG_POS_UPPER    = 0x240,  // Position upper limit (float, degrees)
    XYN_MSG_POS_LOWER    = 0x241,  // Position lower limit (float, degrees)
    XYN_MSG_SPD_CLAMP    = 0x242,  // Speed clamp (float, rpm)
    XYN_MSG_TRQ_CLAMP    = 0x243,  // Torque clamp (float, Nm)
    XYN_MSG_CUR_CLAMP    = 0x244,  // Current clamp (float, A)
    XYN_MSG_READ_INTRINSIC = 0x21F, // Read motor intrinsic params (11*float)
    XYN_MSG_READ_LIMITS  = 0x24F,  // Read limit params (5*float)
    XYN_MSG_RESTORE_DEF  = 0x251,  // Restore default params
    XYN_MSG_MONITOR      = 0x31F,  // Monitor: 9 floats feedback
    XYN_MSG_MIT_LPF_SET  = 0x0230, // MIT LPF filter config (3 floats)
    XYN_MSG_MIT_LPF_GET  = 0x0231, // MIT LPF filter read (3 floats)
    XYN_MSG_MIT_KPKD_SET = 0x0232, // MIT Kp/Kd range config (4 floats)
    XYN_MSG_MIT_KPKD_GET = 0x0233, // MIT Kp/Kd range read (4 floats)
    XYN_MSG_MIT_MULTI    = 0x8080, // MIT multi-motor mode (compact encoding)
};

// Parameter ranges for MIT mode (configurable per motor)
typedef struct {
    float PosMax;   // Maximum position (rad), default 12.5
    float SpdMax;   // Maximum velocity (rad/s), default 45
    float TauMax;   // Maximum torque (Nm), default 40
    // float OKpMin;   // Default 0
    float OKpMax;   // Default 500
    // float OKdMin;   // Default 0
    float OKdMax;   // Default 5
} XYN_Limit_Param;

class XynMotorDriver : public MotorDriver {
   public:
    XynMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                    XYN_Motor_Model motor_model, double motor_zero_offset = 0.0);
    ~XynMotorDriver();

    virtual void lock_motor() override;
    virtual void unlock_motor() override;
    virtual uint8_t init_motor() override;
    virtual void deinit_motor() override;

    /**
     * @note Saving the zero position to flash requires the motor to be disabled,
     *       so calling set_motor_zero() will disable the motor.
     */
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
    /**
     * @note Feedback differs from DM/LRO/EVO/RP : XYN feedback is not a command
     *       reply. motor_pos_ / motor_spd_ etc. are only updated after calling
     *       refresh_motor_status() (which sends the 0x31F monitor query), otherwise
     *       these values never update automatically. Replies also lag by ~1 ms.
     */
    virtual void refresh_motor_status() override;
    virtual void clear_motor_error() override;

   private:
    uint8_t motor_index_{0};

    std::atomic<int> response_count_{0};
    XYN_Motor_Model motor_model_;
    XYN_Limit_Param limit_param_;
    uint16_t device_id_;
    std::atomic<uint8_t> mos_temperature_{0};
    void start_motor();
    void stop_motor();
    // Set Kp/Kd range used for 0x8080 multi-motor protocol-value mapping (0x0232)
    void set_kp_kd_range(float kp_min, float kp_max, float kd_min, float kd_max);
    // 0x0233 read Kp/Kd range
    std::atomic<float> kp_min_{0.0f};
    std::atomic<float> kp_max_{0.0f};
    std::atomic<float> kd_min_{0.0f};
    std::atomic<float> kd_max_{0.0f};
    // 0x0231 read MIT LPF cutoff: position/speed/torque LPF cutoff (Hz)
    std::atomic<float> pos_lpf_{0.0f};
    std::atomic<float> spd_lpf_{0.0f};
    std::atomic<float> trq_lpf_{0.0f};
    // 0x21F read peak speed / peak torque
    std::atomic<float> peak_speed_{0.0f};   // rad/s
    std::atomic<float> peak_torque_{0.0f};
    std::atomic<float> inertia_{0.0f};      // kg·m²
    // 0x24F read pos upper/lower, speed clamp, torque clamp, current clamp
    std::atomic<float> pos_upper_limit_{0.0f};  // rad
    std::atomic<float> pos_lower_limit_{0.0f};  
    std::atomic<float> spd_clamp_{0.0f};        // rad/s
    std::atomic<float> trq_clamp_{0.0f};        
    std::atomic<float> cur_clamp_{0.0f};        // A
    void save_motor_zero_xyn();

    void set_motor_zero_xyn();
    void write_register_xyn(uint8_t index, int32_t value);
    void save_register_xyn();

    virtual void canfd_rx_cbk(const canfd_frame& rx_frame);
    // virtual void ethercat_rx_cbk(const ethercat_frame& rx_frame);
    std::shared_ptr<MotorsSocketCANFD> canfd_;
    // std::shared_ptr<MotorsEthercat> ethercat_;

    inline static std::shared_mutex bus_registry_mutex_;
    inline static std::unordered_map<std::string, std::vector<XynMotorDriver*>> bus_registry_;
};
