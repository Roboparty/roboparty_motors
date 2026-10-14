// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty
// Copyright (C) 2026 lixingyi-2006

// version 1.0.0 2026.7.29 wentywenty init
// version 1.0.1 2026.8.6 lixingyi-2006 update motor_mit_cmd to support multi-motor command
// version 1.0.2 2026.8.6 wentywenty update hanshu
// version 1.0.3 2026.8.7 lixingyi-2006 add hanshu and update mit miti control api
// version 1.0.4 2026.8.7 wentywenty update hanshu
// version 1.0.5 2026.8.7 lixingyi-2006 update Comments
// version 1.0.6 2026.8.7 wentywenty update hanshu

#pragma once

#include <atomic>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "motor_driver.hpp"
#include "protocol/canfd_iso.hpp"
// #include "protocol/ethercat_iso.hpp"
#include "utils.hpp"

// Error code values are not defined here
enum RPError {
    RP_NO_ERROR             = 0x00,
    RP_OVER_VOLTAGE         = 0x01,
    RP_OVER_CURRENT         = 0x02,
    RP_MOTOR_OVER_TEMP      = 0x03,
    RP_BOARD_OVER_TEMP      = 0x04,
    RP_UNDER_VOLTAGE        = 0x05,
    RP_ENCODER_FAULT        = 0x06,
    RP_COMM_FAULT           = 0x07,
    RP_WARN_MOTOR_OVER_TEMP = 0x08,
    RP_WARN_BOARD_OVER_TEMP = 0x09,
};


enum RP_Motor_Model {
    RP_6562,
    RP_8462,
    RP_10062,
    RP_Num_Of_Model
};


enum RPCanId : uint16_t {
    RP_SET_ZERO_BASE       = 0x380,
    RP_ENABLE_BASE         = 0x3C0,
    RP_MODE_BASE           = 0x3E0,
    RP_MULTI_MIT           = 0x400,
    RP_MULTI_MIT_RESP_BASE = 0x410,
    RP_REQUEST_DATA_BASE   = 0x500,
    RP_BROADCAST           = 0x7FF,
};

enum RPCommand : uint8_t {
    RP_CONTROLLER_TO_MOTOR = 0x00,
    RP_BC_ID_CHANGE        = 0x11,
    RP_BC_ID_RESET         = 0x12,
    RP_DISABLE                      = 0x00,
    RP_ENABLE                       = 0x01,
    RP_MODE_MIT                     = 0x20,
    RP_SET_CURRENT_POSITION_AS_ZERO = 0x01,
    RP_CLEAR_ERROR                  = 0x80,
    RP_REQUEST_DATA                 = 0x01,
};

typedef struct {
    float PosMax;   
    float SpdMax;   
    float TauMax;   
    float OKpMax;   
    float OKdMax;   
} RP_Limit_Param;

class RPMotorDriver : public MotorDriver {
   public:
    RPMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                  RP_Motor_Model motor_model, double motor_zero_offset = 0.0);
    ~RPMotorDriver() override;

    void lock_motor() override;
    void unlock_motor() override;
    uint8_t init_motor() override;
    void deinit_motor() override;
    void set_motor_zero() override;
    bool write_motor_flash() override { return false; }
    void get_motor_param(uint8_t param_cmd) override {}
    
    void motor_pos_cmd(float pos, float spd, bool ignore_limit) override {}
    void motor_spd_cmd(float spd) override {}
    void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override {}
    void motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) override;
    void set_motor_control_mode(uint8_t motor_control_mode) override;
    int get_response_count() const override {
        return response_count_; 
    }
    void set_motor_id(uint8_t old_id, uint8_t new_id) override;
    void reset_motor_id() override;
    void refresh_motor_status() override;
    void clear_motor_error() override;

   private:
    uint8_t motor_index_{0};
    uint16_t device_id_;

    std::atomic<int> response_count_{0};
    RP_Motor_Model motor_model_;
    RP_Limit_Param limit_param_;
    std::atomic<float> mos_temperature_{0.0f};
    void canfd_rx_cbk(const canfd_frame& rx_frame);
    // virtual void ethercat_rx_cbk(const ethercat_frame& rx_frame);
    std::shared_ptr<MotorsCANFD> canfd_;
    // std::shared_ptr<MotorsEthercat> ethercat_;

    inline static std::shared_mutex bus_registry_mutex_;
    inline static std::unordered_map<std::string, std::vector<RPMotorDriver*>> bus_registry_;
};
