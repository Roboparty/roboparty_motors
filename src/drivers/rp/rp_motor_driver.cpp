// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty
// Copyright (C) 2026 lixingyi-2006

// version 1.0.0 2026.7.29 wentywenty init
// version 1.0.1 2026.8.6 lixingyi-2006 update motor_mit_cmd to support multi-motor command
// version 1.0.2 2026.8.6 wentywenty update hanshu
// version 1.0.3 2026.8.7 lixingyi-2006 add hanshu and update mit miti control api
// version 1.0.4 2026.8.7 wentywenty update hanshu
// version 1.0.5 2026.8.7 lixingyi-2006 update hanshu 
// version 1.0.6 2026.8.7 wentywenty update hanshu

#include "rp_motor_driver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
constexpr uint8_t kRPBatchMotorCount = 7;
constexpr uint8_t kRPCurrentReportType = 6;

int16_t read_be_i16(const uint8_t* data) {
    return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) |
                                static_cast<uint16_t>(data[1]));
}

int32_t read_be_i32(const uint8_t* data) {
    return static_cast<int32_t>((static_cast<uint32_t>(data[0]) << 24) |
                                (static_cast<uint32_t>(data[1]) << 16) |
                                (static_cast<uint32_t>(data[2]) << 8) |
                                static_cast<uint32_t>(data[3]));
}

constexpr RP_Limit_Param rp_limit_param[RP_Num_Of_Model] = {
    {6.283185f, 20.94395f, 34.0f, 500.0f, 100.0f},  // 6562
    {6.283185f, 16.23156f, 42.0f, 500.0f, 100.0f},  // 8462
    {6.283185f, 13.6135679f, 140.0f, 500.0f, 100.0f}, // 10062
};
}  // namespace

RPMotorDriver::RPMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                             RP_Motor_Model motor_model, double motor_zero_offset)
    : MotorDriver(), motor_model_(motor_model) {
    if (interface_type != "canfd" && interface_type != "ethercanfd") {
        throw std::runtime_error("RP driver only supports CAN-FD and ethercanfd");
    }
    if (motor_model < RP_6562 || motor_model >= RP_Num_Of_Model) {
        throw std::invalid_argument("Invalid RP motor model");
    }
    if (motor_id == 0 || motor_id > kRPBatchMotorCount) {
        throw std::invalid_argument("RP batch MIT motor ID must be in the range [1, 7]");
    }
    motor_id_ = motor_id;
    limit_param_ = rp_limit_param[motor_model_];
    can_interface_ = can_interface;
    motor_zero_offset_ = motor_zero_offset;

    if (interface_type == "canfd" || interface_type == "ethercanfd") {
        comm_type_ = CommType::CANFD;
        device_id_ = motor_id;
        motor_index_ = device_id_ - 1;
        canfd_ = MotorsCANFD::get(can_interface);

        CanFdCbkFunc canfd_callback =
            std::bind(&RPMotorDriver::canfd_rx_cbk, this, std::placeholders::_1);
        canfd_->add_canfd_callback(
            canfd_callback,
            static_cast<CanFdCbkId>(RP_MULTI_MIT_RESP_BASE + device_id_));
        std::lock_guard<std::shared_mutex> lock(bus_registry_mutex_);
        bus_registry_[can_interface].push_back(this);
    } else if (interface_type == "ethercat") {
        comm_type_ = CommType::ETHERCAT;
        throw std::runtime_error("RP driver does not support EtherCAT interface yet");
    }
}

RPMotorDriver::~RPMotorDriver() {
    if (comm_type_ == CommType::CANFD) {
        canfd_->remove_canfd_callback(
            static_cast<CanFdCbkId>(RP_MULTI_MIT_RESP_BASE + device_id_));
        std::lock_guard<std::shared_mutex> lock(bus_registry_mutex_);
        auto it = bus_registry_.find(can_interface_);
        if (it != bus_registry_.end()) {
            auto& motors = it->second;
            motors.erase(std::remove(motors.begin(), motors.end(), this), motors.end());
            if (motors.empty()) {
                bus_registry_.erase(it);
            }
        }
    }
}

void RPMotorDriver::lock_motor() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = RP_ENABLE_BASE + device_id_;
        tx_frame.len = 0x01;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = RP_ENABLE;

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void RPMotorDriver::unlock_motor() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = RP_ENABLE_BASE + device_id_;
        tx_frame.len = 0x01;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = RP_DISABLE;

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void RPMotorDriver::refresh_motor_status() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = RP_REQUEST_DATA_BASE + device_id_;
        tx_frame.len = 0x01;
        tx_frame.flags = CANFD_BRS;
        tx_frame.data[0] = RP_REQUEST_DATA;

        canfd_->transmit(tx_frame);
        response_count_++;
    }
}

uint8_t RPMotorDriver::init_motor() {
    // Send disable command to enter config mode
    RPMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
    RPMotorDriver::set_motor_control_mode(MIT);  // Default to MIT mode
    Timer::sleep_for(normal_sleep_time);
    // Send enable command to enter control mode
    RPMotorDriver::lock_motor();
    Timer::sleep_for(normal_sleep_time);
    RPMotorDriver::refresh_motor_status();
    Timer::sleep_for(normal_sleep_time);
    return error_id_;
}

void RPMotorDriver::deinit_motor() {
    RPMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
}

void RPMotorDriver::set_motor_zero() {
    if (comm_type_ != CommType::CANFD) {
        return;
    }

    canfd_frame tx_frame{};
    tx_frame.can_id = RP_SET_ZERO_BASE + device_id_;
    tx_frame.len = 0x05;
    tx_frame.flags = CANFD_BRS;
    tx_frame.data[0] = RP_SET_CURRENT_POSITION_AS_ZERO;

    canfd_->transmit(tx_frame);
    Timer::sleep_for(setup_sleep_time);
}

void RPMotorDriver::clear_motor_error() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = RP_ENABLE_BASE + device_id_;
        tx_frame.len = 0x01;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = RP_CLEAR_ERROR;

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void RPMotorDriver::canfd_rx_cbk(const canfd_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    const uint8_t feedback_type = rx_frame.data[0];
    if (feedback_type != kRPCurrentReportType) {
        return;
    }

    error_id_ = (static_cast<uint16_t>(rx_frame.data[3]) << 8) | rx_frame.data[4];
    if (error_id_ > 0) {
        if (logger_) {
            logger_->error("can_interface: {0}\tmotor_id: {1}\terror_id: 0x{2:x}", can_interface_, motor_id_, static_cast<uint32_t>(error_id_));
        }
    }

    const int32_t pos_int = read_be_i32(&rx_frame.data[5]);
    const int32_t spd_int = read_be_i32(&rx_frame.data[9]);
    const int16_t current_int = read_be_i16(&rx_frame.data[13]);
    motor_pos_ = static_cast<float>(pos_int) * 0.0001f + static_cast<float>(motor_zero_offset_);
    motor_spd_ = static_cast<float>(spd_int) * 0.0001f;
    motor_current_ = static_cast<float>(current_int) * 0.01f;
    motor_temperature_ = static_cast<float>(read_be_i16(&rx_frame.data[15])) * 0.01f;
    mos_temperature_ = static_cast<float>(read_be_i16(&rx_frame.data[17])) * 0.01f;
}

void RPMotorDriver::motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) {
    if (!f_p || !f_v || !f_kp || !f_kd || !f_t) {
        return;
    }
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    canfd_frame tx_frame{};
    tx_frame.can_id = RP_MULTI_MIT;  // 0x400
    tx_frame.len = 64;
    tx_frame.flags = CANFD_BRS;

    // Unconfigured motor slots must encode zero commands, not negative full scale.
    constexpr uint8_t neutral_mit[8] = {0x80, 0x00, 0x80, 0x08, 0x00, 0x00, 0x00, 0x00};
    for (uint8_t slot = 0; slot < kRPBatchMotorCount; ++slot) {
        std::copy(std::begin(neutral_mit), std::end(neutral_mit), &tx_frame.data[slot * 8]);
    }

    std::shared_lock<std::shared_mutex> lock(bus_registry_mutex_);
    auto it = bus_registry_.find(can_interface_);
    if (it != bus_registry_.end()) {
        for (RPMotorDriver* motor : it->second) {
            if (!motor || motor->motor_index_ >= kRPBatchMotorCount)
            {
                continue;
            }
            const uint8_t slot = motor->motor_index_;

            float p_f, v_f, kp_f, kd_f, t_f;
            uint16_t p, v, kp, kd, t;

            p_f = limit(f_p[slot] - static_cast<float>(motor->motor_zero_offset_),-motor->limit_param_.PosMax, motor->limit_param_.PosMax);
            v_f = limit(f_v[slot], -motor->limit_param_.SpdMax, motor->limit_param_.SpdMax);
            kp_f = limit(f_kp[slot], 0.0f, motor->limit_param_.OKpMax);
            kd_f = limit(f_kd[slot], 0.0f, motor->limit_param_.OKdMax);
            t_f = limit(f_t[slot], -motor->limit_param_.TauMax, motor->limit_param_.TauMax);

            kp = range_map(kp_f, 0.0f, motor->limit_param_.OKpMax, uint16_t(0), uint16_t(0x0FFF));
            kd = range_map(kd_f, 0.0f, motor->limit_param_.OKdMax, uint16_t(0), uint16_t(0x0FFF));
            // Round to the nearest code so zero maps to 32768 / 2048 / 2048.
            p = static_cast<uint16_t>(std::lround(range_map(p_f, -motor->limit_param_.PosMax, motor->limit_param_.PosMax, 0.0, 65535.0)));
            v = static_cast<uint16_t>(std::lround(range_map(v_f, -motor->limit_param_.SpdMax, motor->limit_param_.SpdMax, 0.0, 4095.0)));
            t = static_cast<uint16_t>(std::lround(range_map(t_f, -motor->limit_param_.TauMax, motor->limit_param_.TauMax, 0.0, 4095.0)));

            uint64_t packed = 0;
            packed |= (static_cast<uint64_t>(p) << 48);   // pos: 16 bits
            packed |= (static_cast<uint64_t>(v) << 36);   // spd: 12 bits
            packed |= (static_cast<uint64_t>(t) << 24);   // torque: 12 bits
            packed |= (static_cast<uint64_t>(kp) << 12);  // kp: 12 bits
            packed |= static_cast<uint64_t>(kd);          // kd: 12 bits

            uint8_t* base = &tx_frame.data[slot * 8];
            base[0] = (packed >> 56) & 0xFF;
            base[1] = (packed >> 48) & 0xFF;
            base[2] = (packed >> 40) & 0xFF;
            base[3] = (packed >> 32) & 0xFF;
            base[4] = (packed >> 24) & 0xFF;
            base[5] = (packed >> 16) & 0xFF;
            base[6] = (packed >> 8) & 0xFF;
            base[7] = packed & 0xFF;
        }
        for (RPMotorDriver* motor : it->second) {
            if (motor && motor->motor_index_ < kRPBatchMotorCount) {
                motor->response_count_++;
            }
        }
    }

    canfd_->transmit(tx_frame);
}

void RPMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) {
    if (comm_type_ == CommType::CANFD) {
        uint8_t mode_value;
        switch (motor_control_mode) {
            case MIT:    mode_value = RP_MODE_MIT;    break;
            default:     return;
        }
        canfd_frame tx_frame{};
        tx_frame.can_id = RP_MODE_BASE + device_id_;
        tx_frame.len = 0x01;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = mode_value;

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
    Timer::sleep_for(normal_sleep_time); 
    motor_control_mode_ = motor_control_mode;
}

void RPMotorDriver::set_motor_id(uint8_t old_id, uint8_t new_id) {
    if (old_id < 1 || old_id > kRPBatchMotorCount ||
        new_id < 1 || new_id > kRPBatchMotorCount) {
        logger_->error("Invalid ID range: old={}, new={}", old_id, new_id);
        return;
    }
    if (old_id == new_id) {
        return;
    }

    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = RP_BROADCAST;  // 0x7FF
        tx_frame.len = 0x06;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = (old_id >> 8) & 0xFF;   // Old ID high byte
        tx_frame.data[1] = old_id & 0xFF;          // Old ID low byte
        tx_frame.data[2] = RP_CONTROLLER_TO_MOTOR;
        tx_frame.data[3] = RP_BC_ID_CHANGE;     // Command: 0x11
        tx_frame.data[4] = (new_id >> 8) & 0xFF;   // New ID high byte
        tx_frame.data[5] = new_id & 0xFF;          // New ID low byte

        canfd_->transmit(tx_frame);
        {
            response_count_++;
        }

        Timer::sleep_for(setup_sleep_time);
    }
}

void RPMotorDriver::reset_motor_id() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = RP_BROADCAST;  // 0x7FF
        tx_frame.len = 0x06;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0xFF;            
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = RP_CONTROLLER_TO_MOTOR;
        tx_frame.data[3] = RP_BC_ID_RESET;   // Command: 0x12
        tx_frame.data[4] = (0x01 >> 8) & 0xFF;                // Reset target ID high byte
        tx_frame.data[5] = 0x01 & 0xFF;                // Reset target ID low byte (-> ID 1)

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}
