// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty
// Copyright (C) 2026 Luo1imasi

#include "lro_motor_driver.hpp"

LRO_Limit_Param lro_limit_param[LRO_Num_Of_Motor] = {
    {12.5, 21.0, 36.0, 500.0, 50.0},  // LRO_PJ2_55_5550
    {12.5, 21.89, 35.3, 500.0, 50.0},  // LRO_PJ3_60_6562
    {12.5, 16.13, 56.5, 500.0, 50.0},  // LRO_PJ3_75_8462
    {12.5, 14.24, 141.7, 500.0, 50.0}   // LRO_PJ3_97_10062
};

LroMotorDriver::LroMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                               LRO_Motor_Model motor_model, double motor_zero_offset)
    : MotorDriver(), motor_model_(motor_model) {
    if (interface_type != "canfd" && interface_type != "ethercanfd" && interface_type != "ethercat") {
        throw std::runtime_error("LRO driver only supports CAN-FD and Ethercat interfaces");
    }
    motor_id_ = motor_id;
    limit_param_ = lro_limit_param[motor_model_];
    can_interface_ = can_interface;
    motor_zero_offset_ = motor_zero_offset;

    if (interface_type == "canfd" || interface_type == "ethercanfd") {
        comm_type_ = CommType::CANFD;
        motor_index_ = (motor_id_ > 0 && motor_id_ <= 8) ? (motor_id_ - 1) : 0;
        canfd_ = MotorsCANFD::get(can_interface);

        CanFdCbkFunc canfd_callback = std::bind(&LroMotorDriver::canfd_rx_cbk, this, std::placeholders::_1);
        canfd_->add_canfd_callback(canfd_callback, motor_id_);
        std::lock_guard<std::shared_mutex> lock(bus_registry_mutex_);
        bus_registry_[can_interface].push_back(this);
    } else if (interface_type == "ethercat") {
        comm_type_ = CommType::ETHERCAT;
        throw std::runtime_error("LRO driver does not support EtherCAT interface yet");
    }
}

LroMotorDriver::~LroMotorDriver() {
    if (comm_type_ == CommType::CANFD) {
        canfd_->remove_canfd_callback(motor_id_);
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

void LroMotorDriver::lock_motor() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x7FF;
        tx_frame.len = 0x04;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
        tx_frame.data[1] = motor_id_ & 0xFF;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = LRO_CMD_ENABLE;
        
        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void LroMotorDriver::unlock_motor() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x7FF;
        tx_frame.len = 0x04;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
        tx_frame.data[1] = motor_id_ & 0xFF;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = LRO_CMD_DISABLE;

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

uint8_t LroMotorDriver::init_motor() {
    // send disable command to enter read mode
    LroMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
    LroMotorDriver::set_motor_control_mode(MIT);
    Timer::sleep_for(normal_sleep_time);
    // send enable command to enter contorl mode
    LroMotorDriver::lock_motor();
    Timer::sleep_for(normal_sleep_time);
    LroMotorDriver::refresh_motor_status();
    Timer::sleep_for(normal_sleep_time);
    switch (error_id_) {
        case LROError::LRO_MOTOR_OVERHEAT:
            return LROError::LRO_MOTOR_OVERHEAT;
        case LROError::LRO_OVER_CURRENT:
            return LROError::LRO_OVER_CURRENT;
        case LROError::LRO_UNDER_VOLTAGE:
            return LROError::LRO_UNDER_VOLTAGE;
        case LROError::LRO_ENCODER_ERROR:
            return LROError::LRO_ENCODER_ERROR;
        case LROError::LRO_BRAKE_OVERVOLT:
            return LROError::LRO_BRAKE_OVERVOLT;
        case LROError::LRO_DRV_ERROR:
            return LROError::LRO_DRV_ERROR;
        default:
            return error_id_;
    }
    return error_id_;
}

void LroMotorDriver::deinit_motor() {
    LroMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
}

bool LroMotorDriver::write_motor_flash() {
    return true;
}

void LroMotorDriver::set_motor_zero() {
    LroMotorDriver::set_motor_zero_lro();
    Timer::sleep_for(setup_sleep_time);
    LroMotorDriver::unlock_motor();
}

void LroMotorDriver::canfd_rx_cbk(const canfd_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    if (rx_frame.len < 0x08) return;
    uint16_t pos_int = 0;
    uint16_t spd_int = 0;
    uint16_t t_int = 0;
    uint8_t fb_type = (rx_frame.data[0] >> 5) & 0x07;
    error_id_ = rx_frame.data[0] & 0x1F;
    if (error_id_ > 0) {
        if (logger_) {
            logger_->error("can_interface: {0}\tmotor_id: {1}\terror_id: 0x{2:x}", can_interface_, motor_id_, static_cast<uint32_t>(error_id_));
        }
    }
    pos_int = (static_cast<uint16_t>(rx_frame.data[1]) << 8) | rx_frame.data[2];
    spd_int= (static_cast<uint16_t>(rx_frame.data[3]) << 4) | ((rx_frame.data[4] >> 4) & 0x0F);
    t_int = (static_cast<uint16_t>(rx_frame.data[4] & 0x0F) << 8) | rx_frame.data[5];
    motor_pos_ = 
        range_map(pos_int, uint16_t(0), bitmax<uint16_t>(16), -limit_param_.PosMax, limit_param_.PosMax) + static_cast<float>(motor_zero_offset_);
    motor_spd_ = 
        range_map(spd_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.SpdMax, limit_param_.SpdMax);
    motor_current_ = 
        range_map(t_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.TauMax, limit_param_.TauMax);
    mos_temperature_ = static_cast<float>(static_cast<int>(rx_frame.data[7]) - 25);
    motor_temperature_ = static_cast<float>(static_cast<int>(rx_frame.data[6]) - 25); // Temperature: value - 25 = actual temperature
}

// void LroMotorDriver::ethercat_rx_cbk(const ethercat_frame& rx_frame) {}

void LroMotorDriver::get_motor_param(uint8_t param_cmd) {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_; // ID: Corresponds to the individual motor ID
        tx_frame.len = 0x02; // Query commands typically require only 2 bytes
        tx_frame.flags = CANFD_BRS;

        // Byte 0: Mode bits [Mode(3 bits) | Reserved(5 bits)]
        // Query mode (Mode 0x07) shifted left by 5 bits results in 0xE0
        tx_frame.data[0] = (uint8_t)(LRO_MODE_QUERY << 5);
        tx_frame.data[1] = param_cmd; //// Query (mode 0x07) — request param/status

        canfd_->transmit(tx_frame);
    }
}

void LroMotorDriver::motor_pos_cmd(float pos, float spd, bool ignore_limit) {
    if (motor_control_mode_ != POS) {
        set_motor_control_mode(POS);
        return;
    }
    float pos_deg = (pos - static_cast<float>(motor_zero_offset_)) * 180.0f / static_cast<float>(M_PI);
    float spd_rpm = std::abs(spd) * 60.0f / (2.0f * static_cast<float>(M_PI));
    uint16_t spd_val = static_cast<uint16_t>(limit(spd_rpm * 10.0f, 0.0f, 32767.0f));
    uint16_t cur_limit = 4095;
    uint8_t ack = 1;
    union32_t rv_type_convert;
    rv_type_convert.f = pos_deg;

    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        uint64_t packed = 0;
        packed |= (static_cast<uint64_t>(LRO_MODE_POS & 0x07)) << 61;
        packed |= (static_cast<uint64_t>(rv_type_convert.buf[3]) << 53) | (static_cast<uint64_t>(rv_type_convert.buf[2]) << 45)
                 | (static_cast<uint64_t>(rv_type_convert.buf[1]) << 37) | (static_cast<uint64_t>(rv_type_convert.buf[0]) << 29);
        packed |= (static_cast<uint64_t>(spd_val & 0x7FFF)) << 14;
        packed |= (static_cast<uint64_t>(cur_limit & 0x0FFF)) << 2;
        packed |= (static_cast<uint64_t>(ack & 0x03));

        for (int i = 0; i < 8; ++i) {
            tx_frame.data[i] = (packed >> (56 - i * 8)) & 0xFF;
        }

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void LroMotorDriver::motor_spd_cmd(float spd) {
    if (motor_control_mode_ != SPD) {
        set_motor_control_mode(SPD);
        return;
    }
    float spd_rpm = spd * 60.0f / (2.0f * static_cast<float>(M_PI));
    uint16_t cur_limit = 65535;
    uint8_t ack = 1;
    union32_t rv_type_convert;
    rv_type_convert.f = spd_rpm;

    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x07;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = ((LRO_MODE_SPD & 0x07) << 5) | (ack & 0x03);
        tx_frame.data[1] = rv_type_convert.buf[3];
        tx_frame.data[2] = rv_type_convert.buf[2];
        tx_frame.data[3] = rv_type_convert.buf[1];
        tx_frame.data[4] = rv_type_convert.buf[0];
        tx_frame.data[5] = (cur_limit >> 8) & 0xFF;
        tx_frame.data[6] = cur_limit & 0xFF;

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

// Transmit MIT-mDme control(hybrid) package. Called in canTask.
// ------------------------------------------------------------------
// MIT impedance control (mode 0x00)
// Protocol bit layout (8 bytes, standard CAN, ID = motor_id:
//   motor_mode  uint3   (bits 63-61)
//   KP          uint12  (bits 60-49)
//   KD          uint9   (bits 48-40)
//   pos         uint16  (bits 39-24)
//   spd         uint12  (bits 23-12)
//   torque      uint12  (bits 11-0)
// ------------------------------------------------------------------
void LroMotorDriver::motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    uint16_t p, v, kp, kd, t;

    f_p -= static_cast<float>(motor_zero_offset_);
    f_p = limit(f_p, -limit_param_.PosMax, limit_param_.PosMax);
    f_v = limit(f_v, -limit_param_.SpdMax, limit_param_.SpdMax);
    f_kp = limit(f_kp, 0.0f, limit_param_.OKpMax);
    f_kd = limit(f_kd, 0.0f, limit_param_.OKdMax);
    f_t = limit(f_t, -limit_param_.TauMax, limit_param_.TauMax);

    p = range_map(f_p, -limit_param_.PosMax, limit_param_.PosMax, uint16_t(0), bitmax<uint16_t>(16));
    v = range_map(f_v, -limit_param_.SpdMax, limit_param_.SpdMax, uint16_t(0), bitmax<uint16_t>(12));
    kp = range_map(f_kp, 0.0f, limit_param_.OKpMax, uint16_t(0), bitmax<uint16_t>(12));
    kd = range_map(f_kd, 0.0f, limit_param_.OKdMax, uint16_t(0), bitmax<uint16_t>(9));
    t = range_map(f_t, -limit_param_.TauMax, limit_param_.TauMax, uint16_t(0), bitmax<uint16_t>(12));

    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        uint64_t packed = 0;
        packed |= (static_cast<uint64_t>(LRO_MODE_MIT & 0x07)) << 61;
        packed |= (static_cast<uint64_t>(kp & 0x0FFF)) << 49;
        packed |= (static_cast<uint64_t>(kd & 0x01FF)) << 40;
        packed |= (static_cast<uint64_t>(p & 0xFFFF)) << 24;
        packed |= (static_cast<uint64_t>(v & 0x0FFF)) << 12;
        packed |= static_cast<uint64_t>(t & 0x0FFF);

        tx_frame.data[0] = (packed >> 56) & 0xFF;
        tx_frame.data[1] = (packed >> 48) & 0xFF;
        tx_frame.data[2] = (packed >> 40) & 0xFF;
        tx_frame.data[3] = (packed >> 32) & 0xFF;
        tx_frame.data[4] = (packed >> 24) & 0xFF;
        tx_frame.data[5] = (packed >> 16) & 0xFF;
        tx_frame.data[6] = (packed >> 8) & 0xFF;
        tx_frame.data[7] = packed & 0xFF;

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void LroMotorDriver::motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) {
    if (!f_p || !f_v || !f_kp || !f_kd || !f_t) {
        return;
    }
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    canfd_frame tx_frame;
    tx_frame.can_id = 0x8080 | CAN_EFF_FLAG;
    tx_frame.len = 64;
    tx_frame.flags = CANFD_BRS;

    for (uint8_t slot = 0; slot < 8; ++slot) {
        uint8_t* base = &tx_frame.data[slot * 8];
        memset(base, 0, 8);
        base[0] = (LRO_MODE_MIT & 0x07) << 5;
    }

    std::shared_lock<std::shared_mutex> lock(bus_registry_mutex_);
    auto it = bus_registry_.find(can_interface_);
    if (it != bus_registry_.end()) {
        for (LroMotorDriver* motor : it->second) {
            if (!motor || motor->motor_index_ >= 8) {
                continue;
            }
            const uint8_t slot = motor->motor_index_;

            float p_f, v_f, kp_f, kd_f, t_f;
            uint16_t p, v, kp, kd, t;

            p_f = limit(f_p[slot] - static_cast<float>(motor->motor_zero_offset_), -motor->limit_param_.PosMax, motor->limit_param_.PosMax);
            v_f = limit(f_v[slot], -motor->limit_param_.SpdMax, motor->limit_param_.SpdMax);
            kp_f = limit(f_kp[slot], 0.0f, motor->limit_param_.OKpMax);
            kd_f = limit(f_kd[slot], 0.0f, motor->limit_param_.OKdMax);
            t_f = limit(f_t[slot], -motor->limit_param_.TauMax, motor->limit_param_.TauMax);

            kp = range_map(kp_f, 0.0f, motor->limit_param_.OKpMax, uint16_t(0), uint16_t(0x0FFF));
            kd = range_map(kd_f, 0.0f, motor->limit_param_.OKdMax, uint16_t(0), uint16_t(0x01FF));
            p = range_map(p_f, -motor->limit_param_.PosMax, motor->limit_param_.PosMax, uint16_t(0), uint16_t(0xFFFF));
            v = range_map(v_f, -motor->limit_param_.SpdMax, motor->limit_param_.SpdMax, uint16_t(0), uint16_t(0x0FFF));
            t = range_map(t_f, -motor->limit_param_.TauMax, motor->limit_param_.TauMax, uint16_t(0), uint16_t(0x0FFF));

            uint64_t packed = 0;
            packed |= (static_cast<uint64_t>(LRO_MODE_MIT & 0x07)) << 61;
            packed |= (static_cast<uint64_t>(kp & 0x0FFF)) << 49;
            packed |= (static_cast<uint64_t>(kd & 0x01FF)) << 40;
            packed |= (static_cast<uint64_t>(p & 0xFFFF)) << 24;
            packed |= (static_cast<uint64_t>(v & 0x0FFF)) << 12;
            packed |= static_cast<uint64_t>(t & 0x0FFF);

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
        for (LroMotorDriver* motor : it->second) {
            if (motor && motor->motor_index_ < 8) {
                motor->response_count_++;
            }
        }
    }

    canfd_->transmit(tx_frame);
}

void LroMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) {
    if (motor_control_mode_ == motor_control_mode) {
        return;
    }
    motor_control_mode_ = motor_control_mode;
}

void LroMotorDriver::set_motor_id(uint8_t old_id, uint8_t new_id) {
    // Parameter Validation: LRO supports IDs from 1 to 0x7FF
    if (old_id < 1 || old_id > 0x7FF || new_id < 1 || new_id > 0x7FF) {
        logger_->error("Invalid ID range: old={}, new={}", old_id, new_id);
        return;
    }
    if (old_id == new_id) {
        return;
    }
    logger_->info("Changing Motor ID: {} -> {} (Interface: {})", old_id, new_id, can_interface_);
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x7FF; // Dedicated Configuration ID
        tx_frame.len = 0x06;
        tx_frame.flags = CANFD_BRS;

        // Populate Protocol Message
        tx_frame.data[0] = (old_id >> 8) & 0xFF;   // Old ID High Byte
        tx_frame.data[1] = old_id & 0xFF;          // Old ID Low Byte
        tx_frame.data[2] = 0x00;                   // Direction: Master -> Slave
        tx_frame.data[3] = LRO_CMD_SET_ID;         // Command: 0x04
        tx_frame.data[4] = (new_id >> 8) & 0xFF;   // New ID High Byte
        tx_frame.data[5] = new_id & 0xFF;          // New ID Low Byte

        canfd_->transmit(tx_frame);
        {
            response_count_++;
        }

        // Allow buffer time for Flash write persistence
        Timer::sleep_for(setup_sleep_time);
    }
}

void LroMotorDriver::reset_motor_id() {
    if (comm_type_ == CommType::CANFD) {
        // Reset all motors on the bus to ID 0x01
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x7FF;
        tx_frame.len = 0x06;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0x7F;
        tx_frame.data[1] = 0x7F;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = LRO_CMD_RESET_ID;
        tx_frame.data[4] = 0x7F;
        tx_frame.data[5] = 0x7F;

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void LroMotorDriver::set_motor_zero_lro() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x7FF;
        tx_frame.len = 0x04;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
        tx_frame.data[1] = motor_id_ & 0xFF;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = LRO_CMD_SET_ZERO;
        
        canfd_->transmit(tx_frame);
    }
}


void LroMotorDriver::clear_motor_error_lro() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x7FF;
        tx_frame.len = 0x04;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
        tx_frame.data[1] = motor_id_ & 0xFF;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = LRO_CMD_CLEAR_ERROR;

        canfd_->transmit(tx_frame);
        response_count_++;
    }
}

void LroMotorDriver::write_register_lro(uint8_t rid, float value) {
    return;
}

void LroMotorDriver::write_register_lro(uint8_t index, int32_t value) {
    return;
}

void LroMotorDriver::save_register_lro() {
    // LRO protocol has no dedicated "save to flash" command.
    return;
}

void LroMotorDriver::refresh_motor_status() {
    // Send a zero MIT command with type-1 feedback to get current status
    motor_mit_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void LroMotorDriver::clear_motor_error() {
    clear_motor_error_lro();
}
