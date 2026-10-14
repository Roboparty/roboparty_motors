// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#include "xyn_motor_driver.hpp"

XYN_Limit_Param xyn_limit_param[XYN_Num_Of_Model] = {
    {12.5, 45.0, 40.0, 500.0, 5.0},   // XYN_5550
    {12.5, 45.0, 40.0, 500.0, 5.0},   // XYN_5757
    {12.5, 10.0, 50.0, 250.0, 50.0},  // XYN_6562
    {12.5, 10.0, 50.0, 250.0, 50.0},  // XYN_8462
    {12.5, 10.0, 50.0, 250.0, 50.0},  // XYN_10062
};

XynMotorDriver::XynMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                               XYN_Motor_Model motor_model, double motor_zero_offset)
    : MotorDriver(), motor_model_(motor_model) {
    if (interface_type != "canfd" && interface_type != "ethercanfd" && interface_type != "ethercat") {
        throw std::runtime_error("XYN driver only supports CAN-FD and Ethercat interface");
    }
    motor_id_ = motor_id;
    limit_param_ = xyn_limit_param[motor_model_];
    can_interface_ = can_interface;
    motor_zero_offset_ = motor_zero_offset;
    device_id_ = motor_id & 0x7F;
    motor_index_ = (device_id_ > 0 && device_id_ <= 7) ? (device_id_ - 1) : 0;

    if (interface_type == "canfd" || interface_type == "ethercanfd") {
        comm_type_ = CommType::CANFD;
        canfd_ = MotorsSocketCANFD::get(can_interface);

        CanFdCbkFunc canfd_callback = std::bind(&XynMotorDriver::canfd_rx_cbk, this, std::placeholders::_1);
        canfd_->set_canfd_key_extractor([](const canfd_frame& frame) -> CanFdCbkId {
            uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
            return static_cast<CanFdCbkId>((raw_id >> 12) & 0x7F);
        });
        canfd_->add_canfd_callback(canfd_callback, static_cast<CanFdCbkId>(device_id_));
        std::lock_guard<std::shared_mutex> lock(bus_registry_mutex_);
        bus_registry_[can_interface].push_back(this);
    } else if (interface_type == "ethercat") {
        comm_type_ = CommType::ETHERCAT;
        throw std::runtime_error("XYN driver does not support EtherCAT interface yet");
    }
}

XynMotorDriver::~XynMotorDriver() { 
    if (comm_type_ == CommType::CANFD) {
        canfd_->remove_canfd_callback(static_cast<CanFdCbkId>(device_id_));
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

void XynMotorDriver::lock_motor() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_ENABLE & 0x3FF);
        tx_frame.len = 0x01;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0x01;

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void XynMotorDriver::unlock_motor() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_ENABLE & 0x3FF);
        tx_frame.len = 0x01;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0x00;

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

uint8_t XynMotorDriver::init_motor() {
    if (comm_type_ == CommType::CANFD) {
        XynMotorDriver::unlock_motor();
        Timer::sleep_for(normal_sleep_time);
        XynMotorDriver::set_motor_control_mode(MIT);
        Timer::sleep_for(normal_sleep_time);
        XynMotorDriver::lock_motor();
        Timer::sleep_for(normal_sleep_time);
        XynMotorDriver::refresh_motor_status();
        Timer::sleep_for(normal_sleep_time);
    } else if (comm_type_ == CommType::ETHERCAT) {
        throw std::runtime_error("XYN driver does not support EtherCAT interface yet");
    }

    switch (error_id_) {
        case XYN_OVER_VOLTAGE:
            return XYN_OVER_VOLTAGE;
        case XYN_OVER_CURRENT:
            return XYN_OVER_CURRENT;
        case XYN_MOTOR_OVER_TEMP:
            return XYN_MOTOR_OVER_TEMP;
        case XYN_BOARD_OVER_TEMP:
            return XYN_BOARD_OVER_TEMP;
        case XYN_UNDER_VOLTAGE:
            return XYN_UNDER_VOLTAGE;
        case XYN_ENCODER_FAULT:
            return XYN_ENCODER_FAULT;
        case XYN_COMM_FAULT:
            return XYN_COMM_FAULT;
        default:
            return error_id_;
    }
    return 0;
}

void XynMotorDriver::deinit_motor() {
    XynMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
}

bool XynMotorDriver::write_motor_flash() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};

        // save basic params
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_SAVE_BASIC & 0x3FF);
        tx_frame.len = 0;
        tx_frame.flags = CANFD_BRS;

        canfd_->transmit(tx_frame);
        {
            response_count_++;
        }
        Timer::sleep_for(setup_sleep_time);

        // save control params
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_SAVE_CTRL & 0x3FF);
        tx_frame.len = 0;
        tx_frame.flags = CANFD_BRS;

        canfd_->transmit(tx_frame);
        {
            response_count_++;
        }
        Timer::sleep_for(setup_sleep_time);

        // save limit params
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_SAVE_LIMIT & 0x3FF);
        tx_frame.len = 0;
        tx_frame.flags = CANFD_BRS;

        canfd_->transmit(tx_frame);
        {
            response_count_++;
        }
        Timer::sleep_for(setup_sleep_time);
    }

    return true;
}

void XynMotorDriver::set_motor_zero() {
    XynMotorDriver::set_motor_zero_xyn();
    Timer::sleep_for(setup_sleep_time);
}

void XynMotorDriver::canfd_rx_cbk(const canfd_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    if (rx_frame.len < 9 * sizeof(float)) return;
    uint16_t msg_id = static_cast<uint16_t>(rx_frame.can_id & 0x3FF);
    if (msg_id != XYN_MSG_MONITOR) return;

    const float* vals = reinterpret_cast<const float*>(rx_frame.data);
    motor_pos_ = vals[1] * static_cast<float>(M_PI) / 180.0f +
                 static_cast<float>(motor_zero_offset_);
    motor_spd_ = vals[3] * 2.0f * static_cast<float>(M_PI) / 60.0f;
    motor_current_ = vals[5];
    motor_temperature_ = vals[6];
}

// void XynMotorDriver::ethercat_rx_cbk(const ethercat_frame& rx_frame) {}

void XynMotorDriver::get_motor_param(uint8_t param_cmd) {
    uint16_t msg_id;
    switch (param_cmd) {
        case 0x01: msg_id = XYN_MSG_MIT_LPF_GET; break;
        case 0x02: msg_id = XYN_MSG_MIT_KPKD_GET; break;
        default:   msg_id = XYN_MSG_READ_LIMITS; break;
    }
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (msg_id & 0x3FF);
        tx_frame.len = 0;
        tx_frame.flags = CANFD_BRS;

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void XynMotorDriver::motor_pos_cmd(float pos, float spd, bool ignore_limit) {
    if (motor_control_mode_ != POS) {
        set_motor_control_mode(POS);
        return;
    }
    float pos_deg = (pos - static_cast<float>(motor_zero_offset_)) * 180.0f / static_cast<float>(M_PI);
    float spd_rpm = spd * 60.0f / (2.0f * static_cast<float>(M_PI));
    union32_t rv_type_convert;
    rv_type_convert.f = spd_rpm;

    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame;
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_SPD_LIMIT & 0x3FF);
        tx_frame.len = 0x04;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = rv_type_convert.buf[0];
        tx_frame.data[1] = rv_type_convert.buf[1];
        tx_frame.data[2] = rv_type_convert.buf[2];
        tx_frame.data[3] = rv_type_convert.buf[3];

        canfd_->transmit(tx_frame);
    }
    { 
        response_count_++; 
    }

    rv_type_convert.f = pos_deg;
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame;
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_POS_TARGET & 0x3FF);
        tx_frame.len = 0x04;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = rv_type_convert.buf[0];
        tx_frame.data[1] = rv_type_convert.buf[1];
        tx_frame.data[2] = rv_type_convert.buf[2];
        tx_frame.data[3] = rv_type_convert.buf[3];

        canfd_->transmit(tx_frame);
    }
    { 
        response_count_++; 
    }
}

void XynMotorDriver::motor_spd_cmd(float spd) {
    if (motor_control_mode_ != SPD) {
        set_motor_control_mode(SPD);
        return;
    }
    float spd_rpm = spd * 60.0f / (2.0f * static_cast<float>(M_PI));
    union32_t rv_type_convert;
    rv_type_convert.f = spd_rpm;

    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame;
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_SPD_TARGET & 0x3FF);
        tx_frame.len = 0x04;
        tx_frame.flags = CANFD_BRS;
        
        tx_frame.data[0] = rv_type_convert.buf[0];
        tx_frame.data[1] = rv_type_convert.buf[1];
        tx_frame.data[2] = rv_type_convert.buf[2];
        tx_frame.data[3] = rv_type_convert.buf[3];

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++; 
    }
}

void XynMotorDriver::motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }

    f_p -= static_cast<float>(motor_zero_offset_);
    f_v  = std::abs(f_v);
    f_kp = f_kp;
    f_kd = f_kd;
    f_t  = f_t;

    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame;
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_MIT_STD & 0x3FF);
        tx_frame.len = 20;
        tx_frame.flags = CANFD_BRS;

        memcpy(&tx_frame.data[0],  &f_p, 4);
        memcpy(&tx_frame.data[4],  &f_v, 4);
        memcpy(&tx_frame.data[8],  &f_t, 4);
        memcpy(&tx_frame.data[12], &f_kp, 4);
        memcpy(&tx_frame.data[16], &f_kd, 4);

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++; 
    }
}

void XynMotorDriver::motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) {
    if (!f_p || !f_v || !f_kp || !f_kd || !f_t) {
        return;
    }
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }

    canfd_frame tx_frame;
    tx_frame.can_id = CAN_EFF_FLAG | (XYN_MSG_MIT_MULTI & 0x3FF);
    tx_frame.len = 64;
    tx_frame.flags = CANFD_BRS;

    for (uint8_t slot = 0; slot < 8; ++slot) {
        uint8_t* base = &tx_frame.data[slot * 8];
        memset(base, 0, 8);
    }

    std::shared_lock<std::shared_mutex> lock(bus_registry_mutex_);
    auto it = bus_registry_.find(can_interface_);
    if (it != bus_registry_.end()) {
        for (XynMotorDriver* motor : it->second) {
            if (!motor || motor->motor_index_ >= 8) {
                continue;
            }
            const uint8_t slot = motor->motor_index_;

            float p_f, v_f, kp_f, kd_f, t_f;
            uint16_t p, v, kp, kd, t;

            p_f  = limit(f_p[slot] - static_cast<float>(motor->motor_zero_offset_), -motor->limit_param_.PosMax, motor->limit_param_.PosMax);
            v_f  = limit(f_v[slot], -motor->limit_param_.SpdMax, motor->limit_param_.SpdMax);
            kp_f = limit(f_kp[slot], 0.0f, motor->limit_param_.OKpMax);
            kd_f = limit(f_kd[slot], 0.0f, motor->limit_param_.OKdMax);
            t_f  = limit(f_t[slot], -motor->limit_param_.TauMax, motor->limit_param_.TauMax);

            kp = range_map(kp_f, 0.0f, motor->limit_param_.OKpMax, uint16_t(0), uint16_t(0x0FFF));
            kd = range_map(kd_f, 0.0f, motor->limit_param_.OKdMax, uint16_t(0), uint16_t(0x01FF));
            p  = range_map(p_f, -motor->limit_param_.PosMax, motor->limit_param_.PosMax, uint16_t(0), uint16_t(0xFFFF));
            v  = range_map(v_f, -motor->limit_param_.SpdMax, motor->limit_param_.SpdMax, uint16_t(0), uint16_t(0x0FFF));
            t  = range_map(t_f, -motor->limit_param_.TauMax, motor->limit_param_.TauMax, uint16_t(0), uint16_t(0x0FFF));

            // 8-byte 0x8080 compact encoding per SDK section 4.4
            uint8_t* base = &tx_frame.data[slot * 8];
            base[0] = (uint8_t)((kp >> 7) & 0x1F);
            base[1] = (uint8_t)(((kp & 0x7F) << 1) | ((kd >> 8) & 0x01));
            base[2] = (uint8_t)(kd & 0xFF);
            base[3] = (uint8_t)(p >> 8);
            base[4] = (uint8_t)(p & 0xFF);
            base[5] = (uint8_t)(v >> 4);
            base[6] = (uint8_t)(((v & 0x0F) << 4) | ((t >> 8) & 0x0F));
            base[7] = (uint8_t)(t & 0xFF);
        }
        for (XynMotorDriver* motor : it->second) {
            if (motor && motor->motor_index_ < 8) {
                motor->response_count_++;
            }
        }
    }

    canfd_->transmit(tx_frame);
}

void XynMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) {
    uint8_t xyn_mode;
    switch (motor_control_mode) {
        case POS:
            xyn_mode = XYN_MODE_POSITION;
            break;
        case SPD:
            xyn_mode = XYN_MODE_SPEED;
            break;
        case MIT:
            xyn_mode = XYN_MODE_MIT;
            break;
        default:
            xyn_mode = XYN_MODE_POSITION;
            break;
    }

    if (comm_type_ == CommType::CANFD) {
        // stop motor before mode switch
        canfd_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_START & 0x3FF);
        tx_frame.len = 0x01;
        tx_frame.flags = CANFD_BRS;
        tx_frame.data[0] = 0x00;

        canfd_->transmit(tx_frame);
        Timer::sleep_for(normal_sleep_time);

        // set new mode
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_MODE & 0x3FF);
        tx_frame.len = 0x01;
        tx_frame.flags = CANFD_BRS;
        tx_frame.data[0] = xyn_mode;

        canfd_->transmit(tx_frame);
        Timer::sleep_for(normal_sleep_time);

        // restart motor
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_START & 0x3FF);
        tx_frame.len = 0x01;
        tx_frame.flags = CANFD_BRS;
        tx_frame.data[0] = 0x01;

        canfd_->transmit(tx_frame);
        Timer::sleep_for(normal_sleep_time);
    }

    motor_control_mode_ = motor_control_mode;
}

void XynMotorDriver::set_motor_id(uint8_t old_id, uint8_t new_id) {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(old_id & 0x7F) << 12) | (XYN_MSG_DEVICE_ID & 0x3FF);
        tx_frame.len = 0x01;
        tx_frame.flags = CANFD_BRS;
        tx_frame.data[0] = new_id & 0x7F;

        canfd_->transmit(tx_frame);
        {
            response_count_++;
        }
        Timer::sleep_for(setup_sleep_time);
    }
}

void XynMotorDriver::reset_motor_id() {
    XynMotorDriver::set_motor_id(motor_id_ & 0x7F, 0x01);
}

void XynMotorDriver::set_motor_zero_xyn() {
    if (comm_type_ == CommType::CANFD) {
        {
            canfd_frame tx_frame{};
            tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_SET_ZERO & 0x3FF);
            tx_frame.len = 0;
            tx_frame.flags = CANFD_BRS;

            canfd_->transmit(tx_frame);
        }
        Timer::sleep_for(normal_sleep_time);
        {
            canfd_frame tx_frame{};
            tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_SAVE_ZERO & 0x3FF);
            tx_frame.len = 0;
            tx_frame.flags = CANFD_BRS;

            canfd_->transmit(tx_frame);
        }
    }
}

void XynMotorDriver::clear_motor_error_xyn() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_ERROR & 0x3FF);
        tx_frame.len = 0x01;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0x00;

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void XynMotorDriver::write_register_xyn(uint8_t index, int32_t value) {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (index & 0x3FF);
        tx_frame.len = 0x04;
        tx_frame.flags = CANFD_BRS;

        uint8_t* b = (uint8_t*)&value;
        tx_frame.data[0] = b[0];
        tx_frame.data[1] = b[1];
        tx_frame.data[2] = b[2];
        tx_frame.data[3] = b[3];

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void XynMotorDriver::save_register_xyn() {
    write_motor_flash();
}

void XynMotorDriver::refresh_motor_status() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_MONITOR & 0x3FF);
        tx_frame.len = 0;
        tx_frame.flags = CANFD_BRS;

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void XynMotorDriver::clear_motor_error() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(device_id_) << 12) | (XYN_MSG_ERROR & 0x3FF);
        tx_frame.len = 0x01;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0x00;

        canfd_->transmit(tx_frame);
        {
            response_count_++;
        }
    }
}
