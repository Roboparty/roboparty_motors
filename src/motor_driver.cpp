// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2025-2026 Luo1imasi

#include "motor_driver.hpp"

#include "dm_motor_driver.hpp"
#include "evo_motor_driver.hpp"
#include "lro_motor_driver.hpp"
#include "rp_motor_driver.hpp"
#include "xyn_motor_driver.hpp"

MotorDriver::MotorDriver() {
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_st>());
    logger_ = setup_logger(sinks, "motors");
}
std::shared_ptr<MotorDriver> MotorDriver::create_motor(uint16_t motor_id, const std::string& interface_type, const std::string& interface,
                                                      const std::string& motor_type, int motor_model, uint16_t master_id_offset, double motor_zero_offset) {
    if (motor_type == "DM") {
        return std::make_shared<DmMotorDriver>(motor_id, interface_type, interface, master_id_offset,
                                               static_cast<DM_Motor_Model>(motor_model), motor_zero_offset);
    } else if (motor_type == "EVO") { 
        return std::make_shared<EvoMotorDriver>(motor_id, interface_type, interface,
                                                static_cast<EVO_Motor_Model>(motor_model), motor_zero_offset);
    } else if (motor_type == "LRO") {
    return std::make_shared<LroMotorDriver>(motor_id, interface_type, interface,
                                            static_cast<LRO_Motor_Model>(motor_model), motor_zero_offset);
    } else if (motor_type == "RP") {
        return std::make_shared<RPMotorDriver>(motor_id, interface_type, interface,
                                               static_cast<RP_Motor_Model>(motor_model), motor_zero_offset);
    } else if (motor_type == "XYN") {
        return std::make_shared<XynMotorDriver>(motor_id, interface_type, interface,
                                                static_cast<XYN_Motor_Model>(motor_model), motor_zero_offset);
    } else {
        throw std::runtime_error("Motor type not supported");
    }
}
