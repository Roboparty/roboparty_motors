// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 Luo1imasi

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "motor_driver.hpp"
#include "drivers/dm/dm_motor_driver.hpp"
#include "drivers/evo/evo_motor_driver.hpp"
#include "drivers/lro/lro_motor_driver.hpp"
#include "drivers/rp/rp_motor_driver.hpp"
#include "drivers/xyn/xyn_motor_driver.hpp"

namespace py = pybind11;

PYBIND11_MODULE(motors_py, m) {
    m.doc() = "Motor Driver Python SDK"; 

    py::enum_<MotorDriver::MotorControlMode_e>(m, "MotorControlMode")
        .value("NONE", MotorDriver::MotorControlMode_e::NONE)
        .value("MIT", MotorDriver::MotorControlMode_e::MIT)
        .value("POS", MotorDriver::MotorControlMode_e::POS)
        .value("SPD", MotorDriver::MotorControlMode_e::SPD)
        .export_values();

    py::class_<MotorDriver, std::shared_ptr<MotorDriver>>(m, "MotorDriver")
        .def_static("create_motor", &MotorDriver::create_motor,
            py::arg("motor_id"),
            py::arg("interface_type"),
            py::arg("interface"),
            py::arg("motor_type"),
            py::arg("motor_model"),
            py::arg("master_id_offset") = 0,
            py::arg("motor_zero_offset") = 0.0)
        .def("lock_motor", &MotorDriver::lock_motor)
        .def("unlock_motor", &MotorDriver::unlock_motor)
        .def("init_motor", &MotorDriver::init_motor)
        .def("deinit_motor", &MotorDriver::deinit_motor)
        .def("set_motor_zero", &MotorDriver::set_motor_zero)
        .def("write_motor_flash", &MotorDriver::write_motor_flash)
        .def("get_motor_param", &MotorDriver::get_motor_param)
        .def("motor_pos_cmd", &MotorDriver::motor_pos_cmd, py::arg("pos"), py::arg("spd"), py::arg("ignore_limit") = false)
        .def("motor_spd_cmd", &MotorDriver::motor_spd_cmd)
        .def("motor_mit_cmd", static_cast<void (MotorDriver::*)(float, float, float, float, float)>(&MotorDriver::motor_mit_cmd))
        .def("motors_mit_cmd", [](MotorDriver& self, std::vector<float> f_p, std::vector<float> f_v,
                                        std::vector<float> f_kp, std::vector<float> f_kd, std::vector<float> f_t) {
            float p_arr[8] = {}, v_arr[8] = {}, kp_arr[8] = {}, kd_arr[8] = {}, t_arr[8] = {};
            for (size_t i = 0; i < 8 && i < f_p.size(); i++) {
                p_arr[i] = f_p[i];
                v_arr[i] = i < f_v.size() ? f_v[i] : 0.0f;
                kp_arr[i] = i < f_kp.size() ? f_kp[i] : 0.0f;
                kd_arr[i] = i < f_kd.size() ? f_kd[i] : 0.0f;
                t_arr[i] = i < f_t.size() ? f_t[i] : 0.0f;
            }
            self.motor_mit_cmd(p_arr, v_arr, kp_arr, kd_arr, t_arr);
        }, py::arg("f_p"), py::arg("f_v"), py::arg("f_kp"), py::arg("f_kd"), py::arg("f_t"))
        .def("set_motor_control_mode", &MotorDriver::set_motor_control_mode)
        .def("get_response_count", &MotorDriver::get_response_count)
        .def("refresh_motor_status", &MotorDriver::refresh_motor_status)
        .def("reset_motor_id", &MotorDriver::reset_motor_id)
        .def("get_motor_id", &MotorDriver::get_motor_id)
        .def("get_motor_control_mode", &MotorDriver::get_motor_control_mode)
        .def("get_error_id", &MotorDriver::get_error_id)
        .def("get_motor_pos", &MotorDriver::get_motor_pos)
        .def("get_motor_spd", &MotorDriver::get_motor_spd)
        .def("get_motor_current", &MotorDriver::get_motor_current)
        .def("get_motor_temperature", &MotorDriver::get_motor_temperature)
        .def("clear_motor_error", &MotorDriver::clear_motor_error)
        .def("get_can_name", &MotorDriver::get_can_name);

}
