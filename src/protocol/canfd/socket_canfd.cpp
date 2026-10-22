// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty
// Copyright (C) 2026 Luo1imasi

/**
 * @file
 * This file implements functions to receive
 * and transmit CAN-FD frames via SocketCAN.
 */

#include "socket_canfd.hpp"

#include <cerrno>
#include <cstdlib>

#include <spdlog/sinks/stdout_color_sinks.h>

// MotorsCANFD static members
std::shared_ptr<MotorsCANFD> MotorsCANFD::get(const std::string& interface, const std::string& backend) {
    ensure_logger();
    if (backend == "socketcan") {
        return MotorsSocketCANFD::get(interface);
    }
    throw std::runtime_error("Unknown CANFD backend: " + backend);
}

// MotorsSocketCANFD static members
std::unordered_map<std::string, std::shared_ptr<MotorsSocketCANFD>> MotorsSocketCANFD::instances_;

std::shared_ptr<MotorsSocketCANFD> MotorsSocketCANFD::get(const std::string& interface) {
    if (instances_.find(interface) == instances_.end()) instances_[interface] = createInstance(interface);
    return instances_[interface];
}

MotorsSocketCANFD::MotorsSocketCANFD(const std::string& interface)
    : interface_(interface), sockfd_(FD_INIT_FD), receiving_(false), tx_queue_(FD_TX_QUEUE_SIZE) {
    open(interface);
}

MotorsSocketCANFD::~MotorsSocketCANFD() { this->close(); }

void MotorsSocketCANFD::open(const std::string& interface) {
    sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd_ == FD_INIT_FD) {
        logger_->error("Failed to create CAN socket");
        throw std::runtime_error("Failed to create CAN socket");
    }

    int enable_canfd = 1;
    if (setsockopt(sockfd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) != 0) {
        logger_->error("Failed to enable CAN-FD support");
        this->close();
        throw std::runtime_error("Failed to enable CAN-FD support");
    }

    int bufsize = 1024 * 1024;  // 1MB
    setsockopt(sockfd_, SOL_SOCKET, SO_SNDBUF, &bufsize, sizeof(bufsize));

    strncpy(if_request_.ifr_name, interface.c_str(), IFNAMSIZ);
    if (ioctl(sockfd_, SIOCGIFINDEX, &if_request_) == -1) {
        logger_->error("Unable to detect CAN interface {}", interface);

        this->close();
        throw std::runtime_error("Unable to detect CAN interface " + interface);
    }

    // Bind the socket to the network interface
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = if_request_.ifr_ifindex;
    int rc = ::bind(sockfd_, reinterpret_cast<struct sockaddr *>(&addr_), sizeof(addr_));
    if (rc == -1) {
        logger_->error("Failed to bind socket to network interface {}", interface);
        this->close();
        throw std::runtime_error("Failed to bind socket to network interface " + interface);
    }

    int flags = fcntl(sockfd_, F_GETFL, 0);
    if (flags == -1) {
        logger_->error("Failed to get socket flags");
        this->close();
        throw std::runtime_error("Failed to get socket flags");
    }
    if (fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) == -1) {
        logger_->error("Failed to set socket to non-blocking");
        this->close();
        throw std::runtime_error("Failed to set socket to non-blocking");
    }

    int cpu_id = 6;
    if (const char* value = std::getenv("ROBOT_CAN_CPU"); value && *value) {
        char* end = nullptr;
        const long parsed = std::strtol(value, &end, 10);
        if (*value >= '0' && *value <= '9' && *end == '\0' && parsed < CPU_SETSIZE) {
            cpu_id = static_cast<int>(parsed);
        } else {
            logger_->warn("Invalid ROBOT_CAN_CPU='{}'; using CPU{}", value, cpu_id);
        }
    }
    receiving_ = true;
    receiver_thread_ = std::thread([this, cpu_id]() {
        struct sched_param sp{}; sp.sched_priority = 48;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
            logger_->error("Failed to set realtime priority for CANFD RX thread");
        }

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_id, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
            logger_->error("Failed to bind CANFD RX thread to Core {}", cpu_id);
        }
        pthread_setname_np(pthread_self(), "motor_canfd_rx");

        fd_set descriptors;
        int maxfd = sockfd_;
        struct timeval timeout;
        canfd_frame rx_frame;

        while (receiving_) {
            FD_ZERO(&descriptors);
            FD_SET(sockfd_, &descriptors);

            timeout.tv_sec = FD_TIMEOUT_SEC;
            timeout.tv_usec = FD_TIMEOUT_USEC;

            int sel_ret = ::select(maxfd + 1, &descriptors, NULL, NULL, &timeout);
            if (sel_ret < 0) {
                if (errno == EINTR) continue;
                logger_->error("CANFD select error: {}", strerror(errno));
                break;
            }
            if (sel_ret == 1) {
                while (true){
                    int len = ::read(sockfd_, &rx_frame, CANFD_MTU);
                    if (len < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            break; 
                        }
                        logger_->warn("CANFD read error: {}", strerror(errno));
                        break;
                    }
                    if (len == 0){
                        break;
                    }
                    CanFdCbkFunc callback_to_run;
                    {
                        std::lock_guard<std::mutex> lock(canfd_callback_mutex_);
                        CanFdCbkId key = key_extractor_(rx_frame);
                        auto it = canfd_callback_list_.find(key);
                        if (it != canfd_callback_list_.end()) {
                            callback_to_run = it->second;
                        }
                    }
                    if (callback_to_run) {
                        callback_to_run(rx_frame);
                    }
                }
            }
        }
    });

    sender_thread_ = std::thread([this, cpu_id]() {
        struct sched_param sp{}; sp.sched_priority = 48;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
            logger_->error("Failed to set realtime priority for CANFD TX thread");
        }

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_id, &cpuset);
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
            logger_->error("Failed to bind CANFD TX thread to Core {}", cpu_id);
        }
        pthread_setname_np(pthread_self(), "motor_canfd_tx");

        canfd_frame tx_frame;
        int count = 0;
        while (receiving_) {
            {
                std::unique_lock<std::mutex> lock(tx_mutex_);
                tx_cv_.wait(lock, [this]() { return !tx_queue_.empty() || !receiving_; });
                if (!receiving_) break;
                if (!tx_queue_.pop(tx_frame)) continue;
            }
            ssize_t written;
            while ((written = ::write(sockfd_, &tx_frame, sizeof(canfd_frame))) < 0 &&
                   count < FD_MAX_RETRY_COUNT) {
                count += 1;
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }
            if (written < 0) {
                logger_->error("Failed to transmit CAN-FD frame");
            } else if (send_sleep_us_ > 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(send_sleep_us_));
            }
            count = 0;
        }
    });
}

void MotorsSocketCANFD::close() {
    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        receiving_ = false;
    }
    tx_cv_.notify_all();
    if (receiver_thread_.joinable()) receiver_thread_.join();
    if (sender_thread_.joinable()) sender_thread_.join();

    if (sockfd_ != FD_INIT_FD) {
        if (::close(sockfd_) < 0) {
            if (logger_) logger_->warn("Failed to close socket {}: {}", interface_, strerror(errno));
        } else {
            if (logger_) logger_->info("CAN-FD interface {} closed successfully.", interface_);
        }
    }
    sockfd_ = FD_INIT_FD;
}

void MotorsSocketCANFD::transmit(const canfd_frame &frame) {
    bool queued = false;
    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        if (!receiving_) {
            logger_->error("Unable to transmit: Socket not open");
            return;
        }
        queued = tx_queue_.bounded_push(frame);
    }
    if (!queued) {
        logger_->error("CAN-FD transmit queue on {} is full; dropping frame 0x{:X}",
                       interface_, frame.can_id);
        return;
    }
    tx_cv_.notify_one();
}

void MotorsSocketCANFD::add_canfd_callback(const CanFdCbkFunc& callback, const CanFdCbkId id) {
    std::lock_guard<std::mutex> lock(canfd_callback_mutex_);
    canfd_callback_list_[id] = callback;
}

void MotorsSocketCANFD::remove_canfd_callback(CanFdCbkId id) {
    std::lock_guard<std::mutex> lock(canfd_callback_mutex_);
    canfd_callback_list_.erase(id);
}

void MotorsSocketCANFD::clear_canfd_callbacks() {
    std::lock_guard<std::mutex> lock(canfd_callback_mutex_);
    canfd_callback_list_.clear();
}

void MotorsSocketCANFD::set_canfd_key_extractor(CanFdCbkKeyExtractor extractor) {
    std::lock_guard<std::mutex> lock(canfd_callback_mutex_);
    key_extractor_ = std::move(extractor);
}
