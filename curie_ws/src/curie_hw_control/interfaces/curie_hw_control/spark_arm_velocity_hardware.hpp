#pragma once
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <chrono>
#include <optional>

#include "sparks.hpp"
#include "hardware.hpp"

namespace hardware
{
    class SparkArmVelocityInterface : public HardwareInterface
    {
    public:
        SparkArmVelocityInterface();

        int8_t initialize(void* config) override;

        int8_t shutdown() override;

        int8_t read(void* data) override;

        int8_t write(void* data) override;
        
        void run() override;

    private:
        std::mutex read_mtx;
        spark_mmrt::can::SocketCanTransport can_transport_;
        SparkMax base_;
        SparkMax shoulder_;
        SparkMax elbow_;
        SparkMax wrist_roll_;
        SparkMax wrist_pitch_;
    };
}