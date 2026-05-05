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
    class SparkDriveInterface : public HardwareInterface
    {
    public:
        SparkDriveInterface();

        int8_t initialize(void* config) override;

        int8_t shutdown() override;

        int8_t read(void* data) override;

        int8_t write(void* data) override;
        
        void run() override;

    private:
        std::mutex read_mtx;
        spark_mmrt::can::SocketCanTransport can_transport_;
        SparkMax front_left_;
        SparkMax front_right_;
        SparkMax mid_left_;
        SparkMax mid_right_;
        SparkMax back_left_;
        SparkMax back_right_;
    };
}