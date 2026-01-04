#pragma once
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <chrono>
#include <optional>

#include "hardware.hpp"
#include "spark_mmrt/can/SocketCanTransport.hpp"
#include "spark_mmrt/device/SparkMax.hpp"

#define CAN_INTERFACE "can0"
// #define CAN_INTERFACE "vcan0" // For testing

typedef enum : uint8_t
{
    FRONT_LEFT = 1,
    FRONT_RIGHT = 2,
    MID_LEFT = 3,
    MID_RIGHT = 4,
    BACK_LEFT = 5,
    BACK_RIGHT = 6
} SparkDriveID;

typedef struct
{
    float fl_velocity;
    float fr_velocity;
    float ml_velocity;
    float mr_velocity;
    float bl_velocity;
    float br_velocity;
} SparkDriveData;

typedef struct
{
    Status2 fl_status;
    Status2 fr_status;
    Status2 ml_status;
    Status2 mr_status;
    Status2 bl_status;
    Status2 br_status;
} SparkDriveStatus;

namespace hardware
{
    class SparkDriveInterface : public HardwareInterface
    {
    public:
        SparkDriveInterface();

        uint8_t initialize() override;

        uint8_t shutdown() override;

        uint8_t read(void* data) override;

        uint8_t write(void* data) override;
        
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