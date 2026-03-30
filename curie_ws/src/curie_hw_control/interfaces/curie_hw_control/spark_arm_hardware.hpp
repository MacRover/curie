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

typedef enum : uint8_t
{
    BASE = (SPARK_ARM<< 4) | 1,
    SHOULDER = (SPARK_ARM << 4) | 2,
    ELBOW = (SPARK_ARM << 4) | 3,
    WRIST_ROLL = (SPARK_ARM << 4) | 4,
    WRIST_PITCH = (SPARK_ARM << 4) | 5,
} SparkDriveID;

typedef struct
{
    float base_position;
    float shoulder_position;
    float elbow_position;
    float wrist_roll_position;
    float wrist_pitch_position;
} SparkArmCommand;

typedef struct
{
    Status5 base_status;
    Status5 shoulder_status;
    Status5 elbow_status;
    Status5 wrist_roll_status;
    Status5 wrist_pitch_status;
} SparkArmStatus;

namespace hardware
{
    class SparkArmInterface : public HardwareInterface
    {
    public:
        SparkArmInterface();

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