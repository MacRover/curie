#pragma once
#include <cstdint>

#include "hardware.hpp"

namespace hardware
{
    class SparkInterface : public HardwareInterface
    {
    public:
        uint8_t initialize() override
        {
            return 0;
        }

        uint8_t shutdown() override
        {
            return 0;
        }

        uint8_t read() override
        {
            return 0;
        }

        uint8_t write() override
        {
            return 0;
        }
    private:
    };
}