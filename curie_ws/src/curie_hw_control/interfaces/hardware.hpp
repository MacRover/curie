#pragma once
#include <cstdint>
#include <memory>

namespace hardware 
{
    class HardwareInterface
    {
    public:
        /**
         * @brief Initialize hardware interface. Anything that needs to be setup should be done here
         */
        virtual uint8_t initialize() = 0;
        /**
         * @brief Shutdown hardware interface. Anything that needs to be cleaned up before exiting any 
         * process should be done here
         */
        virtual uint8_t shutdown() = 0;
        /**
         * @brief read data from hardware
         * @return 0 if read was successful, negative error code otherwise (<= 0)
         */
        virtual uint8_t read(void* data) = 0;
        /**
         * @brief write data to hardware
         * @return 0 if write was successful, negative error code otherwise (<= 0)
         */
        virtual uint8_t write(void* data) = 0;
        /**
         * @brief main loop for hardware interface
         */
        virtual void run() = 0;
    };
}