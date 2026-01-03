#pragma once
#include <cstdint>

namespace hardware 
{
    class HardwareInterface
    {
    public:
        virtual ~HardwareInterface() = default;

        /**
         * @brief Initialize hardware interface. Anything that needs to be setup should be done here
         * @return 0 if initialization was successful, negative error code otherwise (<= 0)
         */
        virtual uint8_t initialize() = 0;
        /**
         * @brief Shutdown hardware interface. Anything that needs to be cleaned up before exiting any 
         * process should be done here
         * @return 0 if shutdown was successful, negative error code otherwise (<= 0)
         */
        virtual uint8_t shutdown() = 0;
        /**
         * @brief read data from hardware
         * @return 0 if read was successful, negative error code otherwise (<= 0)
         */
        virtual uint8_t read() = 0;
        /**
         * @brief write data to hardware
         * @return 0 if write was successful, negative error code otherwise (<= 0)
         */
        virtual uint8_t write() = 0;
    };
}