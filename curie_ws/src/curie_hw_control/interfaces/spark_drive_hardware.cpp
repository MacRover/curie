#include "spark_drive_hardware.hpp"

hardware::SparkDriveInterface::SparkDriveInterface() : 
    front_left_(can_transport_, FRONT_LEFT),
    front_right_(can_transport_, FRONT_RIGHT),
    mid_left_(can_transport_, MID_LEFT),
    mid_right_(can_transport_, MID_RIGHT),
    back_left_(can_transport_, BACK_LEFT),
    back_right_(can_transport_, BACK_RIGHT)
{

}

uint8_t hardware::SparkDriveInterface::initialize()
{
    can_transport_.open(CAN_INTERFACE);
    if (!can_transport_.isOpen())
    {
        return -1;
    }
    return 0;
}

uint8_t hardware::SparkDriveInterface::shutdown()
{
    can_transport_.close();
    return 0;
}

uint8_t hardware::SparkDriveInterface::read(std::shared_ptr<void> data)
{
    return 0;
}

uint8_t hardware::SparkDriveInterface::write(std::shared_ptr<void> data)
{
    return 0;
}

void hardware::SparkDriveInterface::run()
{
    while (can_transport_.isOpen())
    {
        auto f = can_transport_.recv(std::chrono::microseconds{20000});
        if (!f)
        {
            continue;
        }

        auto &frame = *f; // currently in std::optional<canframe> need the canFrame part 
        read_mtx.lock();
        switch (frame.arbId & 0x3F)
        {
            case FRONT_LEFT:
                front_left_.processFrame(frame);
                break;
            case FRONT_RIGHT:
                front_right_.processFrame(frame);
                break;
            case MID_LEFT:
                mid_left_.processFrame(frame);
                break;
            case MID_RIGHT:
                mid_right_.processFrame(frame);
                break;
            case BACK_LEFT:
                back_left_.processFrame(frame);
                break;
            case BACK_RIGHT:
                back_right_.processFrame(frame);
                break;
            default:
                break;
        }
        read_mtx.unlock();
    }
}