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

uint8_t hardware::SparkDriveInterface::read(void* data)
{
    std::lock_guard<std::mutex> lock(read_mtx);
    SparkDriveStatus* status_data = static_cast<SparkDriveStatus*>(data);

    if (status_data == nullptr || !can_transport_.isOpen())
    {
        return -1;
    }
    status_data->fl_status = front_left_.getStatus2();
    status_data->fr_status = front_right_.getStatus2();
    status_data->ml_status = mid_left_.getStatus2();
    status_data->mr_status = mid_right_.getStatus2();
    status_data->bl_status = back_left_.getStatus2();
    status_data->br_status = back_right_.getStatus2();
    return 0;
}

uint8_t hardware::SparkDriveInterface::write(void* data)
{
    SparkDriveData* drive_data = static_cast<SparkDriveData*>(data);

    if (drive_data == nullptr || !can_transport_.isOpen())
    {
        return -1;
    }

    front_left_.setVelocity(drive_data->fl_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    front_right_.setVelocity(drive_data->fr_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    mid_left_.setVelocity(drive_data->ml_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    mid_right_.setVelocity(drive_data->mr_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    back_left_.setVelocity(drive_data->bl_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    back_right_.setVelocity(drive_data->br_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

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