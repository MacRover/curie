#include "curie_hw_control/spark_drive_hardware.hpp"

hardware::SparkDriveInterface::SparkDriveInterface() : 
    front_left_(can_transport_, FRONT_LEFT),
    front_right_(can_transport_, FRONT_RIGHT),
    mid_left_(can_transport_, MID_LEFT),
    mid_right_(can_transport_, MID_RIGHT),
    back_left_(can_transport_, BACK_LEFT),
    back_right_(can_transport_, BACK_RIGHT)
{

}

int8_t hardware::SparkDriveInterface::initialize(void* config)
{
    bool isVCAN = (config != nullptr) ? *(static_cast<bool*>(config)) : false;
    can_transport_.open(isVCAN ? "vcan0" : CAN_INTERFACE, SPARK_DRIVETRAIN);
    if (!can_transport_.isOpen())
    {
        return -1;
    }
    return 0;
}

int8_t hardware::SparkDriveInterface::shutdown()
{
    can_transport_.close();
    return 0;
}

int8_t hardware::SparkDriveInterface::read(void* data)
{
    std::lock_guard<std::mutex> lock(read_mtx);
    SparkStatus* status_data = static_cast<SparkStatus*>(data);

    if (status_data == nullptr || !can_transport_.isOpen())
    {
        return -1;
    }
    status_data->drive.fl_status = front_left_.getStatus2();
    status_data->drive.fr_status = front_right_.getStatus2();
    status_data->drive.ml_status = mid_left_.getStatus2();
    status_data->drive.mr_status = mid_right_.getStatus2();
    status_data->drive.bl_status = back_left_.getStatus2();
    status_data->drive.br_status = back_right_.getStatus2();
    return 0;
}

int8_t hardware::SparkDriveInterface::write(void* data)
{
    SparkCommand* drive_cmd = static_cast<SparkCommand*>(data);

    if (drive_cmd == nullptr || !can_transport_.isOpen())
    {
        return -1;
    }

    front_left_.setVelocity(drive_cmd->drive.fl_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    front_right_.setVelocity(drive_cmd->drive.fr_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    mid_left_.setVelocity(drive_cmd->drive.ml_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    mid_right_.setVelocity(drive_cmd->drive.mr_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    back_left_.setVelocity(drive_cmd->drive.bl_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    back_right_.setVelocity(drive_cmd->drive.br_velocity);

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