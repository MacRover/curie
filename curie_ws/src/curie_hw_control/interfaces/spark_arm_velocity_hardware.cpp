#include "curie_hw_control/spark_arm_velocity_hardware.hpp"

hardware::SparkArmVelocityInterface::SparkArmVelocityInterface() : 
    base_(can_transport_, BASE),
    shoulder_(can_transport_, SHOULDER),
    elbow_(can_transport_, ELBOW),
    wrist_roll_(can_transport_, WRIST_ROLL),
    wrist_pitch_(can_transport_, WRIST_PITCH)
{

}

int8_t hardware::SparkArmVelocityInterface::initialize(void* config)
{
    bool isVCAN = (config != nullptr) ? *(static_cast<bool*>(config)) : false;
    can_transport_.open(isVCAN ? "vcan0" : CAN_INTERFACE, SPARK_ARM);
    if (!can_transport_.isOpen())
    {
        return -1;
    }
    return 0;
}

int8_t hardware::SparkArmVelocityInterface::shutdown()
{
    can_transport_.close();
    return 0;
}

int8_t hardware::SparkArmVelocityInterface::read(void* data)
{
    std::lock_guard<std::mutex> lock(read_mtx);
    SparkStatus* status_data = static_cast<SparkStatus*>(data);

    if (status_data == nullptr || !can_transport_.isOpen())
    {
        return -1;
    }
    status_data->arm.base_status = base_.getStatus5();
    status_data->arm.shoulder_status = shoulder_.getStatus5();
    status_data->arm.elbow_status = elbow_.getStatus5();
    status_data->arm.wrist_roll_status = wrist_roll_.getStatus5();
    status_data->arm.wrist_pitch_status = wrist_pitch_.getStatus5();
    return 0;
}

int8_t hardware::SparkArmVelocityInterface::write(void* data)
{
    SparkCommand* arm_cmd = static_cast<SparkCommand*>(data);

    if (arm_cmd == nullptr || !can_transport_.isOpen())
    {
        return -1;
    }

    base_.setVelocity(arm_cmd->arm.base_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    shoulder_.setVelocity(arm_cmd->arm.shoulder_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    elbow_.setVelocity(arm_cmd->arm.elbow_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    wrist_roll_.setVelocity(arm_cmd->arm.wrist_roll_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    wrist_pitch_.setVelocity(arm_cmd->arm.wrist_pitch_velocity);
    std::this_thread::sleep_for(std::chrono::microseconds(100));

    return 0;
}

void hardware::SparkArmVelocityInterface::run()
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
            case BASE:
                base_.processFrame(frame);
                break;
            case SHOULDER:
                shoulder_.processFrame(frame);
                break;
            case ELBOW:
                elbow_.processFrame(frame);
                break;
            case WRIST_ROLL:
                wrist_roll_.processFrame(frame);
                break;
            case WRIST_PITCH:
                wrist_pitch_.processFrame(frame);
                break;
            default:
                break;
        }
        read_mtx.unlock();
    }
}