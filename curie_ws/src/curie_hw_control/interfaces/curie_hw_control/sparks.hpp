#pragma once
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
} SparkArmID;

typedef enum : uint8_t
{
    FRONT_LEFT = (SPARK_DRIVETRAIN << 4) | 1,
    FRONT_RIGHT = (SPARK_DRIVETRAIN << 4) | 2,
    MID_LEFT = (SPARK_DRIVETRAIN << 4) | 3,
    MID_RIGHT = (SPARK_DRIVETRAIN << 4) | 4,
    BACK_LEFT = (SPARK_DRIVETRAIN << 4) | 5,
    BACK_RIGHT = (SPARK_DRIVETRAIN << 4) | 6
} SparkDriveID;

typedef union __spark_command
{  
    struct __DriveCommand
    {
        float fl_velocity;
        float fr_velocity;
        float ml_velocity;
        float mr_velocity;
        float bl_velocity;
        float br_velocity;
    } drive;
    struct __ArmCommand
    {
        float base_position;
        float shoulder_position;
        float elbow_position;
        float wrist_roll_position;
        float wrist_pitch_position;

        float base_velocity;
        float shoulder_velocity;
        float elbow_velocity;
        float wrist_roll_velocity;
        float wrist_pitch_velocity;
    } arm;
} SparkCommand;

typedef union __spark_status
{
    struct __DriveStatus
    {
        Status2 fl_status;
        Status2 fr_status;
        Status2 ml_status;
        Status2 mr_status;
        Status2 bl_status;
        Status2 br_status;
    } drive;
    struct __ArmStatus
    {
        Status5 base_status;
        Status5 shoulder_status;
        Status5 elbow_status;
        Status5 wrist_roll_status;
        Status5 wrist_pitch_status;
    } arm;
} SparkStatus;
