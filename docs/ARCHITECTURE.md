# Curie ROS2 Architecture

For V3's ROS stack, similar nodes shall be grouped into individual modules, each holding a single responsibility of the system. The following diagram depicts the overall flow of information between each module, note that this is a simplified model and **not** an accurate representation of the entire system:

```mermaid
graph TB
    dc([curie_drive_controller])
    ac([curie_arm_controller])
    odom([curie_odometry])
    hw([curie_hw_control])
    base([curie_base])
    nav([curie_nav])
    moveit([curie_moveit])

    base --> dc
    base --> ac
    base --> nav
    base --> moveit
    dc --> hw
    ac --> hw
    hw --> odom
    odom --> nav
    odom --> moveit
    nav --> dc
    moveit --> ac
    hw ==> CAN
    CAN ==> hw
```
## Module Explanation

- `curie_base` - nodes running in the basestation, interfacing with the rover
- `curie_drive_controller` - responsible for controlling the drive and anything related to kinematics of the drivetrain
- `curie_arm_controller` - same as `curie_drive_controller` but for the arm
- `curie_hw_control` - provides an abstraction layer between hardware control and upper level modules, mainly interacting with CANbus
- `curie_odometry` - responsible for handling rover odometry (encoders, GPS, IMU, etc)
- `curie_moveit` - contains MoveIt launch configurations for the arm
- `curie_nav` - For bringing up navigation stack