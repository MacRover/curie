#include "moveit_testing.hpp"

PoseSubscriberMover::PoseSubscriberMover() : Node(
    "moveit_testing",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
    string_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/target_string", 10,
        std::bind(&PoseSubscriberMover::string_callback, this, _1));

    key_pose_client_ = this->create_client<keyboard_interfaces::srv::KeyPose>("key_pose");

    RCLCPP_INFO(this->get_logger(), "[DEBUG] PoseSubscriberMover constructed. Subscribed to /target_string, created key_pose client.");
}

void PoseSubscriberMover::init_moveit()
{
    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_moveit() starting");

    auto moveit_node = std::make_shared<rclcpp::Node>("moveit_internal");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        moveit_node, PLANNING_GROUP);
    moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    moveit_executor_->add_node(moveit_node);
    moveit_spin_thread_ = std::thread([this]() {
        RCLCPP_INFO(this->get_logger(), "[DEBUG] moveit_internal executor spin thread started");
        moveit_executor_->spin();
        RCLCPP_INFO(this->get_logger(), "[DEBUG] moveit_internal executor spin thread exiting");
    });

    RCLCPP_INFO(this->get_logger(), "[DEBUG] Calling move_group_->getCurrentPose() to establish home_pose_");
    home_pose_ = move_group_->getCurrentPose();

    RCLCPP_INFO(this->get_logger(), "Current pose: x=%.3f y=%.3f z=%.3f",
        home_pose_.pose.position.x,
        home_pose_.pose.position.y,
        home_pose_.pose.position.z);

    // DEBUG: also report orientation of the captured home pose, since
    // plan_and_execute() inherits this orientation implicitly via home_pose_
    RCLCPP_INFO(this->get_logger(),
        "[DEBUG] home_pose_ orientation: x=%.3f y=%.3f z=%.3f w=%.3f, frame_id=%s",
        home_pose_.pose.orientation.x,
        home_pose_.pose.orientation.y,
        home_pose_.pose.orientation.z,
        home_pose_.pose.orientation.w,
        home_pose_.header.frame_id.c_str());

    RCLCPP_INFO(this->get_logger(), "[DEBUG] init_moveit() complete");
}

geometry_msgs::msg::PoseStamped PoseSubscriberMover::get_key_pose(const std::string & key)
{
    RCLCPP_INFO(this->get_logger(), "[DEBUG] get_key_pose('%s') called, waiting for service...", key.c_str());

    if (!key_pose_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "KeyPose service not available");
        return geometry_msgs::msg::PoseStamped();
    }

    RCLCPP_INFO(this->get_logger(), "[DEBUG] key_pose service is available, sending request for '%s'", key.c_str());

    auto request = std::make_shared<keyboard_interfaces::srv::KeyPose::Request>();
    request->key = key;

    auto future = key_pose_client_->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "[DEBUG] Request sent for '%s', now blocking on future.wait()", key.c_str());
    future.wait();
    RCLCPP_INFO(this->get_logger(), "[DEBUG] future.wait() returned for '%s'", key.c_str());

    auto response = future.get();
    if (!response->success) {
        RCLCPP_WARN(this->get_logger(), "Invalid key requested: '%s'", key.c_str());
        return geometry_msgs::msg::PoseStamped();
    }

    // DEBUG: report the pose actually received from the key_pose service
    RCLCPP_INFO(this->get_logger(),
        "[DEBUG] get_key_pose('%s') received pose: frame_id=%s x=%.4f y=%.4f z=%.4f",
        key.c_str(),
        response->pose.header.frame_id.c_str(),
        response->pose.pose.position.x,
        response->pose.pose.position.y,
        response->pose.pose.position.z);

    return response->pose;
}
// Static extrinsic calibration: camera pose relative to end-effector frame.
// EDIT these numbers to match your actual mount.
static Eigen::Isometry3d T_eef_cam()
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    // Translation: camera origin relative to eef origin, in eef-local axes (meters)
    T.translation() << 0.02, 0.0, 0.03;

    // Rotation: camera optical frame relative to eef frame (radians)
    double yaw = 0.0, pitch = 0.0, roll = 0.0;
    Eigen::Matrix3d R = (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
    T.linear() = R;

    return T;
}

geometry_msgs::msg::PoseStamped PoseSubscriberMover::camera_pose_to_base( // This is where I need to do some changes 
    const geometry_msgs::msg::Pose & pose_in_cam)
{
    Eigen::Isometry3d T_cam_target;
    tf2::fromMsg(pose_in_cam, T_cam_target);

    Eigen::Isometry3d T_base_eef;
    tf2::fromMsg(home_pose_.pose, T_base_eef);

    Eigen::Isometry3d T_base_target = T_base_eef * T_eef_cam() * T_cam_target;

    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = home_pose_.header.frame_id;
    result.pose = tf2::toMsg(T_base_target); // This is returning the base target this is what needs to change specifically. 
    return result;
}

void PoseSubscriberMover::string_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "[DEBUG] string_callback received: '%s'", msg->data.c_str());

    if (!move_group_) {
        RCLCPP_WARN(this->get_logger(), "[DEBUG] string_callback: move_group_ not yet initialized, ignoring message");
        return;
    }
    if (is_typing_.exchange(true)) {
        RCLCPP_WARN(this->get_logger(), "Already typing, ignoring new string");
        return;
    }

    std::string text = msg->data; // this is the input string 

    /*
        Like doing something like 
        std::string pose_list; 
        auto current_offset; 
        for(i = 0; i < len(text); i++)
        {
            auto next_pose = get_key_pose(text[i]); // Use next_pose so you only have to call get_key_pose() once per character

            if(i = 0); pose_list[i] = next_pose; // Get a list of poses and store them to the pose_list.
            else{
                pose_list[i] = next_pose + current_offset
            }
            current_offset -= next_pose; // This is the offset of current position. E.g. your first move leaves you at 0,2,4. Take this as new 0,0,0. Your next move wants you to go to 5,-2, 3. How do you get there? You move 5, -4, -1. Therefor, you get to your position of 5, -2, 3. I BELIEVE THIS IS HOW IT WORKS BUT IDK FOR SURE YET. 
        } 


    */
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Starting typing_thread_ for text of length %zu", text.size());

    typing_thread_ = std::thread([this, text]() {
        for (char c : text) {
            std::string key(1, c);
            RCLCPP_INFO(this->get_logger(), "Requesting key: %s", key.c_str());

            auto pose = get_key_pose(key); // STEP 1: 
            if (pose.header.frame_id.empty()) {
                RCLCPP_WARN(this->get_logger(), "Skipping key '%s'", key.c_str());
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "[DEBUG] Got valid pose for key '%s', calling plan_and_execute()", key.c_str());

            // So if we want to change this so that it does not immediately do it, then we should actually just store the poses in a list, then make subsequent plan_and_execute calls. 
            
            // Check that there is no blocking and that characters will just come through w/out delay
            if (plan_and_execute(pose)) { // STEP 2: 
                RCLCPP_INFO(this->get_logger(), "[DEBUG] plan_and_execute() returned true for key '%s', calling go_home()", key.c_str());
                go_home(); // STEP 3: 
            } else {
                RCLCPP_INFO(this->get_logger(), "[DEBUG] plan_and_execute() returned false for key '%s', skipping go_home()", key.c_str());
            }
        }
        RCLCPP_INFO(this->get_logger(), "[DEBUG] typing_thread_ finished processing text, clearing is_typing_");
        is_typing_.store(false);
    });
    typing_thread_.detach();
}
bool PoseSubscriberMover::plan_and_execute(const geometry_msgs::msg::PoseStamped& pose_msg)
{
    geometry_msgs::msg::PoseStamped target_pose = camera_pose_to_base(pose_msg.pose);

    move_group_->setPositionTarget(
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z);
    move_group_->setGoalTolerance(0.01);
    move_group_->setPlanningTime(10.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group_->execute(my_plan);
        return true;
    }
    return false;
}
void PoseSubscriberMover::go_home()
{
    RCLCPP_INFO(this->get_logger(), "[DEBUG] go_home() called, targeting joint-space home {0,0,0,0}");

    std::vector<double> home_joints = {0.0, 0.0, 0.0, 0.0};
    move_group_->setJointValueTarget(home_joints);
    move_group_->setPlanningTime(10.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    RCLCPP_INFO(this->get_logger(), "[DEBUG] go_home(): calling move_group_->plan()...");
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(this->get_logger(), "[DEBUG] go_home(): plan %s", success ? "SUCCEEDED" : "FAILED");
    if (success) {
        RCLCPP_INFO(this->get_logger(), "[DEBUG] go_home(): calling move_group_->execute()...");
        auto exec_result = move_group_->execute(my_plan);
        RCLCPP_INFO(this->get_logger(),
            "[DEBUG] go_home(): execute() returned MoveItErrorCode val=%d",
            exec_result.val);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSubscriberMover>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() {
        RCLCPP_INFO(rclcpp::get_logger("moveit_testing_main"), "[DEBUG] main executor spin thread started");
        executor.spin();
        RCLCPP_INFO(rclcpp::get_logger("moveit_testing_main"), "[DEBUG] main executor spin thread exiting");
    });
    node->init_moveit();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}