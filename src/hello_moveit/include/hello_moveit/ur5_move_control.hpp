#ifndef UR5_MOVE_CONTROL_HPP
#define UR5_MOVE_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

class UR5MoveControl
{
public:
    // Constructor
    UR5MoveControl(const std::string& group_name, std::shared_ptr<rclcpp::Node> node);

    // Destructor
    ~UR5MoveControl();

    // Function to move to a target pose
    bool moveToPose(const geometry_msgs::msg::Pose& target_pose);

    // Function to move to a named pose (e.g., "home" or predefined poses)
    bool moveToNamedPose(const std::string& pose_name);

    // Function to set joint angles directly
    bool moveToJointAngles(const std::vector<double>& joint_values);

    // Stop movement
    void stop();

    void get_target_pose_list(std::vector<std::vector<double>> & target_pose);

    // Get current pose of the end effector
    geometry_msgs::msg::Pose getCurrentPose();
    
    void go_to_home_position(void);
private:
    // Node handle and logger
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Logger logger_;

    // MoveIt move group interface for controlling UR5
    moveit::planning_interface::MoveGroupInterface move_group_;
    static const std::string PLANNING_GROUP = "ur_manipulator";
    // Function to execute the motion plan
    bool executePlan(moveit::planning_interface::MoveGroupInterface::Plan& plan);
};

#endif // UR5_MOVE_CONTROL_HPP
