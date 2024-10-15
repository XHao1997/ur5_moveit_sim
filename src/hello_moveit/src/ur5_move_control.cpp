#include "hello_moveit/ur5_move_control.hpp"
#include "ur5_move_control.hpp"

UR5MoveControl::UR5MoveControl(const std::string& group_name, std::shared_ptr<rclcpp::Node> node)
:Node("ur5_") {
    // Constructor code
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    

}

// Destructor
UR5MoveControl::~UR5MoveControl() {
    // Cleanup code: free resources, close connections, etc.
    // Example: delete pointers, stop ROS nodes, or close MoveIt interfaces
}

void UR5MoveControl::get_target_pose_list(std::vector<std::vector<double>> & target_pose)(){

};

// Method to plan a motion
bool UR5MoveControl::moveToPose(const geometry_msgs::msg::Pose& target_pose) {
    // Add code for motion planning using MoveIt
}

// Method to execute a motion
bool UR5MoveControl::moveToNamedPose(const std::string& pose_name) {
    // Add code to execute the planned motion
}

geometry_msgs::msg::Pose UR5MoveControl::getCurrentPose(){

}

void UR5MoveControl::go_to_home_position(void){
  moveit::planning_interface::MoveGroupInterfacePtr move_group = PLANNING_GROUP;
  std::string group_state = "home";
  move_group->setNamedTarget(group_state);
  move_group->move();



}
void UR5MoveControl::getCurrentPose()
{
}

bool UR5MoveControl::moveToJointAngles(const std::vector<double>& joint_values){


}

// Stop movement
void UR5MoveControl::stop(){



}

 bool UR5MoveControl::executePlan(moveit::planning_interface::MoveGroupInterface::Plan& plan){
  moveit::planning_interface::MoveGroupInterfacePtr move_group 
  move_group->setNamedTarget(group_state);
  move_group->move();
 }

 void UR5MoveControl::str_list_2_double_list(const std::vector<std::string> & str_list, 
                                            std::vector<std::vector<double>> & double_list){
    double_list.clear();
    // parse the string
    // each element in the list is a string
    // each string is a list of doubles, with ',' as delimiter
    for (auto & pose_str : str_list){
        std::vector<double> pose;
        std::stringstream ss(pose_str);
        std::string token;
        while (std::getline(ss, token, ',')){
            pose.push_back(std::stod(token));
        }
        double_list.push_back(pose);
    }
}