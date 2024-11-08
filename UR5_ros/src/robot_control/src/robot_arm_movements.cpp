// Existing includes
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <interfaces/srv/arm_command.hpp>  // Include custom service header file
#include <cmath>

// Generate target pose message
auto generatePoseMsg(float x, float y, float z, float qx, float qy, float qz, float qw) {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    return msg;
}

moveit_msgs::msg::JointConstraint generateJointConstraint(const std::string& joint_name, double position, double tolerance_above, double tolerance_below) {
    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = joint_name;
    joint_constraint.position = position;
    joint_constraint.tolerance_above = tolerance_above;
    joint_constraint.tolerance_below = tolerance_below;
    joint_constraint.weight = 1.0;
    return joint_constraint;
}

// Set up collision objects in the planning scene
moveit_msgs::msg::CollisionObject generateCollisionObject(
    float sx, float sy, float sz, 
    float x, float y, float z, 
    const std::string& id, const std::string& frame_id) {
    
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = id;
    collision_object.header.frame_id = frame_id;
    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = sx;
    primitive.dimensions[primitive.BOX_Y] = sy;
    primitive.dimensions[primitive.BOX_Z] = sz;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0; 
    box_pose.position.x = x;
    box_pose.position.y = y;
    box_pose.position.z = z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
}

// Add predefined collision objects to the scene
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    const std::string frame_ID = "world";
    auto col_object_table = generateCollisionObject(2.4, 1.2, 0.04, 0.85, 0.25, -0.03, "table", frame_ID);
    auto col_object_backWall = generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.45, 0.5, "backWall", frame_ID);
    auto col_object_sideWall = generateCollisionObject(0.04, 1.2, 1.0, -0.45, 0.25, 0.5, "sideWall", frame_ID);

    planning_scene_interface.applyCollisionObjects({col_object_table, col_object_backWall, col_object_sideWall});
}

// Global variables
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface; 
std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;

// Execute and check success for a given pose
// bool planAndExecutePose(const geometry_msgs::msg::Pose& target_pose) {
//     move_group_interface->setPoseTarget(target_pose);

    

//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     bool success = static_cast<bool>(move_group_interface->plan(plan));
//     if (success) {
//         move_group_interface->execute(plan);
//     }
//     move_group_interface->stop();
//     move_group_interface->clearPoseTargets();
//     return success;
// }

// Move functions for predefined motions
// bool moveToShelf() {
//     geometry_msgs::msg::Pose shelf_pose = generatePoseMsg(0.5, 0.2, 0.3, 0, 1, 0, 0);
//     return planAndExecutePose(shelf_pose);
// }

// bool moveToInventory() {
//     geometry_msgs::msg::Pose inventory_pose = generatePoseMsg(0.6, 0.3, 0.3, 0, 1, 0, 0); // z goes 149mm lower than expected
//     return planAndExecutePose(inventory_pose);
// }

// // Move function for dynamic position based on request coordinates
// bool moveToBookSpine(float x, float y, float z) {
//     geometry_msgs::msg::Pose book_spine_pose = generatePoseMsg(x, y, z, 0, 1, 0, 0);
//     return planAndExecutePose(book_spine_pose);
// }

// Service callback function
void planAndExecuteCallback(const std::shared_ptr<interfaces::srv::ArmCommand::Request> request, std::shared_ptr<interfaces::srv::ArmCommand::Response> response) {

    tf2::Quaternion quaternion;
    quaternion.setRPY(request->roll, request->pitch, request->yaw);

    // Generate target pose
    geometry_msgs::msg::Pose target_pose = generatePoseMsg(
        request->x, request->y, request->z,
        quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()
    );

    move_group_interface->setPoseTarget(target_pose);

    moveit_msgs::msg::Constraints constraints;
    constraints.joint_constraints.push_back(generateJointConstraint("shoulder_pan_joint", 0.0       , M_PI/2, M_PI/2));
    constraints.joint_constraints.push_back(generateJointConstraint("elbow_joint"       , M_PI/2    , M_PI/2, M_PI/2));
    constraints.joint_constraints.push_back(generateJointConstraint("wrist_1_joint"     , -M_PI/2   , M_PI/2, M_PI/2));
    constraints.joint_constraints.push_back(generateJointConstraint("wrist_2_joint"     , -M_PI/2   , M_PI/2, M_PI/2));
    constraints.joint_constraints.push_back(generateJointConstraint("wrist_3_joint"     , 0.0       , M_PI  , M_PI));
    move_group_interface->setPathConstraints(constraints);


    moveit::planning_interface::MoveGroupInterface::Plan planMessage;
    bool success = static_cast<bool>(move_group_interface->plan(planMessage));

    if (success) {
        move_group_interface->execute(planMessage);
        response->success = true;
        // response->message = "Motion successfully executed.";
    } else {
        response->success = false;
        RCLCPP_WARN(rclcpp::get_logger("arm"), "Motion planning failed.");
        // response->message = "Motion planning failed.";
    }

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // //bool success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // bool success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);


    move_group_interface->clearPathConstraints();
    
    // if (request->command == "move_to_shelf") {
    //     response->success = moveToShelf();
    // } else if (request->command == "move_to_inventory") {
    //     response->success = moveToInventory();
    // } 
    // // else if (request->command == "move_to_book_spine") {
    // //     response->success = moveToBookSpine(request->x, request->y, request->z);
    // // } 
    // else {
    //     response->success = false;
    //     RCLCPP_WARN(rclcpp::get_logger("arm"), "Unknown command requested.");
    // }
}

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    // Create ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("arm", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    addCollisionObjects(*planning_scene_interface);
    // Initialize MoveGroupInterface after the node is created
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");
    move_group_interface->setPlanningTime(10.0);

    // if (!move_group_interface->getInterfaceDescription()) {
    //     RCLCPP_ERROR(node->get_logger(), "Unable to connect to MoveGroup. Is MoveIt running?");
    //     return 1;
    // }

    RCLCPP_INFO(node->get_logger(), "Connected to MoveGroup successfully.");

    // Create custom service
    auto plan_service = node->create_service<interfaces::srv::ArmCommand>("arm", &planAndExecuteCallback);

    RCLCPP_INFO(node->get_logger(), "Service 'arm' is ready.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

