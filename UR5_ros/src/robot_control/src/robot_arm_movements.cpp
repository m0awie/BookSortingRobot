// Existing includes
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include "interfaces/srv/arm_command.hpp"  // Include custom service header file

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

// Set up collision objects in the planning scene
moveit_msgs::msg::CollisionObject generateCollisionObject(
    float sx, float sy, float sz, 
    float x, float y, float z, 
    const std::string& id) {
    
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = id;
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
    auto col_object_table = generateCollisionObject(2.4, 1.2, 0.04, 0.85, 0.25, -0.03, "table");
    auto col_object_backWall = generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.45, 0.5, "backWall");
    auto col_object_sideWall = generateCollisionObject(0.04, 1.2, 1.0, -0.45, 0.25, 0.5, "sideWall");

    planning_scene_interface.applyCollisionObjects({col_object_table, col_object_backWall, col_object_sideWall});
}

// Global variables
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;

// Execute and check success for a given pose
bool planAndExecutePose(const geometry_msgs::msg::Pose& target_pose) {
    move_group_interface->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface->plan(plan));
    if (success) {
        move_group_interface->execute(plan);
    }
    move_group_interface->stop();
    move_group_interface->clearPoseTargets();
    return success;
}

// Move functions for predefined motions
bool moveToShelf() {
    geometry_msgs::msg::Pose shelf_pose = generatePoseMsg(0.5, 0.2, 1.0, 0, 0, 0, 1);
    return planAndExecutePose(shelf_pose);
}

bool moveToInventory() {
    geometry_msgs::msg::Pose inventory_pose = generatePoseMsg(0.6, 0.3, 1.0, 0, 0, 0, 1);
    return planAndExecutePose(inventory_pose);
}

// Move function for dynamic position based on request coordinates
bool moveToBookSpine(float x, float y, float z) {
    geometry_msgs::msg::Pose book_spine_pose = generatePoseMsg(x, y, z, 0, 0, 0, 1);
    return planAndExecutePose(book_spine_pose);
}

// Service callback function
void planAndExecuteCallback(
    const std::shared_ptr<interfaces::srv::ArmCommand::Request> request,
    std::shared_ptr<interfaces::srv::ArmCommand::Response> response) {

    if (request->command == "move_to_shelf") {
        response->success = moveToShelf();
    } else if (request->command == "move_to_inventory") {
        response->success = moveToInventory();
    } else if (request->command == "move_to_book_spine") {
        response->success = moveToBookSpine(request->x, request->y, request->z);
    } else {
        response->success = false;
        RCLCPP_WARN(rclcpp::get_logger("arm"), "Unknown command requested.");
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("arm", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    addCollisionObjects(*planning_scene_interface);

    // Create MoveGroupInterface
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");
    move_group_interface->setPlanningTime(10.0);

    // Create custom service
    auto plan_service = node->create_service<interfaces::srv::ArmCommand>(
        "arm", 
        &planAndExecuteCallback
    );

    RCLCPP_INFO(node->get_logger(), "Service 'arm' is ready.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
