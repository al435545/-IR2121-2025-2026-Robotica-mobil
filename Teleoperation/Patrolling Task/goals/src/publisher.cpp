#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

using PoseStamped = geometry_msgs::msg::PoseStamped;

geometry_msgs::msg::Quaternion euler_to_quaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw); // plano al ser 0 0
    return tf2::toMsg(q);
}

void send_goal(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("version1_goal_publisher");

    auto publisher = node->create_publisher<PoseStamped>("/goal_pose", 10);
    
    double TARGET_X = -1.0;
    double TARGET_Y = 3.0;
    double TARGET_YAW = 0.0;

    auto goal_msg = PoseStamped();
    
    goal_msg.header.frame_id = "map";
    // Usar el tiempo actual para el timestamp
    goal_msg.header.stamp = node->now(); 
    
    goal_msg.pose.position.x = TARGET_X;
    goal_msg.pose.position.y = TARGET_Y;
    goal_msg.pose.position.z = 0.0;
    
    goal_msg.pose.orientation = euler_to_quaternion(TARGET_YAW);

    RCLCPP_INFO(node->get_logger(), "Publicando objetivo a: (X: %.2f, Y: %.2f)", TARGET_X, TARGET_Y);

    publisher->publish(goal_msg);
    
    //rclcpp::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(node->get_logger(), "Objetivo publicado. El nodo finalizará ahora (Versión 1).");

    rclcpp::shutdown();
}

int main(int argc, char * argv[]) {
    send_goal(argc, argv);
    return 0;
}

