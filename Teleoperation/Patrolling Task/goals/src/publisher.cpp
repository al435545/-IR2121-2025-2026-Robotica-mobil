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



int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("version1_goal_publisher");

    auto publisher = node->create_publisher<PoseStamped>("/goal_pose", 10);
    
    double TARGET_X = -0.4019;
    double TARGET_Y = 7.59398;
    double TARGET_YAW = 0.0;

    auto goal_msg = PoseStamped();
    
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = node->now(); 
    
    goal_msg.pose.position.x = TARGET_X;
    goal_msg.pose.position.y = TARGET_Y;
    goal_msg.pose.position.z = 0.0;
    
    goal_msg.pose.orientation = euler_to_quaternion(TARGET_YAW);

    RCLCPP_INFO(node->get_logger(), "Publicando objetivo a: (X: %.2f, Y: %.2f)", TARGET_X, TARGET_Y);

  
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(node->get_logger(), "Objetivo publicado. El nodo finalizará ahora (Versión 1).");
	
    while(rclcpp::ok()){
    	  publisher->publish(goal_msg);
    	  rclcpp:spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}

