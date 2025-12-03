#include <chrono>
#include <iostream>
#include <cmath>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <vector>

using namespace std::chrono_literals;

// Constantes de Tolerancia para la Detección de Llegada
const double DISTANCE_TOLERANCE = 0.4;
const double ORIENTATION_TOLERANCE = 0.4;
const double LOOP_FREQUENCY = 0.5; // 0.5 Hz (equivalente a WallRate(2s))

// Variables globales actualizadas por el callback
double current_x = 0.0;
double current_y = 0.0;
double current_w = 1.0;

/**
 * @brief Callback para recibir la pose del robot desde /amcl_pose.
 */
void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msgAMCL)
{
    current_x = msgAMCL->pose.pose.position.x;
    current_y = msgAMCL->pose.pose.position.y;
    current_w = msgAMCL->pose.pose.orientation.w;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared ("waypoint_publisher_node");

    auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);

    auto subscription =
    node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 1, std::bind(amcl_callback, std::placeholders::_1));

    geometry_msgs::msg::PoseStamped goal_message;
    goal_message.header.frame_id = "map";

    std::vector<std::vector<double>> vector_goals = {
        {-0.40, 7.59, 0.0, 0.0, 0.0, 0.0, 1.0},
        {-5.7, 2.55, 0.002, 0.0, 0.0, 0.180, 0.983},
        {0.0364, -4.65, -0.009, 0.0, 0.0, -0.830, 0.557},
        {-5.33, -0.781, -0.009, 0.0, 0.0, -0.111, 0.993}
    };

    rclcpp::Rate loop_rate(LOOP_FREQUENCY);

    for (const auto& target_pose : vector_goals) {

        // 1. Configurar el mensaje del objetivo
        goal_message.pose.position.x = target_pose[0];
        goal_message.pose.position.y = target_pose[1];
        goal_message.pose.position.z = target_pose[2];
        goal_message.pose.orientation.x = target_pose[3];
        goal_message.pose.orientation.y = target_pose[4];
        goal_message.pose.orientation.z = target_pose[5];
        goal_message.pose.orientation.w = target_pose[6];

        bool needs_publishing = true;

        RCLCPP_INFO(node->get_logger(), "Estableciendo Objetivo: (%.2f, %.2f)", target_pose[0], target_pose[1]);

        while (rclcpp::ok())
        {
          
            rclcpp::spin_some(node);
            loop_rate.sleep();

         
            if (needs_publishing) {
                goal_message.header.stamp = node->now();
                RCLCPP_WARN(node->get_logger(), "Publicando objetivo en /goal_pose...");
                publisher->publish(goal_message);
                loop_rate.sleep();
                needs_publishing = false;
            }

            double dx = current_x - target_pose[0];
            double dy = current_y - target_pose[1];
            
            double distance = std::sqrt(dx * dx + dy * dy);
            
            double orientation_diff = std::abs(current_w - target_pose[6]);

            if (distance < DISTANCE_TOLERANCE && orientation_diff < ORIENTATION_TOLERANCE) {

                RCLCPP_INFO(node->get_logger(), "Robot ha arribat al destí: (%.2f, %.2f)", target_pose[0], target_pose[1]);
                loop_rate.sleep();
                break;
            }
            else {
                RCLCPP_INFO(node->get_logger(), "Estat: Distància restant: %.2f m", distance);
            }

            loop_rate.sleep();
        }
    }

    RCLCPP_INFO(node->get_logger(), "Ruta completada. Tancant el node.");
    rclcpp::shutdown();
    return 0;
}
