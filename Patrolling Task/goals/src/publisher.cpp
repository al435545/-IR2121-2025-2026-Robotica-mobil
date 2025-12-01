#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <vector>

using PoseStamped = geometry_msgs::msg::PoseStamped;
using AmclPose = geometry_msgs::msg::PoseWithCovarianceStamped;
using namespace std::chrono_literals;

// Variables globales
double last_x = 0.0;
double last_y = 0.0;
int stable_counter = 0;
bool goal_achieved = false;

// --- Funciones Auxiliares ---
geometry_msgs::msg::Quaternion euler_to_quaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

// --- Callback de AMCL Pose ---
void amcl_callback(const AmclPose::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double dx = x - last_x;
    double dy = y - last_y;
    double dist = std::sqrt(dx*dx + dy*dy);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cambio en amcl_pose: %.6f m", dist);

    // Ajuste: tolerancia mayor y menos ciclos
    if (dist < 0.05) {  // antes 0.003
        stable_counter++;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot detenido (%d/15)", stable_counter);
    } else {
        stable_counter = 0;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot moviéndose");
    }

    last_x = x;
    last_y = y;

    if (stable_counter > 15) {  // antes 30
        if (!goal_achieved) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Objetivo completado (según amcl_pose)");
            goal_achieved = true;
        }
    } else {
        goal_achieved = false;
    }
}

// --- Función Principal ---
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("version4_goal_publisher");

    auto publisher = node->create_publisher<PoseStamped>("/goal_pose", 10);
    auto subscriber = node->create_subscription<AmclPose>("/amcl_pose", 10, amcl_callback);

    rclcpp::WallRate loop_rate(100ms);

    std::vector<std::vector<double>> vector_goals = {
        {-0.40, 7.59, 0.0},
        {-5.7, 2.55, 0.002},
        {0.0364, -4.65, -0.009},
        {-5.33, -0.781, -0.009}
    };

    int indice = 0;
    bool initial_goal_published = false;

    rclcpp::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(node->get_logger(), "Iniciando secuencia de objetivos (v4 con amcl_pose).");

    while (rclcpp::ok()) {
    	RCLCPP_INFO(node->get_logger(), "estoy en el while");
        rclcpp::spin_some(node);
        loop_rate.sleep();

        if (!initial_goal_published) {
            // Publicar siempre el primer objetivo tras 2 segundos
            double target_x = vector_goals[indice][0];
            double target_y = vector_goals[indice][1];
            double target_yaw = vector_goals[indice][2];

            PoseStamped current_goal_msg;
            current_goal_msg.header.frame_id = "map";
            current_goal_msg.header.stamp = node->now();
            current_goal_msg.pose.position.x = target_x;
            current_goal_msg.pose.position.y = target_y;
            current_goal_msg.pose.position.z = 0.0;
            current_goal_msg.pose.orientation = euler_to_quaternion(target_yaw);

            publisher->publish(current_goal_msg);
            RCLCPP_WARN(node->get_logger(), "Publicando objetivo inicial #%d: (%.3f, %.3f, %.3f)", 
                        indice + 1, target_x, target_y, target_yaw);

            initial_goal_published = true;
        } 
        else if (goal_achieved) {
            indice++;
            if (indice >= vector_goals.size()) {
                RCLCPP_INFO(node->get_logger(), "¡Todos los objetivos (%zu) completados! Finalizando.", vector_goals.size());
                rclcpp::shutdown();
                break;
            }

            stable_counter = 0;
            goal_achieved = false;

            double target_x = vector_goals[indice][0];
            double target_y = vector_goals[indice][1];
            double target_yaw = vector_goals[indice][2];

            PoseStamped current_goal_msg;
            current_goal_msg.header.frame_id = "map";
            current_goal_msg.header.stamp = node->now();
            current_goal_msg.pose.position.x = target_x;
            current_goal_msg.pose.position.y = target_y;
            current_goal_msg.pose.position.z = 0.0;
            current_goal_msg.pose.orientation = euler_to_quaternion(target_yaw);

            publisher->publish(current_goal_msg);
            RCLCPP_WARN(node->get_logger(), "Publicando objetivo #%d: (%.3f, %.3f, %.3f)", 
                        indice + 1, target_x, target_y, target_yaw);
        }
    }

    rclcpp::shutdown();
    return 0;
}

