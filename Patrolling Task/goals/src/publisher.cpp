#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <vector>

using PoseStamped = geometry_msgs::msg::PoseStamped;
using Odom = nav_msgs::msg::Odometry;
using namespace std::chrono_literals;

geometry_msgs::msg::Quaternion euler_to_quaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

double last_x = 0.0;
double last_y = 0.0;
bool first_odom = true;
int stable_counter = 0;

void odom_callback(const Odom::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    if (first_odom) {
        first_odom = false;
        last_x = x;
        last_y = y;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Primer /odom recibido");
        return;
    }

    double dx = x - last_x;
    double dy = y - last_y;
    double dist = std::sqrt(dx*dx + dy*dy);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cambio en odom: %.6f m", dist);

    if (dist < 0.003) {
        stable_counter++;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot detenido (%d/5)", stable_counter);
    } else {
        stable_counter = 0;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot moviéndose");
    }

    last_x = x;
    last_y = y;

    if (stable_counter > 5) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Objetivo completado (según odom)");
        rclcpp::shutdown();
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::WallRate loop_rate(2s);

    auto node = std::make_shared<rclcpp::Node>("version2_goal_publisher");

    auto publisher = node->create_publisher<PoseStamped>("/goal_pose", 10);
    auto subscriber = node->create_subscription<Odom>("/odom", 10, odom_callback);

    // Las siguientes variables (TARGET_X, Y, YAW) y la creación del mensaje 'goal_msg' 
    // están siendo reemplazadas por el bucle de 'vector_goals',
    // pero se mantienen aquí si quieres publicarlas antes del bucle.
    double TARGET_X = -0.4019;
    double TARGET_Y = 7.59398;
    double TARGET_YAW = 0.0;

    PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = node->now();
    goal_msg.pose.position.x = TARGET_X;
    goal_msg.pose.position.y = TARGET_Y;
    goal_msg.pose.position.z = 0.0;
    goal_msg.pose.orientation = euler_to_quaternion(TARGET_YAW);

    RCLCPP_INFO(node->get_logger(), "Publicando objetivo: (%.3f, %.3f)", TARGET_X, TARGET_Y);

    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    bool inicio = true;
    int indice = 0;
    std::vector<std::vector<double>> vector_goals = {
    {-0.40, 7.59, 0.0},
    {-3.4, 2.55, 0.002},
    {0.0364, -4.65, -0.009},
    {-5.33, -0.781, -0.009}
    };

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
	loop_rate.sleep();
        
        PoseStamped current_goal_msg;
        current_goal_msg.header.frame_id = "map";
        current_goal_msg.header.stamp = node->now();
        current_goal_msg.pose.position.x = vector_goals[indice][0];
        current_goal_msg.pose.position.y = vector_goals[indice][1];
	current_goal_msg.pose.position.z = 0.0;
        current_goal_msg.pose.orientation = euler_to_quaternion(TARGET_YAW);
        
        loop_rate.sleep();
        
	if (inicio) {
              publisher->publish(current_goal_msg);
              inicio = false;
              indice++;
              RCLCPP_INFO(node->get_logger(), "entro dentro del bucle de 1 vez");
              rclcpp::sleep_for(std::chrono::milliseconds(500));
        }

        if (indice >= vector_goals.size()) {
            indice = 0;
        }
        
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
