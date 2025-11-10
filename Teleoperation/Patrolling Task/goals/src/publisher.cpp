#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

// Alias para el tipo de mensaje
using PoseStamped = geometry_msgs::msg::PoseStamped;

/**
 * @brief Convierte un ángulo de yaw (en radianes) a un Cuaternión.
 */
geometry_msgs::msg::Quaternion euler_to_quaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw); // Roll=0, Pitch=0 para navegación planar
    return tf2::toMsg(q);
}

/**
 * @brief Función principal que ejecuta el nodo.
 */
void send_goal(int argc, char * argv[]) {
    // 1. Inicializar ROS 2
    rclcpp::init(argc, argv);
    
    // 2. Crear un nodo
    auto node = std::make_shared<rclcpp::Node>("version1_goal_publisher");

    // 3. Crear el publicador para el tópico /goal_pose
    // El Quality of Service (QoS) 'SystemDefaults' es adecuado para este uso.
    auto publisher = node->create_publisher<PoseStamped>("/goal_pose", 10);

    // --- Definición del Objetivo ---
    
    // Usaremos las coordenadas de ejemplo mencionadas en tu documentación
    const double TARGET_X = -1.0;
    const double TARGET_Y = 3.0;
    const double TARGET_YAW = 0.0; // Orientación simple, mirando al eje X

    // 4. Crear el mensaje de objetivo
    auto goal_msg = PoseStamped();
    
    // Configurar el header
    goal_msg.header.frame_id = "map";
    // Usar el tiempo actual para el timestamp
    goal_msg.header.stamp = node->now(); 
    
    // Configurar la posición
    goal_msg.pose.position.x = TARGET_X;
    goal_msg.pose.position.y = TARGET_Y;
    goal_msg.pose.position.z = 0.0;
    
    // Configurar la orientación (usando una función de utilidad)
    goal_msg.pose.orientation = euler_to_quaternion(TARGET_YAW);

    RCLCPP_INFO(node->get_logger(), "Publicando objetivo a: (X: %.2f, Y: %.2f)", TARGET_X, TARGET_Y);

    // 5. Publicar el mensaje
    publisher->publish(goal_msg);
    
    // Dar un pequeño tiempo para que el mensaje se envíe antes de finalizar
    // Esto es una buena práctica en ROS
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(node->get_logger(), "Objetivo publicado. El nodo finalizará ahora (Versión 1).");

    // 6. Finalizar ROS 2
    rclcpp::shutdown();
}

int main(int argc, char * argv[]) {
    send_goal(argc, argv);
    return 0;
}

