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

// Variables globales (Accedidas por el main thread y el odom_callback)
double last_x = 0.0;
double last_y = 0.0;
bool first_odom = true;
int stable_counter = 0; // Usado para detectar que el robot está parado

// Se necesita una bandera para que el bucle principal sepa que el objetivo
// actual ha sido completado y se debe pasar al siguiente.
bool goal_achieved = false; 

// --- Funciones Auxiliares ---

geometry_msgs::msg::Quaternion euler_to_quaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

// --- Callback de Odometría ---

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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot detenido (%d/30)", stable_counter);
    } else {
        stable_counter = 0;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot moviéndose");
    }

    last_x = x;
    last_y = y;

    // Si ha estado estable el tiempo suficiente, marcar el objetivo como alcanzado
    if (stable_counter > 30) {
        if (!goal_achieved) { // Evita registrar el objetivo varias veces
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Objetivo completado (según odom)");
            goal_achieved = true;
        }
    } else {
        // Asegurarse de que la bandera se restablece si el robot comienza a moverse de nuevo
        // Esto es útil si el robot "sobrepasa" el objetivo ligeramente y luego vuelve.
        goal_achieved = false; 
    }
}

// --- Función Principal ---

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("version2_goal_publisher");

    auto publisher = node->create_publisher<PoseStamped>("/goal_pose", 10);
    auto subscriber = node->create_subscription<Odom>("/odom", 10, odom_callback);

    // Reducir el WallRate a un valor práctico (10Hz) para que el bucle principal
    // reaccione rápidamente a los cambios en 'goal_achieved'.
    rclcpp::WallRate loop_rate(100ms); 
    
    // El resto de variables y código de ejemplo
    // ...
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
    // ...

    std::vector<std::vector<double>> vector_goals = {
        {-0.40, 7.59, 0.0},
        {-5.7, 2.55, 0.002},
        {0.0364, -4.65, -0.009},
        {-5.33, -0.781, -0.009}
    };

    int indice = 0;
    
    // Flag para saber si hemos publicado el primer objetivo.
    bool initial_goal_published = false; 
    
    // Pausa inicial
    rclcpp::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(node->get_logger(), "Iniciando secuencia de objetivos.");

    // --- BUCLE PRINCIPAL MODIFICADO ---
    while (rclcpp::ok()) {
        // 1. Ejecutar callbacks (aquí se llama a odom_callback y se actualizan stable_counter y goal_achieved)
        rclcpp::spin_some(node);
        loop_rate.sleep();
        // 2. Lógica de Publicación de Objetivos
        if (!initial_goal_published) {
            // Publicar el primer objetivo para empezar la secuencia.
            
            // Solo publicar el primer goal después de recibir la primera odom
            if (!first_odom) { 
                rclcpp::sleep_for(std::chrono::seconds(1));
                RCLCPP_WARN(node->get_logger(), "SIGUIENTES ODOM AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"); 
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
        } 
        else if (goal_achieved) {
            // El callback de odom ha marcado que el robot se detuvo.
            
            // 2a. Incrementar el índice para el siguiente objetivo
            indice++;
            
            // 2b. Comprobación final
            if (indice >= vector_goals.size()) {
                RCLCPP_INFO(node->get_logger(), "¡Todos los objetivos (%zu) completados! Finalizando.", vector_goals.size());
                rclcpp::shutdown();
                break; 
            }
            
            // 2c. Resetear el estado para el nuevo movimiento
            stable_counter = 0;
            goal_achieved = false; 

            // 2d. Publicar el siguiente objetivo
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

        // 3. Esperar al siguiente ciclo
    }

    rclcpp::shutdown();
    return 0;
}
