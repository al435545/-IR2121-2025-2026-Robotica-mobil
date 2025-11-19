int main(int argc, char * argv[]) { 
    rclcpp::init(argc, argv); 

    // NOTA: Cal utilitzar SharedPtr per a la neteja de recursos
    rclcpp::Publisher<PoseStamped>::SharedPtr publisher;
    rclcpp::Subscription<Odometry>::SharedPtr subscriber;

    node = std::make_shared<rclcpp::Node>("version2_goal_publisher"); 

    publisher = node->create_publisher<PoseStamped>("/goal_pose", 1); 

    subscriber = node->create_subscription<Odometry>( 
        "/odom", 10, odom_callback); 

    double TARGET_X = -0.4019; 
    double TARGET_Y = 7.59398; 
    double TARGET_YAW = 0.0; 

    const auto STOPPING_TIMEOUT = 7s;  
    bool moving = false; // Comença a FALSE, esperant l'enviament inicial

    // Corregim WallRate a Rate(10Hz) per a una detecció precisa
    double loop_frequency = 10.0;
    rclcpp::Rate loop_rate(loop_frequency); 

    auto goal_msg = PoseStamped(); 
    goal_msg.header.frame_id = "map"; 
    goal_msg.header.stamp = node->now();  
    goal_msg.pose.position.x = TARGET_X; 
    goal_msg.pose.position.y = TARGET_Y; 
    goal_msg.pose.position.z = 0.0; 
    goal_msg.pose.orientation = euler_to_quaternion(TARGET_YAW); 

    RCLCPP_INFO(node->get_logger(), "Publicando objetivo a: (X: %.2f, Y: %.2f)", TARGET_X, TARGET_Y); 

    RCLCPP_INFO(node->get_logger(), "Objetivo publicado. Esperando finalización..."); 
    
    bool inicio = true; // Estat per controlar si el goal ja s'ha publicat

    while(rclcpp::ok()) { 

        rclcpp::spin_some(node); 
        
        // 1. LÒGICA D'INICI: Publica el goal una vegada
        if (moving == false && inicio == true) { 
            RCLCPP_INFO(node->get_logger(), "inicio: Publicando objetivo por primera vez.");
            publisher->publish(goal_msg); 
            moving = true; // El robot comença a moure's
            inicio = false; // Ja no estem en la fase d'inici
        }

        // 2. LÒGICA DE DETECCIÓ DE PARADA (S'executa mentre el robot es mou)
        if (moving == true && first_odom_received && (node->now() - last_change_time) > STOPPING_TIMEOUT) { 
            RCLCPP_INFO(node->get_logger(), "7 segundos de inactividad detectados. Finalizando.");
            moving = false; // Finalitza el moviment
        } 

        // 3. LÒGICA DE FINALITZACIÓ: Surt del bucle quan el moviment ha acabat
        if (moving == false && inicio == false) {
             RCLCPP_INFO(node->get_logger(), "fin: El objetivo se considera alcanzado.");
             break;
        }
          
        loop_rate.sleep(); 
    } 

    RCLCPP_INFO(node->get_logger(), "Objetivo alcanzado. Nodo finalizado."); 

    // Correcció del Segmentation Fault: Neteja de recursos
    publisher.reset();
    subscriber.reset();
    node.reset();
    
    rclcpp::shutdown(); 
    return 0; 
}