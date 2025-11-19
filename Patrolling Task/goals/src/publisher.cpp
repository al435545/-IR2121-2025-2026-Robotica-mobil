 #include "rclcpp/rclcpp.hpp" 

 #include "geometry_msgs/msg/pose_stamped.hpp" 

 #include "nav_msgs/msg/odometry.hpp"  

 #include "tf2/LinearMath/Quaternion.h" 

 #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 

 #include <cmath> 

 #include <chrono> 

 #include <memory> 


 using PoseStamped = geometry_msgs::msg::PoseStamped; 

 using Odometry = nav_msgs::msg::Odometry; 

 using namespace std::chrono_literals; 


 std::shared_ptr<rclcpp::Node> node = nullptr;  


 double prev_x = 0.0; 

 double prev_y = 0.0; 

 rclcpp::Time last_change_time; 

 bool first_odom_received = false; 


 void odom_callback(const Odometry::SharedPtr msg) { 

     double current_x = msg->pose.pose.position.x; 

     double current_y = msg->pose.pose.position.y; 

      

     // Perquè això funcioni, 'node' ha de ser inicialitzat amb l'objecte global a main() 

     if (!node) return; 


     if (!first_odom_received) { 

         prev_x = current_x; 

         prev_y = current_y; 

         last_change_time = node->now(); 

         first_odom_received = true; 

         return; 

     } 


     double distance = std::hypot(current_x - prev_x, current_y - prev_y); 


     const double TOLERANCE_DISTANCE = 0.01;  

      

     if (distance > TOLERANCE_DISTANCE) { 

         prev_x = current_x; 

         prev_y = current_y; 

         last_change_time = node->now(); 

 } 

 } 


 geometry_msgs::msg::Quaternion euler_to_quaternion(double yaw) { 

     tf2::Quaternion q; 

     q.setRPY(0, 0, yaw); 

     return tf2::toMsg(q); 

 } 


 int main(int argc, char * argv[]) { 

     rclcpp::init(argc, argv); 

      

     node = std::make_shared<rclcpp::Node>("version2_goal_publisher"); 


     auto publisher = node->create_publisher<PoseStamped>("/goal_pose", 1); 

      

     auto subscriber = node->create_subscription<Odometry>( 

         "/odom", 10, odom_callback); 

      

     double TARGET_X = -0.4019; 

     double TARGET_Y = 7.59398; 

     double TARGET_YAW = 0.0; 

      

     const auto STOPPING_TIMEOUT = 7s;  

     bool moving = false;  

      

     auto goal_msg = PoseStamped(); 

     goal_msg.header.frame_id = "map"; 

     goal_msg.header.stamp = node->now();  

     goal_msg.pose.position.x = TARGET_X; 

     goal_msg.pose.position.y = TARGET_Y; 

     goal_msg.pose.position.z = 0.0; 

     goal_msg.pose.orientation = euler_to_quaternion(TARGET_YAW); 


     RCLCPP_INFO(node->get_logger(), "Publicando objetivo a: (X: %.2f, Y: %.2f)", TARGET_X, TARGET_Y); 

   

     RCLCPP_INFO(node->get_logger(), "Objetivo publicado. Esperando finalización..."); 

     bool inicio = true;
     rclcpp::WallRate loop_rate(2s);
     while(rclcpp::ok()) { 

         rclcpp::spin_some(node); 
         RCLCPP_INFO(node->get_logger(), "1");
         loop_rate.sleep();



         if (moving==false && inicio==true){ 
             RCLCPP_INFO(node->get_logger(), "inicio");
             publisher->publish(goal_msg); 

             moving=true;

         if (moving==false && inicio==false){
             RCLCPP_INFO(node->get_logger(), "fin");
             break;
         } 

         if (first_odom_received && (node->now() - last_change_time) > STOPPING_TIMEOUT) { 
             RCLCPP_INFO(node->get_logger(), "3");
             moving = false; 


         } 

          

         loop_rate.sleep(); 

     } 


     RCLCPP_INFO(node->get_logger(), "Objetivo alcanzado. Nodo finalizado."); 


     rclcpp::shutdown(); 

     return 0; 

 }
 }