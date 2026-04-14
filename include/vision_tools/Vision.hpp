#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp> // Necesario para el alineado
#include "vision_interfaces/srv/detect_by_class.hpp"
#include "faces_interfaces/srv/face_recog.hpp"

#include <string>
#include <memory>
#include <chrono>
#include <mutex>
#include <future>

class Vision
{
public:
    explicit Vision(const rclcpp::Node::SharedPtr& node);

    // --- Objetos ---
    std::pair<bool, geometry_msgs::msg::PoseStamped>
        detectByClass(const std::string& class_name, const std::string& model);

    // --- Personas ---
    bool DetectPerson(const std::string& name);
    bool IdentifyPerson(const std::string& name);
    bool DescribePerson(const std::string& name);
    bool RegisterPerson(const std::string& name);

    // --- Alineación de Mesa ---
    double getTableAlignment(); // <--- LA QUE TE FALTABA

private:
    rclcpp::Node::SharedPtr node_;

    // Clientes
    rclcpp::Client<vision_interfaces::srv::DetectByClass>::SharedPtr detect_by_class_client_;
    rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr detect_face_client_;
    rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr identify_face_client_;
    rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr describe_face_client_;
    rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr register_face_client_;

    // Suscripción para alineado
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_table_alignment_;
    
    // Manejo de promesas y sincronización
    std::shared_ptr<std::promise<double>> alignment_promise_;
    std::mutex promise_mutex_;
};