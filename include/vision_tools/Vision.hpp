#pragma once

/**
 * @brief Facade over ROS2 Vision services.
 *
 * Responsibility: provide a clean, synchronous interface to Vision
 * services (object detection, face recognition) so callers never
 * need to know about ROS2 async patterns or timeouts.
 *
 * Design patterns applied:
 *   Facade          (GoF Structural)  — hides ROS2 service clients
 *                                       behind intention-revealing methods.
 *   Template Method (GoF Behavioral)  — call_service<T>() captures the
 *                                       invariant async→wait→return flow;
 *                                       each public method supplies only
 *                                       what varies (request + response
 *                                       mapping).
 *
 * Principles applied:
 *   SRP  — one class, one responsibility: Vision communication.
 *   DRY  — the async + future.wait_for + timeout pattern lives in
 *          exactly one place.
 *   DIP  — callers depend on Vision, not on rclcpp::Client directly.
 *
 * BioRobotics Laboratory, Facultad de Ingenieria, UNAM
 */

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <utility>

#include "faces_interfaces/srv/face_recog.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "human_interfaces/srv/detect_pointing.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "vision_interfaces/srv/detect_by_class.hpp"
#include "vision_interfaces/srv/find_seat.hpp"
#include "vision_interfaces/srv/get_detections.hpp"
#include "vision_interfaces/srv/get_table_alignment.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class Vision {
public:
  explicit Vision(const rclcpp::Node::SharedPtr &node);

  // detect_by_class POR PROBAR
  std::pair<bool, geometry_msgs::msg::PoseStamped>
  DetectByClass(const std::string &class_name, const std::string &model,
                std::chrono::seconds timeout = 5s);

  // FACE SERVICES
  bool DetectPerson(const std::string &name,
                    std::chrono::seconds timeout = 10s);

  bool IdentifyPerson(const std::string &name,
                      std::chrono::seconds timeout = 10s);

  bool DescribePerson(const std::string &name,
                      std::chrono::seconds timeout = 10s);

  bool RegisterPerson(const std::string &name,
                      std::chrono::seconds timeout = 10s);

  // Object detection

  std::pair<bool, std::vector<vision_interfaces::srv::GetDetections::Response::_detections_type::value_type>>
  DetectOwlV2(const std::vector<std::string> &queries,
              std::chrono::seconds timeout = 10s);

  // Find seat
  std::pair<bool, geometry_msgs::msg::PointStamped>
  FindSeat(std::chrono::seconds timeout = 10s);

  // Enable continuous detection
  bool EnableContinuous(bool enable, std::chrono::seconds timeout = 5s);

  // Pointing detection
  struct PointingResult {
    bool human_detected{false};
    std::string direction;
    geometry_msgs::msg::PointStamped point;
    sensor_msgs::msg::Image pose_image;
  };
  // Detect pointing direction and point
  PointingResult DetectPointing(std::chrono::seconds timeout = 10s);
  
  // Table alignment
  double GetTableAlignment(std::chrono::seconds timeout = 1s);

  // Focus
  bool Focus(bool enable, std::chrono::seconds timeout = 5s);

  // Find Object TF
  bool FindObjectTF(
    const std::string& query,
    geometry_msgs::msg::PoseStamped& pose,
    std::chrono::seconds timeout = 5s);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_clients_;

  // Service clients
  rclcpp::Client<vision_interfaces::srv::DetectByClass>::SharedPtr
      detect_by_class_client_;
  rclcpp::Client<vision_interfaces::srv::GetDetections>::SharedPtr
      owl_v2_client_;
  rclcpp::Client<vision_interfaces::srv::FindSeat>::SharedPtr
      find_seat_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
      enable_continuous_client_;
  rclcpp::Client<human_interfaces::srv::DetectPointing>::SharedPtr
      detect_pointing_client_;
  rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr
      detect_face_client_;
  rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr
      identify_face_client_;
  rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr
      describe_face_client_;
  rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr
      register_face_client_;
  rclcpp::Client<vision_interfaces::srv::GetTableAlignment>::SharedPtr
      get_table_alignment_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
      focus_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  template <typename ServiceT, typename CallbackT>
  bool call_service(typename rclcpp::Client<ServiceT>::SharedPtr &client,
                    typename ServiceT::Request::SharedPtr request,
                    CallbackT on_response, std::chrono::seconds timeout,
                    const std::string &label);
};

//align 
template <typename ServiceT, typename CallbackT>
bool Vision::call_service(typename rclcpp::Client<ServiceT>::SharedPtr &client,
                          typename ServiceT::Request::SharedPtr request,
                          CallbackT on_response, std::chrono::seconds timeout,
                          const std::string &label) {
  if (!client->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "%s service not available.",
                 label.c_str());
    return false;
  }

  auto future = client->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timeout calling %s service after %ld s.",
                 label.c_str(), timeout.count());
    return false;
  }

  on_response(future.get());
  return true;
}
