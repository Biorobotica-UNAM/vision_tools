#pragma once

/**
 * @brief Facade over ROS2 Vision services.
 * BioRobotics Laboratory, Facultad de Ingenieria, UNAM
 */

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "faces_interfaces/srv/face_recog.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "human_interfaces/srv/detect_pointing.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "vision_interfaces/srv/detect_by_class.hpp"
#include "vision_interfaces/srv/find_seat.hpp"
#include "vision_interfaces/srv/analyze_person.hpp"
#include "vision_interfaces/srv/get_detections.hpp"
#include "vision_interfaces/srv/bbox_to_pose.hpp"
#include "vision_interfaces/srv/get_table_alignment.hpp"
#include "vision_interfaces/srv/get_analytics.hpp"

using namespace std::chrono_literals;

class Vision {
public:
  using YoloDetection =
      vision_interfaces::srv::GetDetections::Response::_detections_type::
          value_type;

  explicit Vision(const rclcpp::Node::SharedPtr &node);

  // ── Object detection ───────────────────────────────────────────────────────

  std::pair<bool, geometry_msgs::msg::PoseStamped>
  DetectByClass(const std::string &class_name, const std::string &model,
                std::chrono::seconds timeout = 5s);

  std::pair<bool, std::vector<YoloDetection>>
  DetectOwlV2(const std::vector<std::string> &queries,
              std::chrono::seconds timeout = 10s);

  std::pair<bool, geometry_msgs::msg::PointStamped>
  FindSeat(std::chrono::seconds timeout = 10s);

  // ── Face pipeline ──────────────────────────────────────────────────────────

  bool DetectPerson(const std::string &name,
                    std::string &out_coords, float &out_z,
                    std::chrono::seconds timeout = 5s);

  bool IdentifyPerson(std::string &out_name,
                      std::chrono::seconds timeout = 5s);

  bool DescribePerson(const std::string &name, std::string &out_description,
                      std::chrono::seconds timeout = 10s);

  bool RegisterPerson(const std::string &name,
                      std::chrono::seconds timeout = 10s);


  // ── Analytics ────────────────────────────────────────────────────────
  bool AnalyzeCount(const std::string &target_class, bool reset_count,
                    int32_t &out_current_count, int32_t &out_class_count, int32_t &out_total_cumulative,
                    std::chrono::seconds timeout = 5s);

  bool AnalyzeSize(const std::string &target_class, const std::string &filter_mode, int32_t count_limit,
                   YoloDetection &out_best_match,
                   std::vector<YoloDetection> &out_detections,
                   std::chrono::seconds timeout = 5s);

  bool AnalyzeColor(const std::string &target_class, const std::string &color,
                    YoloDetection &out_best_match,
                    std::chrono::seconds timeout = 5s);

  bool AnalyzeCompare(const std::vector<std::string> &compare_classes,
                      std::string &out_winner_class,
                      std::chrono::seconds timeout = 5s);

  bool AnalyzePerson(const std::string &action, const std::string &color,
                     const std::string &attribute,
                     const std::string &gesture_type,
                     const std::string &arm_side,
                     std::string &out_message,
                     std::vector<YoloDetection> &out_detections,
                     bool &out_has_point,
                     geometry_msgs::msg::PointStamped &out_point,
                     std::chrono::seconds timeout = 10s);

                    
  // ── Auxiliary ──────────────────────────────────────────────────────────────

  bool EnableContinuous(bool enable, std::chrono::seconds timeout = 5s);

  struct PersonDetection {
    bool success{false};
    std::string coords;
    float z{0.f};
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::PoseStamped nav; // For navigation purposes, may be different from the actual pose due to reachability constraints
  };

  bool DetectPersonPose(PersonDetection &out_detection, std::chrono::seconds timeout = 5s);

  struct PointingResult {
    bool human_detected{false};
    std::string direction;
    geometry_msgs::msg::PointStamped point;
    sensor_msgs::msg::Image pose_image;
  };

  PointingResult DetectPointing(std::chrono::seconds timeout = 10s);

  bool GetOwlPose(
    const std::string& object_name,
    geometry_msgs::msg::PoseStamped& pose,
    geometry_msgs::msg::PoseStamped& nav, // For navigation purposes, may be different from the actual pose due to reachability constraints
    std::string& detected_name,
    std::chrono::seconds timeout = 10s);

  bool DetectionToNavPose(
    const YoloDetection& detection,
    geometry_msgs::msg::PoseStamped& pose,
    geometry_msgs::msg::PoseStamped& nav,
    std::chrono::seconds timeout = 5s);

  static constexpr double MIN_Z_HEIGHT      = 0.5;

  double GetTableAlignment(bool is_round = false, std::chrono::seconds timeout = 1s);

  bool Focus(bool enable, std::chrono::seconds timeout = 5s);

  rclcpp::CallbackGroup::SharedPtr getCallbackGroup() const {
    return cb_group_clients_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_clients_;

  rclcpp::Client<vision_interfaces::srv::DetectByClass>::SharedPtr detect_by_class_client_;
  rclcpp::Client<vision_interfaces::srv::GetDetections>::SharedPtr owl_v2_client_;
  rclcpp::Client<vision_interfaces::srv::FindSeat>::SharedPtr find_seat_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_continuous_client_;
  rclcpp::Client<human_interfaces::srv::DetectPointing>::SharedPtr detect_pointing_client_;
  rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr detect_face_client_;
  rclcpp::Client<vision_interfaces::srv::BboxToPose>::SharedPtr bbox_to_pose_client_;
  rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr identify_face_client_;
  rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr describe_face_client_;
  rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedPtr register_face_client_;
  rclcpp::Client<vision_interfaces::srv::GetTableAlignment>::SharedPtr get_table_alignment_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr focus_client_;
  rclcpp::Client<vision_interfaces::srv::GetAnalytics>::SharedPtr count_analize_client_;
  rclcpp::Client<vision_interfaces::srv::GetAnalytics>::SharedPtr size_analize_client_;
  rclcpp::Client<vision_interfaces::srv::GetAnalytics>::SharedPtr color_analize_client_;
  rclcpp::Client<vision_interfaces::srv::GetAnalytics>::SharedPtr compare_analize_client_;
  rclcpp::Client<vision_interfaces::srv::AnalyzePerson>::SharedPtr analyze_person_client_;

  template <typename ServiceT, typename CallbackT>
  bool call_service(
      typename rclcpp::Client<ServiceT>::SharedPtr &client,
      typename ServiceT::Request::SharedPtr request,
      CallbackT on_response,
      std::chrono::seconds timeout,
      const std::string &label);
};

// Template implementation
template <typename ServiceT, typename CallbackT>
bool Vision::call_service(
    typename rclcpp::Client<ServiceT>::SharedPtr &client,
    typename ServiceT::Request::SharedPtr request,
    CallbackT on_response,
    std::chrono::seconds timeout,
    const std::string &label)
{
  if (!client->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "%s service not available.", label.c_str());
    return false;
  }

  auto prom = std::make_shared<std::promise<typename ServiceT::Response::SharedPtr>>();
  auto fut = prom->get_future();

  client->async_send_request(
      request,
      [prom](typename rclcpp::Client<ServiceT>::SharedFuture response) {
        prom->set_value(response.get());
      });

  if (fut.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timeout calling %s after %ld s.", label.c_str(), timeout.count());
    return false;
  }

  on_response(fut.get());
  return true;
}
