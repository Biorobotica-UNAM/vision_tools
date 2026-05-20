/**
 * @brief Facade implementation for Vision services.
 * BioRobotics Laboratory, Facultad de Ingenieria, UNAM
 */

#include "vision_tools/Vision.hpp"
#include <vector>
#include <algorithm>
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

Vision::Vision(const rclcpp::Node::SharedPtr &node) : node_(node)
{
  cb_group_clients_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  const rclcpp::QoS qos = rclcpp::ServicesQoS();

  detect_by_class_client_ =
      node_->create_client<vision_interfaces::srv::DetectByClass>(
          "/vision/detect_by_class", qos, cb_group_clients_);

  owl_v2_client_ =
      node_->create_client<vision_interfaces::srv::GetDetections>(
          "/vision/owl/owlv2_service",
          rmw_qos_profile_services_default,
          cb_group_clients_);

  find_seat_client_ =
      node_->create_client<vision_interfaces::srv::FindSeat>(
          "/vision/find_seat", qos, cb_group_clients_);

  enable_continuous_client_ =
      node_->create_client<std_srvs::srv::SetBool>(
          "/vision/enable_continuous", qos, cb_group_clients_);

  detect_pointing_client_ =
      node_->create_client<human_interfaces::srv::DetectPointing>(
          "/human_detection/detect_wrist",
          rmw_qos_profile_services_default,
          cb_group_clients_);

  detect_face_client_ =
      node_->create_client<faces_interfaces::srv::FaceRecog>(
          "/vision/detect_person", qos, cb_group_clients_);

  identify_face_client_ =
      node_->create_client<faces_interfaces::srv::FaceRecog>(
          "/vision/identify_person", qos, cb_group_clients_);

  describe_face_client_ =
      node_->create_client<faces_interfaces::srv::FaceRecog>(
          "/vision/describe_person", qos, cb_group_clients_);

  register_face_client_ =
      node_->create_client<faces_interfaces::srv::FaceRecog>(
          "/vision/register_person", qos, cb_group_clients_);

  get_table_alignment_client_ =
      node_->create_client<vision_interfaces::srv::GetTableAlignment>(
          "/vision/get_table_alignment", qos, cb_group_clients_);

  focus_client_ =
      node_->create_client<std_srvs::srv::SetBool>(
          "/vision/focus", qos, cb_group_clients_);

  sub_table_alignment_ = node_->create_subscription<std_msgs::msg::Float64>(
      "camera/table_alignment", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(promise_mutex_);
        if (alignment_promise_) {
          alignment_promise_->set_value(msg->data);
          alignment_promise_ = nullptr;
        }
      });
}

// ── DetectByClass ────────────────────────────────────────────────────────────

std::pair<bool, geometry_msgs::msg::PoseStamped>
Vision::DetectByClass(const std::string &class_name,
                      const std::string &model,
                      std::chrono::seconds timeout)
{
  auto request =
      std::make_shared<vision_interfaces::srv::DetectByClass::Request>();
  request->class_name = class_name;
  request->model      = model;

  std::pair<bool, geometry_msgs::msg::PoseStamped> result{false, {}};

  bool completed = call_service<vision_interfaces::srv::DetectByClass>(
      detect_by_class_client_, request,
      [&](vision_interfaces::srv::DetectByClass::Response::SharedPtr response) {
        result.first  = response->success;
        result.second = response->pose;
        RCLCPP_INFO(node_->get_logger(), "DetectByClass [%s] %s",
                    class_name.c_str(), result.first ? "OK" : "NOT FOUND");
      },
      timeout, "DetectByClass");

  return completed ? result
                   : std::make_pair(false, geometry_msgs::msg::PoseStamped{});
}

// ── DetectPerson ─────────────────────────────────────────────────────────────

bool Vision::DetectPerson(const std::string &name,
                          std::string &out_coords, float &out_z,
                          std::chrono::seconds timeout)
{
  auto request =
      std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
  request->name = name;

  bool success = false;
  bool completed = call_service<faces_interfaces::srv::FaceRecog>(
      detect_face_client_, request,
      [&](faces_interfaces::srv::FaceRecog::Response::SharedPtr response) {
        success = response->success;
        if (success) {
          if (response->bbox.size() >= 4) {
            out_coords =
                std::to_string(static_cast<int>(response->bbox[0])) + "," +
                std::to_string(static_cast<int>(response->bbox[1])) + "," +
                std::to_string(static_cast<int>(response->bbox[2])) + "," +
                std::to_string(static_cast<int>(response->bbox[3]));
          }
          out_z = response->z;
        }
        RCLCPP_INFO(node_->get_logger(), "DetectPerson %s: %s",
                    success ? "OK" : "FAILED", response->name.c_str());
      },
      timeout, "DetectPerson");

  return completed && success;
}

// ── IdentifyPerson ───────────────────────────────────────────────────────────

bool Vision::IdentifyPerson(std::string &out_name,
                            std::chrono::seconds timeout)
{
  auto request =
      std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
  request->name = "";

  bool success = false;
  bool completed = call_service<faces_interfaces::srv::FaceRecog>(
      identify_face_client_, request,
      [&](faces_interfaces::srv::FaceRecog::Response::SharedPtr response) {
        success = response->success;
        if (success) {
          out_name = response->name;
        }
        RCLCPP_INFO(node_->get_logger(), "IdentifyPerson %s: %s",
                    success ? "OK" : "FAILED", response->name.c_str());
      },
      timeout, "IdentifyPerson");

  return completed && success;
}

// ── DescribePerson ───────────────────────────────────────────────────────────

bool Vision::DescribePerson(const std::string &name,
                            std::string &out_description,
                            std::chrono::seconds timeout)
{
  auto request =
      std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
  request->name = name;

  bool success = false;
  bool completed = call_service<faces_interfaces::srv::FaceRecog>(
      describe_face_client_, request,
      [&](faces_interfaces::srv::FaceRecog::Response::SharedPtr response) {
        success = response->success;
        if (success) {
          out_description = response->description;
        }
        RCLCPP_INFO(node_->get_logger(), "DescribePerson %s: %s",
                    success ? "OK" : "FAILED", response->description.c_str());
      },
      timeout, "DescribePerson");

  return completed && success;
}

// ── RegisterPerson ───────────────────────────────────────────────────────────

bool Vision::RegisterPerson(const std::string &name,
                            std::chrono::seconds timeout)
{
  auto request =
      std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
  request->name = name;

  bool success = false;
  bool completed = call_service<faces_interfaces::srv::FaceRecog>(
      register_face_client_, request,
      [&](faces_interfaces::srv::FaceRecog::Response::SharedPtr response) {
        success = response->success;
        RCLCPP_INFO(node_->get_logger(), "RegisterPerson %s: %s",
                    success ? "OK" : "FAILED", response->name.c_str());
      },
      timeout, "RegisterPerson");

  return completed && success;
}

// ── DetectOwlV2 ──────────────────────────────────────────────────────────────

std::pair<bool, std::vector<Vision::YoloDetection>>
Vision::DetectOwlV2(const std::vector<std::string> &queries,
                    std::chrono::seconds timeout)
{
  auto request =
      std::make_shared<vision_interfaces::srv::GetDetections::Request>();
  request->queries = queries;

  std::pair<bool, std::vector<Vision::YoloDetection>> result{false, {}};

  bool completed = call_service<vision_interfaces::srv::GetDetections>(
      owl_v2_client_, request,
      [&](vision_interfaces::srv::GetDetections::Response::SharedPtr response) {
        result.first  = response->success;
        result.second = response->detections;
        RCLCPP_INFO(node_->get_logger(),
                    "DetectOwlV2 %s: %zu detection(s). %s",
                    result.first ? "OK" : "FAILED",
                    result.second.size(),
                    response->message.c_str());
      },
      timeout, "DetectOwlV2");

  return completed ? result
                   : std::make_pair(false, std::vector<Vision::YoloDetection>{});
}

// ── FindSeat ─────────────────────────────────────────────────────────────────

std::pair<bool, geometry_msgs::msg::PointStamped>
Vision::FindSeat(std::chrono::seconds timeout)
{
  auto request =
      std::make_shared<vision_interfaces::srv::FindSeat::Request>();

  std::pair<bool, geometry_msgs::msg::PointStamped> result{false, {}};

  bool completed = call_service<vision_interfaces::srv::FindSeat>(
      find_seat_client_, request,
      [&](vision_interfaces::srv::FindSeat::Response::SharedPtr response) {
        result.first  = response->success;
        result.second = response->point;
        RCLCPP_INFO(node_->get_logger(), "FindSeat %s: %s",
                    result.first ? "OK" : "FAILED", response->message.c_str());
      },
      timeout, "FindSeat");

  return completed ? result
                   : std::make_pair(false, geometry_msgs::msg::PointStamped{});
}

// ── EnableContinuous ─────────────────────────────────────────────────────────

bool Vision::EnableContinuous(bool enable, std::chrono::seconds timeout)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enable;

  bool success = false;
  bool completed = call_service<std_srvs::srv::SetBool>(
      enable_continuous_client_, request,
      [&](std_srvs::srv::SetBool::Response::SharedPtr response) {
        success = response->success;
        RCLCPP_INFO(node_->get_logger(), "EnableContinuous %s %s: %s",
                    enable ? "ON" : "OFF",
                    success ? "OK" : "FAILED",
                    response->message.c_str());
      },
      timeout, "EnableContinuous");

  return completed && success;
}

// ── DetectPointing ───────────────────────────────────────────────────────────

Vision::PointingResult Vision::DetectPointing(std::chrono::seconds timeout)
{
  auto request =
      std::make_shared<human_interfaces::srv::DetectPointing::Request>();

  PointingResult result{};

  bool completed = call_service<human_interfaces::srv::DetectPointing>(
      detect_pointing_client_, request,
      [&](human_interfaces::srv::DetectPointing::Response::SharedPtr response) {
        result.human_detected = response->human_detected;
        result.direction      = response->direction;
        result.point          = response->point;
        result.pose_image     = response->pose_image;
        RCLCPP_INFO(node_->get_logger(),
                    "DetectPointing: human=%s direction=%s",
                    result.human_detected ? "YES" : "NO",
                    result.direction.c_str());
      },
      timeout, "DetectPointing");

  if (!completed) {
    result.human_detected = false;
  }
  return result;
}

// ── GetTableAlignment ────────────────────────────────────────────────────────

double Vision::GetTableAlignment(std::chrono::seconds timeout)
{
  auto request = std::make_shared<vision_interfaces::srv::GetTableAlignment::Request>();

  auto prom = std::make_shared<std::promise<double>>();
  auto fut  = prom->get_future();

  {
    std::lock_guard<std::mutex> lock(promise_mutex_);
    alignment_promise_ = prom;
  }

  double angle = 0.0;
  bool success = false;

  bool completed = call_service<vision_interfaces::srv::GetTableAlignment>(
      get_table_alignment_client_, request,
      [&](vision_interfaces::srv::GetTableAlignment::Response::SharedPtr response) {
        success = response->success;
        angle = response->angle;
        RCLCPP_INFO(node_->get_logger(), "GetTableAlignment %s: %s (angle=%.3f)",
                    success ? "OK" : "FAILED", response->message.c_str(),
                    angle);
      },
      timeout, "GetTableAlignment");

  if (!completed || !success) {
    std::lock_guard<std::mutex> lock(promise_mutex_);
    alignment_promise_ = nullptr;
    return 0.0;
  }

  if (fut.wait_for(timeout) == std::future_status::ready) {
    return fut.get();
  }

  {
    std::lock_guard<std::mutex> lock(promise_mutex_);
    alignment_promise_ = nullptr;
  }

  RCLCPP_WARN(node_->get_logger(),
              "Timeout waiting for table alignment topic after %ld s.",
              timeout.count());
  return 0.0;
}

// ── Focus ────────────────────────────────────────────────────────────────────

bool Vision::Focus(bool enable, std::chrono::seconds timeout)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enable;

  bool success = false;
  bool completed = call_service<std_srvs::srv::SetBool>(
      focus_client_, request,
      [&](std_srvs::srv::SetBool::Response::SharedPtr response) {
        success = response->success;
        RCLCPP_INFO(node_->get_logger(), "Focus %s: %s",
                    enable ? "ON" : "OFF",
                    success ? "OK" : "FAILED");
      },
      timeout, "Focus");

  return completed && success;
}