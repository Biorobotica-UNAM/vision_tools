/**
 * @brief Facade implementation for Vision services.
 * BioRobotics Laboratory, Facultad de Ingenieria, UNAM
 */

#include "vision_tools/Vision.hpp"
#include <vector>
#include <algorithm>
#include <thread>

using namespace std::chrono_literals;

Vision::Vision(const rclcpp::Node::SharedPtr &node) : node_(node)
{
  cb_group_clients_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Guys, please. Make sure the code compiles with ROS 2 Humble too. We talked about this!!!
  //const rclcpp::QoS qos = rclcpp::ServicesQoS();
  const rmw_qos_profile_t qos = rmw_qos_profile_services_default;

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

  bbox_to_pose_client_ =
      node_->create_client<vision_interfaces::srv::BboxToPose>(
          "/vision/bbox_to_pose", rmw_qos_profile_services_default, cb_group_clients_);

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

  count_analize_client_ =
      node_->create_client<vision_interfaces::srv::GetAnalytics>(
          "vision/count_objects", qos, cb_group_clients_);
  
  size_analize_client_ =
      node_->create_client<vision_interfaces::srv::GetAnalytics>(
          "vision/get_by_size", qos, cb_group_clients_);
  
  color_analize_client_ =
      node_->create_client<vision_interfaces::srv::GetAnalytics>(
          "vision/get_by_color", qos, cb_group_clients_);
  
  compare_analize_client_ =
      node_->create_client<vision_interfaces::srv::GetAnalytics>(
          "vision/compare_count", qos, cb_group_clients_);
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
        // Solo los exitos quedan en INFO. Los fallos por frame sin cara son
        // DEBUG para no saturar la terminal durante los reintentos de escaneo.
        if (success && response->bbox.size() >= 4) {
          RCLCPP_INFO(
              node_->get_logger(),
              "DetectPerson %s: name='%s' desc='%s' bbox=[%.1f %.1f %.1f %.1f] z=%.3f",
              success ? "OK" : "FAILED", response->name.c_str(),
              response->description.c_str(), response->bbox[0],
              response->bbox[1], response->bbox[2], response->bbox[3],
              response->z);
        } else if (success) {
          RCLCPP_INFO(node_->get_logger(),
                      "DetectPerson OK: name='%s' desc='%s' bbox_size=%zu",
                      response->name.c_str(), response->description.c_str(),
                      response->bbox.size());
        } else if (response->bbox.size() >= 4) {
          RCLCPP_DEBUG(
              node_->get_logger(),
              "DetectPerson FAILED: name='%s' desc='%s' bbox=[%.1f %.1f %.1f %.1f] z=%.3f",
              response->name.c_str(), response->description.c_str(),
              response->bbox[0], response->bbox[1], response->bbox[2],
              response->bbox[3], response->z);
        } else {
          RCLCPP_DEBUG(node_->get_logger(),
                       "DetectPerson FAILED: name='%s' desc='%s' bbox_size=%zu",
                       response->name.c_str(), response->description.c_str(),
                       response->bbox.size());
        }
      },
      timeout, "DetectPerson");

  if (!completed) {
    RCLCPP_ERROR(node_->get_logger(),
                 "DetectPerson service call did not complete within %ld s",
                 timeout.count());
  }

  return completed && success;
}

// ── DetectPerson ─────────────────────────────────────────────────────────────

bool Vision::DetectPersonPose(
    PersonDetection &out_detection,
    std::chrono::seconds timeout)
{
  std::string coords;
  float z = 0.f;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int detect_attempt = 0;

  // DetectPerson se reintenta hasta el deadline de la posicion actual. La
  // cabeza no se mueve aqui; FindPerson decide el siguiente pan/tilt si esta
  // ventana termina sin detectar una cara.
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    ++detect_attempt;
    if (DetectPerson("", coords, z, 1s)) {
      break;
    }

    RCLCPP_DEBUG(
        node_->get_logger(),
        "DetectPersonPose: DetectPerson failed before BboxToPose (attempt %d); retrying",
        detect_attempt);
    std::this_thread::sleep_for(200ms);
  }

  if (coords.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "DetectPersonPose: no face detected within %ld s",
                timeout.count());
    return false;
  }

  int32_t x1{0}, y1{0}, x2{0}, y2{0};

  try {
    std::istringstream ss(coords);
    std::string tok;
    std::vector<int32_t> vals;

    while (std::getline(ss, tok, ',')) {
      vals.push_back(static_cast<int32_t>(std::stoi(tok)));
    }

    if (vals.size() < 4) {
      RCLCPP_WARN(node_->get_logger(),
                  "DetectPersonPose: invalid bbox '%s'",
                  coords.c_str());
      return false;
    }

    x1 = vals[0];
    y1 = vals[1];
    x2 = vals[2];
    y2 = vals[3];
  }
  catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "DetectPersonPose parse error: %s",
                 e.what());
    return false;
  }

  if (!bbox_to_pose_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "BboxToPose service not available");
    return false;
  }

  auto request =
      std::make_shared<vision_interfaces::srv::BboxToPose::Request>();

  request->header.frame_id = "head_front_camera_rgb_optical_frame";

  request->x1 = x1;
  request->y1 = y1;
  request->x2 = x2;
  request->y2 = y2;

  int attempt = 0;

  // Si la cara ya fue detectada pero la nube/TF aun no da pose, se reintenta
  // BboxToPose con el mismo bbox hasta agotar la misma ventana de tiempo.
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    ++attempt;
    request->header.stamp = node_->now();

    geometry_msgs::msg::PoseStamped pose;
    bool success = false;

    const bool completed =
        call_service<vision_interfaces::srv::BboxToPose>(
            bbox_to_pose_client_,
            request,
            [&](vision_interfaces::srv::BboxToPose::Response::SharedPtr response)
            {
              success = response->success;

              if (success) {
                pose = response->pose;
              }
            },
            1s,
            "BboxToPose");

    if (completed && success) {
      out_detection.success = true;
      out_detection.coords  = coords;
      out_detection.z       = z;
      out_detection.pose    = pose;

      return true;
    }

    RCLCPP_DEBUG(
      node_->get_logger(),
      "DetectPersonPose: BboxToPose failed after face detection (attempt %d); retrying",
      attempt);
    std::this_thread::sleep_for(200ms);
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "DetectPersonPose: face detected but BboxToPose never returned a valid pose");
  return false;
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
        RCLCPP_INFO(node_->get_logger(),
                    "IdentifyPerson %s: name='%s' desc='%s'",
                    success ? "OK" : "FAILED", response->name.c_str(),
                    response->description.c_str());
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
        RCLCPP_INFO(node_->get_logger(),
                    "RegisterPerson %s: name='%s' desc='%s'",
                    success ? "OK" : "FAILED", response->name.c_str(),
                    response->description.c_str());
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

// ── GetOwlPose ─────────────────────────────────────────────────────────────────

bool Vision::GetOwlPose(
    const std::string& object_name,
    geometry_msgs::msg::PoseStamped& pose,
    std::string& detected_name,
    std::chrono::seconds timeout)
{
    auto result = DetectOwlV2({object_name}, timeout);

    if (!result.first || result.second.empty())
        return false;

    // Tomar mejor detección
    const auto& det = result.second[0];

    detected_name = det.class_name;

    // Verificar servicio
    if (!bbox_to_pose_client_->wait_for_service(1s))
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "BboxToPose service not available");

        return false;
    }

    // Crear request
    auto request =
        std::make_shared<
            vision_interfaces::srv::BboxToPose::Request>();

    request->header.stamp = node_->now();

    request->header.frame_id =
        "head_front_camera_rgb_optical_frame";

    // OWL bbox
    request->x1 =
    static_cast<int32_t>(det.x_min);

	request->y1 =
		static_cast<int32_t>(det.y_min);

	request->x2 =
		static_cast<int32_t>(det.x_max);

	request->y2 =
		static_cast<int32_t>(det.y_max);

    bool success = false;

    bool completed =
        call_service<vision_interfaces::srv::BboxToPose>(
            bbox_to_pose_client_,
            request,
            [&](vision_interfaces::srv::BboxToPose::Response::SharedPtr response)
            {
                success = response->success;

                if (success)
                {
                    pose = response->pose;

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "BboxToPose OK "
                        "pos=[%.3f %.3f %.3f] "
                        "quat=[%.3f %.3f %.3f %.3f]",
                        pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z,
                        pose.pose.orientation.x,
                        pose.pose.orientation.y,
                        pose.pose.orientation.z,
                        pose.pose.orientation.w);
                }
            },
            timeout,
            "BboxToPose");

    if (!completed || !success)
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "Failed to compute pose from bbox");

        return false;
    }

    // Clamp mínimo de altura
    pose.pose.position.z =
        std::max(
            pose.pose.position.z,
            MIN_Z_HEIGHT);

    return true;
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
  auto request =
      std::make_shared<vision_interfaces::srv::GetTableAlignment::Request>();

  double angle = 0.0;
  bool success = false;

  bool completed = call_service<vision_interfaces::srv::GetTableAlignment>(
      get_table_alignment_client_,
      request,
      [&](vision_interfaces::srv::GetTableAlignment::Response::SharedPtr response)
      {
        success = response->success;
        angle   = response->angle;

        RCLCPP_INFO(
            node_->get_logger(),
            "GetTableAlignment %s: %s (angle=%.3f)",
            success ? "OK" : "FAILED",
            response->message.c_str(),
            angle);
      },
      timeout,
      "GetTableAlignment");

  if (!completed || !success) {
    return 0.0;
  }

  return angle;
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

// ── Analyze count ────────────────────────────────────────────────────────────

bool Vision::AnalyzeCount(const std::string &target_class, bool reset_count,
                          int32_t &out_current_count, int32_t &out_class_count, int32_t &out_total_cumulative,
                          std::chrono::seconds timeout)
{
  auto request = std::make_shared<vision_interfaces::srv::GetAnalytics::Request>();
  request->action = "count";
  request->target_class = target_class;
  request->reset_count = reset_count;

  bool success = false;
  bool completed = call_service<vision_interfaces::srv::GetAnalytics>(
      count_analize_client_, request,
      [&](vision_interfaces::srv::GetAnalytics::Response::SharedPtr response) {
        success = response->success;
        if (success) {
          out_current_count = response->current_count;
          out_class_count   = response->class_count;
          out_total_cumulative = response->total_cumulative;
        }
        RCLCPP_INFO(node_->get_logger(), "AnalyzeCount [%s] %s: %s",
                    target_class.c_str(), success ? "OK" : "FAILED", response->message.c_str());
      },
      timeout, "AnalyzeCount");

  return completed && success;
}

// ── Analyze Size ─────────────────────────────────────────────────────────────

bool Vision::AnalyzeSize(const std::string &target_class, const std::string &filter_mode, int32_t count_limit,
                         vision_interfaces::msg::YoloDetection &out_best_match,
                         std::vector<vision_interfaces::msg::YoloDetection> &out_detections,
                         std::chrono::seconds timeout)
{
  auto request = std::make_shared<vision_interfaces::srv::GetAnalytics::Request>();
  request->action = "get_by_size";
  request->target_class = target_class;
  request->filter_mode = filter_mode; // "big" | "small" | "mixed"
  request->count_limit = count_limit;

  bool success = false;
  bool completed = call_service<vision_interfaces::srv::GetAnalytics>(
      size_analize_client_, request,
      [&](vision_interfaces::srv::GetAnalytics::Response::SharedPtr response) {
        success = response->success;
        if (success && response->has_best_match) {
          out_best_match = response->best_match;
          out_detections = response->detections;
        }
        RCLCPP_INFO(node_->get_logger(), "AnalyzeSize [%s - %s] %s: %s",
                    target_class.c_str(), filter_mode.c_str(), success ? "OK" : "FAILED", response->message.c_str());
      },
      timeout, "AnalyzeSize");

  return completed && success;
}

// ── Analyze Color ────────────────────────────────────────────────────────────

bool Vision::AnalyzeColor(const std::string &target_class, const std::string &color,
                          vision_interfaces::msg::YoloDetection &out_best_match,
                          std::chrono::seconds timeout)
{
  auto request = std::make_shared<vision_interfaces::srv::GetAnalytics::Request>();
  request->action = "get_by_color";
  request->target_class = target_class;
  request->color = color;

  bool success = false;
  bool completed = call_service<vision_interfaces::srv::GetAnalytics>(
      color_analize_client_, request,
      [&](vision_interfaces::srv::GetAnalytics::Response::SharedPtr response) {
        success = response->success;
        if (success && response->has_best_match) {
          out_best_match = response->best_match;
        }
        RCLCPP_INFO(node_->get_logger(), "AnalyzeColor [%s - %s] %s: %s",
                    target_class.c_str(), color.c_str(), success ? "OK" : "FAILED", response->message.c_str());
      },
      timeout, "AnalyzeColor");

  return completed && success;
}

// ── Analyze Compare ──────────────────────────────────────────────────────────

bool Vision::AnalyzeCompare(const std::vector<std::string> &compare_classes,
                            std::string &out_winner_class,
                            std::chrono::seconds timeout)
{
  auto request = std::make_shared<vision_interfaces::srv::GetAnalytics::Request>();
  request->action = "compare_count";
  request->compare_classes = compare_classes;

  bool success = false;
  bool completed = call_service<vision_interfaces::srv::GetAnalytics>(
      compare_analize_client_, request,
      [&](vision_interfaces::srv::GetAnalytics::Response::SharedPtr response) {
        success = response->success;
        if (success) {
          out_winner_class = response->winner_class;
        }
        RCLCPP_INFO(node_->get_logger(), "AnalyzeCompare %s: %s",
                    success ? "OK" : "FAILED", response->message.c_str());
      },
      timeout, "AnalyzeCompare");

  return completed && success;
}
