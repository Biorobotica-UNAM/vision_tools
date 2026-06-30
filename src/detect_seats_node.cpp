#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>
#include <optional>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "vision_interfaces/srv/find_seat.hpp"
#include "vision_interfaces/srv/bbox_to_pose.hpp"
#include "vision_interfaces/srv/get_detections.hpp"
#include "vision_interfaces/msg/yolo_detection.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
//#include "cv_bridge/cv_bridge.hpp"

#if __has_include(<cv_bridge/cv_bridge.hpp>)
  #include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
  #include <cv_bridge/cv_bridge.h>
#else
  #error "cv_bridge header not found"
#endif
#include <opencv2/opencv.hpp>

#include <opencv2/opencv.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

//stable version seat detection node: detect_seats_node.cpp
class DetectSeatsNode : public rclcpp::Node
{
public:
  DetectSeatsNode()
  : Node("detect_seats_node")
  {
    // Grupo de rellamada reentrante para permitir que el servicio y los clientes corran en paralelo
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto sub_options_ = rclcpp::SubscriptionOptions();
    sub_options_.callback_group = cb_group_;
  
    // =========================
    // Parameters
    // =========================
    this->declare_parameter<std::string>("yolo_service", "detection");
    this->declare_parameter<std::string>("bbox_to_pose_service", "bbox_to_pose");
    this->declare_parameter<std::string>("find_seat_service", "find_seat");

    this->declare_parameter<double>("seat_conf_threshold", 0.6);
    this->declare_parameter<double>("person_conf_threshold", 0.6);
    this->declare_parameter<double>("min_seat_bbox_area", 3000.0);
    this->declare_parameter<double>("seat_person_overlap_threshold", 0.30);

    this->declare_parameter<double>("service_wait_sec", 2.0);
    this->declare_parameter<double>("yolo_timeout_sec", 3.0);
    this->declare_parameter<double>("bbox_timeout_sec", 3.0);

    this->declare_parameter<std::string>("image_topic", "/hardware/camera/image_raw");

    yolo_service_ = this->get_parameter("yolo_service").as_string();
    bbox_to_pose_service_ = this->get_parameter("bbox_to_pose_service").as_string();
    find_seat_service_ = this->get_parameter("find_seat_service").as_string();

    seat_conf_threshold_ = this->get_parameter("seat_conf_threshold").as_double();
    person_conf_threshold_ = this->get_parameter("person_conf_threshold").as_double();
    min_seat_bbox_area_ = this->get_parameter("min_seat_bbox_area").as_double();
    seat_person_overlap_threshold_ = this->get_parameter("seat_person_overlap_threshold").as_double();

    service_wait_sec_ = this->get_parameter("service_wait_sec").as_double();
    yolo_timeout_sec_ = this->get_parameter("yolo_timeout_sec").as_double();
    bbox_timeout_sec_ = this->get_parameter("bbox_timeout_sec").as_double();

    // =========================
    // Clients
    // =========================
    yolo_detection_client_ = this->create_client<vision_interfaces::srv::GetDetections>(
      yolo_service_, rmw_qos_profile_services_default, cb_group_);
    
    bbox_to_pose_client_ = this->create_client<vision_interfaces::srv::BboxToPose>(
      bbox_to_pose_service_, rmw_qos_profile_services_default, cb_group_);

    // =========================
    // Service
    // =========================
    check_service_ = this->create_service<vision_interfaces::srv::FindSeat>(
      find_seat_service_,
      std::bind(&DetectSeatsNode::handleFindSeat, this, _1, _2),
      rmw_qos_profile_services_default, cb_group_);

    // [DEBUG] subscriber de imagen raw + publisher de imagen debug
    std::string image_topic = this->get_parameter("image_topic").as_string();
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(image_mutex_);
        last_image_ = msg;
      },
      sub_options_);

    rclcpp::QoS debug_qos(1);
    debug_qos.reliable();
    debug_qos.transient_local();
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/vision/detect_seats/debug_image", debug_qos);

    RCLCPP_INFO(this->get_logger(), "Detect seats node started.");
  }

private:

  struct SeatInfo{
    geometry_msgs::msg::Point point;
    std_msgs::msg::Header header;
  };

  struct SeatCandidate{
    vision_interfaces::msg::YoloDetection bbox;
    std::string label;
  };

  void handleFindSeat(
    const std::shared_ptr<vision_interfaces::srv::FindSeat::Request> /*req*/,
    std::shared_ptr<vision_interfaces::srv::FindSeat::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Find seat request received.");

    // Check services once
    if (!yolo_detection_client_->wait_for_service(std::chrono::duration<double>(service_wait_sec_))) {
      res->success = false;
      res->message = "YOLO service not available";
      return;
    }

    if (!bbox_to_pose_client_->wait_for_service(std::chrono::duration<double>(service_wait_sec_))) {
      res->success = false;
      res->message = "bbox_to_pose service not available";
      return;
    }

    // =====================================
    // Get detections from YOLO
    // =====================================
    auto yolo_req = std::make_shared<vision_interfaces::srv::GetDetections::Request>();
    yolo_req->queries = {"person", "chair", "couch"};
    auto yolo_future = yolo_detection_client_->async_send_request(yolo_req);

    if (yolo_future.wait_for(std::chrono::duration<double>(yolo_timeout_sec_)) != std::future_status::ready) {
      res->success = false;
      res->message = "YOLO call timeout";
      return;
    }

    auto yolo_res = yolo_future.get();
    if (!yolo_res->success || yolo_res->detections.empty()) {
      res->success = false;
      res->message = "No detections in current image";
      return;
    }

    // =====================================
    // Split detections
    // =====================================
    std::vector<vision_interfaces::msg::YoloDetection> persons;
    std::vector<vision_interfaces::msg::YoloDetection> seats;

    for (const auto & d : yolo_res->detections) {
      if (d.class_name == "person") {
        if (d.confidence >= person_conf_threshold_) {
          persons.push_back(d);
        }
      } else if (d.class_name == "chair" || d.class_name == "couch") {
        if (d.confidence >= seat_conf_threshold_) {
          seats.push_back(d);
        }
      }
    }

    if (seats.empty()) {
      res->success = false;
      res->message = "No seat candidates detected";
      return;
    }

    // =====================================
    // Evaluate free seats
    // =====================================
    std::vector<SeatCandidate> seat_candidates;
    std::vector<SeatInfo> free_seats;

    for (const auto & seat_msg : seats) {
      const double width = seat_msg.x_max - seat_msg.x_min;
      const double height = seat_msg.y_max - seat_msg.y_min;
      const double area = width * height;

      if (width <= 0.0 || height <= 0.0 || area < min_seat_bbox_area_) {
        continue;
      }

      auto candidates = buildSeatCandidates(seat_msg);
      seat_candidates.insert(seat_candidates.end(), candidates.begin(), candidates.end());
    }

    if (seat_candidates.empty()) {
      res->success = false;
      res->message = "No valid seat candidates detected";
      return;
    }

    sensor_msgs::msg::Image::SharedPtr image_snapshot;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      image_snapshot = last_image_;
    }

    cv::Mat debug_img;
    bool has_debug_img = false;
    if (image_snapshot) {
      try {
        auto cv_ptr = cv_bridge::toCvCopy(image_snapshot, "rgb8");
        debug_img = cv_ptr->image.clone();
        has_debug_img = true;
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "[DEBUG] No se pudo convertir imagen para debug");
      }
    }

    for (const auto & candidate : seat_candidates) {
      const auto & seat_msg = candidate.bbox;
      const double width = seat_msg.x_max - seat_msg.x_min;
      const double height = seat_msg.y_max - seat_msg.y_min;

      bool occupied = false;
      for (const auto & person_msg : persons) {
        if (calculateSeatOverlap(person_msg, seat_msg) > seat_person_overlap_threshold_) {
          occupied = true;
          break;
        }
      }

      if (has_debug_img) {
        cv::Rect seat_rect(
          static_cast<int>(seat_msg.x_min),
          static_cast<int>(seat_msg.y_min),
          static_cast<int>(width),
          static_cast<int>(height));
        cv::Scalar color = occupied ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        std::string label = candidate.label + (occupied ? " OCCUPIED" : " FREE");
        cv::rectangle(debug_img, seat_rect, color, 2);
        cv::putText(debug_img, label,
          cv::Point(seat_rect.x, seat_rect.y - 6),
          cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
      }

      if (occupied) {
        continue;
      }

      auto pose_opt = requestSeat3D(seat_msg);
      if (pose_opt.has_value()) {
        free_seats.push_back(pose_opt.value());
      }
    }
    // verde = asiento libre, rojo = ocupado, azul = persona
    if (has_debug_img) {
      for (const auto & person_msg : persons) {
        cv::Rect person_rect(
          static_cast<int>(person_msg.x_min),
          static_cast<int>(person_msg.y_min),
          static_cast<int>(person_msg.x_max - person_msg.x_min),
          static_cast<int>(person_msg.y_max - person_msg.y_min));
        cv::rectangle(debug_img, person_rect, cv::Scalar(255, 0, 0), 2);
        cv::putText(debug_img, "PERSON",
          cv::Point(person_rect.x, person_rect.y - 6),
          cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
      }

      auto out_msg = cv_bridge::CvImage(image_snapshot->header, "rgb8", debug_img).toImageMsg();
      debug_image_pub_->publish(*out_msg);
    }

    // =====================================
    // Pick nearest free seat
    // =====================================
    if (free_seats.empty()) {
      res->success = false;
      res->message = "No free seats detected in this view";
      return;
    }

    auto best_it = std::min_element(
      free_seats.begin(), free_seats.end(),
      [](const SeatInfo & a, const SeatInfo & b) {
        const double da = squaredDistance(a.point);
        const double db = squaredDistance(b.point);
        return da < db;
      });

    res->success = true;
    res->message = "Free seat found";
    res->point.header = best_it->header;
    res->point.point = best_it->point;

    RCLCPP_INFO(
      this->get_logger(),
      "Best seat: x=%.2f y=%.2f z=%.2f",
      best_it->point.x, best_it->point.y, best_it->point.z);
  }

  std::optional<SeatInfo> requestSeat3D(const vision_interfaces::msg::YoloDetection & seat_msg)
  {
    auto bbox_req = std::make_shared<vision_interfaces::srv::BboxToPose::Request>();
    bbox_req->x1 = seat_msg.x_min;
    bbox_req->y1 = seat_msg.y_min;
    bbox_req->x2 = seat_msg.x_max;
    bbox_req->y2 = seat_msg.y_max;

    auto bbox_future = bbox_to_pose_client_->async_send_request(bbox_req);

    if (bbox_future.wait_for(std::chrono::duration<double>(bbox_timeout_sec_)) != std::future_status::ready) {
      return std::nullopt;
    }

    auto bbox_res = bbox_future.get();
    if (!bbox_res->success) {
      return std::nullopt;
    }

    SeatInfo info;
    info.point = bbox_res->pose.pose.position;
    info.header = bbox_res->pose.header;
    return info;
  }

  static std::vector<SeatCandidate> buildSeatCandidates(
    const vision_interfaces::msg::YoloDetection & seat_msg)
  {
    if (seat_msg.class_name != "couch") {
      return {SeatCandidate{seat_msg, "CHAIR"}};
    }

    std::vector<SeatCandidate> candidates;
    candidates.reserve(3);

    const double width = seat_msg.x_max - seat_msg.x_min;
    const char * labels[3] = {"COUCH_L", "COUCH_C", "COUCH_R"};

    for (int i = 0; i < 3; ++i) {
      auto segment = seat_msg;
      segment.x_min = seat_msg.x_min + width * static_cast<double>(i) / 3.0;
      segment.x_max = (i == 2)
        ? seat_msg.x_max
        : seat_msg.x_min + width * static_cast<double>(i + 1) / 3.0;

      if (segment.x_max > segment.x_min) {
        candidates.push_back(SeatCandidate{segment, labels[i]});
      }
    }

    return candidates;
  }

  static double calculateSeatOverlap(
    const vision_interfaces::msg::YoloDetection & person,
    const vision_interfaces::msg::YoloDetection & seat)
  {
    const double ix1 = std::max(person.x_min, seat.x_min);
    const double iy1 = std::max(person.y_min, seat.y_min);
    const double ix2 = std::min(person.x_max, seat.x_max);
    const double iy2 = std::min(person.y_max, seat.y_max);

    if (ix2 <= ix1 || iy2 <= iy1) {
      return 0.0;
    }

    const double inter_area = (ix2 - ix1) * (iy2 - iy1);
    const double seat_area = (seat.x_max - seat.x_min) * (seat.y_max - seat.y_min);

    if (seat_area <= 0.0) {
      return 0.0;
    }

    return inter_area / seat_area;
  }

  static double squaredDistance(const geometry_msgs::msg::Point & p)
  {
    return p.x * p.x + p.y * p.y + p.z * p.z;
  }

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Client<vision_interfaces::srv::GetDetections>::SharedPtr yolo_detection_client_;
  rclcpp::Client<vision_interfaces::srv::BboxToPose>::SharedPtr bbox_to_pose_client_;
  rclcpp::Service<vision_interfaces::srv::FindSeat>::SharedPtr check_service_;

  std::string yolo_service_;
  std::string bbox_to_pose_service_;
  std::string find_seat_service_;

  double seat_conf_threshold_;
  double person_conf_threshold_;
  double min_seat_bbox_area_;
  double seat_person_overlap_threshold_;

  double service_wait_sec_;
  double yolo_timeout_sec_;
  double bbox_timeout_sec_;

  // DEBUG
  std::mutex image_mutex_;
  sensor_msgs::msg::Image::SharedPtr last_image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DetectSeatsNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
