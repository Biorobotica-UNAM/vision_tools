#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "vision_interfaces/srv/find_seat.hpp"
#include "vision_interfaces/srv/bbox_to_pose.hpp"
#include "vision_interfaces/srv/get_detections.hpp"
#include "vision_interfaces/msg/yolo_detection.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

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

    this->declare_parameter<double>("seat_conf_threshold", 0.7);
    this->declare_parameter<double>("person_conf_threshold", 0.5);
    this->declare_parameter<double>("min_seat_bbox_area", 3000.0);
    this->declare_parameter<double>("seat_person_overlap_threshold", 0.30);

    this->declare_parameter<double>("service_wait_sec", 2.0);
    this->declare_parameter<double>("yolo_timeout_sec", 3.0);
    this->declare_parameter<double>("bbox_timeout_sec", 3.0);

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


    RCLCPP_INFO(this->get_logger(), "Detect seats node started.");
  }

private:

  struct SeatInfo{
    geometry_msgs::msg::Point point;
    std::string frame_id;
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

    auto yolo_req = std::make_shared<vision_interfaces::srv::GetDetections::Request>();
    auto yolo_future = yolo_detection_client_->async_send_request(yolo_req);
    
    // En un MultiThreadedExecutor, podemos esperar el future sin bloquear todo el nodo
    if (yolo_future.wait_for(3s) != std::future_status::ready) {
      res->success = false;
      res->message = "YOLO call timeout";
      return;
    }

    // =====================================
    // 1. Get detections from YOLO
    // =====================================
    auto yolo_req = std::make_shared<vision_interfaces::srv::GetDetections::Request>();
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
    // 2. Split detections
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
    // 3. Evaluate free seats
    // =====================================
    std::vector<SeatInfo> free_seats;

    for (const auto & seat_msg : seats) {
      const double width = seat_msg.x_max - seat_msg.x_min;
      const double height = seat_msg.y_max - seat_msg.y_min;
      const double area = width * height;

      if (area < min_seat_bbox_area_) {
        continue;
      }

      bool occupied = false;
      for (const auto & person_msg : persons) {
        if (calculateSeatOverlap(person_msg, seat_msg) > seat_person_overlap_threshold_) {
          occupied = true;
          break;
        }
      }

      if (occupied) {
        continue;
      }

      auto pose_opt = requestSeat3D(seat_msg);
      if (pose_opt.has_value()) {
        free_seats.push_back(pose_opt.value());
      }
    }

    // =====================================
    // 4. Pick nearest free seat
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
    res->point.header.frame_id = best_it->frame_id;
    res->point.header.stamp = this->now();
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
    info.frame_id = bbox_res->pose.header.frame_id;
    return info;
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