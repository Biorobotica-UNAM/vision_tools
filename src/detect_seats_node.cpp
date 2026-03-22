#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "vision_interfaces/msg/yolo_detections.hpp"
#include "vision_interfaces/msg/yolo_detection.hpp"
#include "vision_interfaces/srv/bbox_to_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

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
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    yolo_detections_sub_ = this->create_subscription<vision_interfaces::msg::YoloDetections>(
      "detections_list", 10,
      std::bind(&DetectSeatsNode::yoloCallback, this, _1),
      sub_opt);

    yolo_detection_client_ = this->create_client<std_srvs::srv::Trigger>(
      "detection", rmw_qos_profile_services_default, callback_group_);
    
    bbox_to_pose_client_ = this->create_client<vision_interfaces::srv::BboxToPose>(
      "bbox_to_pose", rmw_qos_profile_services_default, callback_group_);

    check_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/check_seats",
      std::bind(&DetectSeatsNode::checkSeatsService, this, _1, _2),
      rmw_qos_profile_services_default,
      callback_group_);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    tf_timer_ = this->create_wall_timer(
      100ms,
      std::bind(&DetectSeatsNode::broadcastTransform, this),
      callback_group_);

    RCLCPP_INFO(get_logger(), "Detect seats node started.");
  }

private:
  void yoloCallback(const vision_interfaces::msg::YoloDetections::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_dets_ = msg;
  }

  void broadcastTransform()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!has_free_seat_) return;

    free_seat_tf_.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(free_seat_tf_);
  }

  void checkSeatsService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    RCLCPP_INFO(get_logger(), "Check seats service called.");

    // 1. Llamar a YOLO para refrescar detecciones
    if (!yolo_detection_client_->wait_for_service(2s)) {
      res->success = false;
      res->message = "YOLO service not available";
      return;
    }

    auto yolo_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto yolo_future = yolo_detection_client_->async_send_request(yolo_req);
    
    // En un MultiThreadedExecutor, podemos esperar el future sin bloquear todo el nodo
    if (yolo_future.wait_for(2s) != std::future_status::ready) {
      res->success = false;
      res->message = "YOLO call timeout";
      return;
    }

    // 2. Procesar detecciones
    vision_interfaces::msg::YoloDetections::SharedPtr current_dets;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      current_dets = last_dets_;
    }

    if (!current_dets) {
      res->success = false;
      res->message = "No detections available yet";
      return;
    }

    std::vector<vision_interfaces::msg::YoloDetection> persons;
    std::vector<vision_interfaces::msg::YoloDetection> seats;

    for (const auto &d : current_dets->detections) {
      if (d.class_name == "person") persons.push_back(d);
      else if (d.class_name == "chair" || d.class_name == "couch") seats.push_back(d);
    }

    double best_dist = std::numeric_limits<double>::infinity();
    geometry_msgs::msg::Point best_point;
    bool found_any = false;

    for (const auto &seat : seats) {
      bool occupied = false;
      for (const auto &p : persons) {
        if (calculateOverlap(p, seat) > 0.30) {
          occupied = true;
          break;
        }
      }
      if (occupied) continue;

      // 3. Obtener pose 3D del asiento libre
      auto bbox_req = std::make_shared<vision_interfaces::srv::BboxToPose::Request>();
      bbox_req->x1 = seat.x_min; bbox_req->y1 = seat.y_min;
      bbox_req->x2 = seat.x_max; bbox_req->y2 = seat.y_max;

      if (!bbox_to_pose_client_->wait_for_service(1s)) continue;

      auto bbox_future = bbox_to_pose_client_->async_send_request(bbox_req);
      if (bbox_future.wait_for(1s) == std::future_status::ready) {
        auto bbox_res = bbox_future.get();
        if (bbox_res->success) {
          auto p = bbox_res->pose.pose.position;
          double dist = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
          if (dist < best_dist) {
            best_dist = dist;
            best_point = p;
            found_any = true;
            // Guardamos el frame_id que viene de la cámara
            free_seat_tf_.header.frame_id = bbox_res->pose.header.frame_id;
          }
        }
      }
    }

    if (found_any) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      has_free_seat_ = true;
      free_seat_tf_.child_frame_id = "free_seat";
      free_seat_tf_.transform.translation.x = best_point.x;
      free_seat_tf_.transform.translation.y = best_point.y;
      free_seat_tf_.transform.translation.z = best_point.z;
      free_seat_tf_.transform.rotation.w = 1.0;

      res->success = true;
      res->message = "Seat found at: " + std::to_string(best_point.x);
    } else {
      res->success = false;
      res->message = "No free seats found";
    }
  }

  static double calculateOverlap(const vision_interfaces::msg::YoloDetection& p, const vision_interfaces::msg::YoloDetection& s)
  {
    double ix1 = std::max(p.x_min, s.x_min);
    double iy1 = std::max(p.y_min, s.y_min);
    double ix2 = std::min(p.x_max, s.x_max);
    double iy2 = std::min(p.y_max, s.y_max);
    if (ix2 < ix1 || iy2 < iy1) return 0.0;
    double area_i = (ix2 - ix1) * (iy2 - iy1);
    double area_s = (s.x_max - s.x_min) * (s.y_max - s.y_min);
    return (area_s <= 0) ? 0.0 : area_i / area_s;
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<vision_interfaces::msg::YoloDetections>::SharedPtr yolo_detections_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr yolo_detection_client_;
  rclcpp::Client<vision_interfaces::srv::BboxToPose>::SharedPtr bbox_to_pose_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr check_service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  vision_interfaces::msg::YoloDetections::SharedPtr last_dets_;
  bool has_free_seat_{false};
  geometry_msgs::msg::TransformStamped free_seat_tf_;
  std::mutex data_mutex_; // Para proteger el acceso a variables compartidas
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectSeatsNode>();
  
  // USAR MULTITHREADED EXECUTOR ES CLAVE
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}