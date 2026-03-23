#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include <chrono>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "vision_interfaces/msg/yolo_detections.hpp"
#include "vision_interfaces/msg/yolo_detection.hpp"
#include "vision_interfaces/srv/bbox_to_pose.hpp"
#include "vision_interfaces/srv/get_detections.hpp"
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

    yolo_detection_client_ = this->create_client<vision_interfaces::srv::GetDetections>(
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
      100ms, std::bind(&DetectSeatsNode::broadcastTransform, this), callback_group_);

    RCLCPP_INFO(get_logger(), "Detect seats node started.");
  }

private:

  struct SeatInfo{
    geometry_msgs::msg::Point point;
    std::string frame_id;
    bool is_occupied;
  };

  void yoloCallback(const vision_interfaces::msg::YoloDetections::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_dets_ = msg;
  }

  void broadcastTransform()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto now = this->get_clock()->now();

    int free_count = 0;
    int occ_count = 0;

    for (const auto &seat : detected_seats_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now;
      tf.header.frame_id = seat.frame_id;
      
      if (seat.is_occupied) {
        tf.child_frame_id = "occ_seat_" + std::to_string(++occ_count);
      } else {
        tf.child_frame_id = "free_seat_" + std::to_string(++free_count);
      }

      tf.transform.translation.x = seat.point.x;
      tf.transform.translation.y = seat.point.y;
      tf.transform.translation.z = seat.point.z;
      tf.transform.rotation.w = 1.0;

      tf_broadcaster_->sendTransform(tf);
    }
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

    auto yolo_req = std::make_shared<vision_interfaces::srv::GetDetections::Request>();
    auto yolo_future = yolo_detection_client_->async_send_request(yolo_req);
    
    // En un MultiThreadedExecutor, podemos esperar el future sin bloquear todo el nodo
    if (yolo_future.wait_for(3s) != std::future_status::ready) {
      res->success = false;
      res->message = "YOLO call timeout";
      return;
    }

    // 2. Procesar detecciones
    auto yolo_res = yolo_future.get();
      if (!yolo_res->success || yolo_res->detections.empty()) {
        res->success = false;
        res->message = "No se detectó nada en la imagen";
        return;
      }

    std::vector<SeatInfo> new_seats;
    auto detections = yolo_res->detections;
    std::vector<vision_interfaces::msg::YoloDetection> persons;
    std::vector<vision_interfaces::msg::YoloDetection> seats;

    for (const auto &d : detections) {
      if (d.class_name == "person") persons.push_back(d);
      else if (d.class_name == "chair" || d.class_name == "couch") seats.push_back(d);
    }

    for (const auto &seat_msg : seats) {
      SeatInfo info;
      info.is_occupied = false;

      // Lógica de solapamiento
      for (const auto &p : persons) {
        if (calculateOverlap(p, seat_msg) > 0.30) {
          info.is_occupied = true;
          break;
        }
      }

      // 3. Obtener pose 3D del asiento libre
      auto bbox_req = std::make_shared<vision_interfaces::srv::BboxToPose::Request>();
      bbox_req->x1 = seat_msg.x_min; bbox_req->y1 = seat_msg.y_min;
      bbox_req->x2 = seat_msg.x_max; bbox_req->y2 = seat_msg.y_max;

      if (bbox_to_pose_client_->wait_for_service(1s)) {
        auto bbox_future = bbox_to_pose_client_->async_send_request(bbox_req);
        if (bbox_future.wait_for(1s) == std::future_status::ready) {
          auto bbox_res = bbox_future.get();
          if (bbox_res->success) {
            info.point = bbox_res->pose.pose.position;
            info.frame_id = bbox_res->pose.header.frame_id;
            new_seats.push_back(info);
          }
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      detected_seats_ = new_seats;
    }

    res->success = !new_seats.empty();
    res->message = "Detected " + std::to_string(new_seats.size()) + " seats total.";
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
  rclcpp::Client<vision_interfaces::srv::GetDetections>::SharedPtr yolo_detection_client_;
  rclcpp::Client<vision_interfaces::srv::BboxToPose>::SharedPtr bbox_to_pose_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr check_service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  vision_interfaces::msg::YoloDetections::SharedPtr last_dets_;
  bool has_free_seat_{false};
  geometry_msgs::msg::TransformStamped free_seat_tf_;
  std::vector<SeatInfo> detected_seats_;
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