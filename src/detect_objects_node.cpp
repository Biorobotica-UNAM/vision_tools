#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "vision_interfaces/srv/find_object.hpp"
#include "vision_interfaces/srv/get_detections.hpp"
#include "vision_interfaces/srv/bbox_to_pose.hpp"
#include "vision_interfaces/msg/yolo_detection.hpp"

#include "vision_interfaces/srv/find_objects_all.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class DetectObjectNode : public rclcpp::Node
{
public:
  DetectObjectNode()
  : Node("detect_object_node")
  {
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    this->declare_parameter<std::string>("yolo_service", "detection");
    this->declare_parameter<std::string>("bbox_to_pose_service", "bbox_to_pose");
    this->declare_parameter<std::string>("find_object_service", "find_object");

    this->declare_parameter<double>("min_confidence", 0.5);
    this->declare_parameter<double>("min_bbox_area", 1200.0);

    this->declare_parameter<double>("service_wait_sec", 2.0);
    this->declare_parameter<double>("yolo_timeout_sec", 3.0);
    this->declare_parameter<double>("bbox_timeout_sec", 3.0);

    yolo_service_ = this->get_parameter("yolo_service").as_string();
    bbox_to_pose_service_ = this->get_parameter("bbox_to_pose_service").as_string();
    find_object_service_ = this->get_parameter("find_object_service").as_string();

    min_confidence_ = this->get_parameter("min_confidence").as_double();
    min_bbox_area_ = this->get_parameter("min_bbox_area").as_double();

    service_wait_sec_ = this->get_parameter("service_wait_sec").as_double();
    yolo_timeout_sec_ = this->get_parameter("yolo_timeout_sec").as_double();
    bbox_timeout_sec_ = this->get_parameter("bbox_timeout_sec").as_double();

    yolo_detection_client_ = this->create_client<vision_interfaces::srv::GetDetections>(
      yolo_service_, rmw_qos_profile_services_default, callback_group_);

    bbox_to_pose_client_ = this->create_client<vision_interfaces::srv::BboxToPose>(
      bbox_to_pose_service_, rmw_qos_profile_services_default, callback_group_);

    find_object_service_server_ = this->create_service<vision_interfaces::srv::FindObject>(
      find_object_service_,
      std::bind(&DetectObjectNode::findObjectService, this, _1, _2),
      rmw_qos_profile_services_default,
      callback_group_);

    find_all_service_server_ = this->create_service<vision_interfaces::srv::FindObjectsAll>(
      "find_objects_all",
      std::bind(&DetectObjectNode::findAllObjectsService, this, _1, _2),
      rmw_qos_profile_services_default,
      callback_group_);

    

    RCLCPP_INFO(this->get_logger(), "Detect object node ready.");
  }

private:
  struct ObjectInfo
  {
    geometry_msgs::msg::Point point;
    std::string frame_id;
    float confidence;
    std::string class_name;
    vision_interfaces::msg::YoloDetection det;
  };

  const std::unordered_map<std::string, std::vector<std::string>> CATEGORY_MAP = {
    {"cuttlery",  {"spoon", "fork", "knife"}},
    {"tableware", {"mug", "cup", "plate"}},
    {"trash",     {"trash_can", "crumpled_paper"}}, // Ajusta según tus nombres de YOLO
    {"other",     {}} // Caso especial
  };

  bool isClassInCategory(const std::string& detected_class, const std::string& requested_category) {
      // 1. Limpieza básica y log de depuración (útil para ver por qué falla)
      // Puedes comentar estos logs después, pero ayudan a ver si llega "spoon " o "Spoon"
      RCLCPP_DEBUG(this->get_logger(), "Comparando detectado: '%s' con pedido: '%s'", 
                  detected_class.c_str(), requested_category.c_str());

      // 2. Coincidencia exacta
      if (detected_class == requested_category) return true;

      // 3. Buscar en el mapa de categorías
      auto it = CATEGORY_MAP.find(requested_category);
      if (it != CATEGORY_MAP.end()) {
          for (const auto& member : it->second) {
              if (member == detected_class) return true;
          }
      }

      // 4. Lógica de "other"
      if (requested_category == "other") {
          for (const auto& pair : CATEGORY_MAP) {
              if (pair.first == "other") continue;
              for (const auto& member : pair.second) {
                  if (member == detected_class) return false;
              }
          }
          return true; // No está en ninguna categoría conocida, así que es "other"
      }

      return false;
  }
  void findObjectService(
    const std::shared_ptr<vision_interfaces::srv::FindObject::Request> req,
    std::shared_ptr<vision_interfaces::srv::FindObject::Response> res)
  {
    if (req->class_name.empty()) {
      res->success = false;
      res->message = "Requested class_name is empty";
      return;
    }

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

    std::vector<vision_interfaces::msg::YoloDetection> candidates_2d;

    for (const auto & d : yolo_res->detections) {
      if (!isClassInCategory(d.class_name, req->class_name)) {
          continue;
        }

      if (d.confidence < min_confidence_) {
        continue;
      }

      const double width = d.x_max - d.x_min;
      const double height = d.y_max - d.y_min;
      const double area = width * height;

      if (area < min_bbox_area_) {
        continue;
      }

      candidates_2d.push_back(d);
    }

    if (candidates_2d.empty()) {
      res->success = false;
      res->message = "No valid detections for requested class";
      return;
    }

    std::vector<ObjectInfo> candidates_3d;

    for (const auto & det : candidates_2d) {
      auto obj_opt = requestObject3D(det);
      if (obj_opt.has_value()) {
        candidates_3d.push_back(obj_opt.value());
      }
    }

    if (candidates_3d.empty()) {
      res->success = false;
      res->message = "Detected object(s), but no valid 3D pose found";
      return;
    }

    auto best_it = std::min_element(
      candidates_3d.begin(), candidates_3d.end(),
      [](const ObjectInfo & a, const ObjectInfo & b) {
        return squaredDistance(a.point) < squaredDistance(b.point);
      });

    res->success = true;
    res->message = "Object found";
    res->point.header.frame_id = best_it->frame_id;
    res->point.header.stamp = this->now();
    res->point.point = best_it->point;
    res->confidence = best_it->confidence;
    res->detected_class = best_it->class_name;

    RCLCPP_INFO(
      this->get_logger(),
      "Best object [%s]: x=%.2f y=%.2f z=%.2f conf=%.2f",
      best_it->class_name.c_str(),
      best_it->point.x, best_it->point.y, best_it->point.z,
      best_it->confidence);
  }

  void findAllObjectsService(
    const std::shared_ptr<vision_interfaces::srv::FindObjectsAll::Request> req,
    std::shared_ptr<vision_interfaces::srv::FindObjectsAll::Response> res)
  {
    // 1. Validaciones de servicios (YOLO y BboxToPose)
    if (!yolo_detection_client_->wait_for_service(2s) || !bbox_to_pose_client_->wait_for_service(2s)) {
      res->success = false;
      res->message = "Servicios de visión no disponibles";
      return;
    }

    // 2. Obtener detecciones 2D de YOLO
    auto yolo_req = std::make_shared<vision_interfaces::srv::GetDetections::Request>();
    auto yolo_future = yolo_detection_client_->async_send_request(yolo_req);

    if (yolo_future.wait_for(std::chrono::duration<double>(yolo_timeout_sec_)) != std::future_status::ready) {
      res->success = false;
      res->message = "Timeout en YOLO";
      return;
    }

    auto yolo_res = yolo_future.get();
    
    // 3. Procesar y filtrar cada detección
    for (const auto & d : yolo_res->detections) {
      // Filtrar por clase (si se especificó una)
      if (!req->class_name.empty() && d.class_name != req->class_name) continue;
      
      // Filtrar por confianza y área
      if (d.confidence < min_confidence_) continue;
      double area = (d.x_max - d.x_min) * (d.y_max - d.y_min);
      if (area < min_bbox_area_) continue;

      // 4. Convertir a 3D usando el cliente existente
      auto obj_3d_opt = requestObject3D(d);
      if (obj_3d_opt.has_value()) {
        auto info = obj_3d_opt.value();
        
        // Creamos el mensaje individual
        vision_interfaces::msg::ObjectResult obj_msg;
        obj_msg.point.header.frame_id = info.frame_id;
        obj_msg.point.header.stamp = this->now();
        obj_msg.point.point = info.point;
        obj_msg.confidence = info.confidence;
        obj_msg.detected_class = info.class_name;

        // Lo añadimos al array de la respuesta
        res->objects.push_back(obj_msg);
      }
    }

    if (res->objects.empty()) {
      res->success = false;
      res->message = "No se encontraron objetos que cumplan los criterios";
    } else {
      res->success = true;
      res->message = "Se encontraron " + std::to_string(res->objects.size()) + " objetos";
    }
  }

  std::optional<ObjectInfo> requestObject3D(const vision_interfaces::msg::YoloDetection & det)
  {
    auto bbox_req = std::make_shared<vision_interfaces::srv::BboxToPose::Request>();
    bbox_req->x1 = det.x_min;
    bbox_req->y1 = det.y_min;
    bbox_req->x2 = det.x_max;
    bbox_req->y2 = det.y_max;

    auto bbox_future = bbox_to_pose_client_->async_send_request(bbox_req);

    if (bbox_future.wait_for(std::chrono::duration<double>(bbox_timeout_sec_)) != std::future_status::ready) {
      return std::nullopt;
    }

    auto bbox_res = bbox_future.get();
    if (!bbox_res->success) {
      return std::nullopt;
    }

    ObjectInfo info;
    info.point = bbox_res->pose.pose.position;
    info.frame_id = bbox_res->pose.header.frame_id;
    info.confidence = det.confidence;
    info.class_name = det.class_name;
    info.det = det;
    return info;
  }

  static double squaredDistance(const geometry_msgs::msg::Point & p)
  {
    return p.x * p.x + p.y * p.y + p.z * p.z;
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<vision_interfaces::srv::GetDetections>::SharedPtr yolo_detection_client_;
  rclcpp::Client<vision_interfaces::srv::BboxToPose>::SharedPtr bbox_to_pose_client_;
  rclcpp::Service<vision_interfaces::srv::FindObject>::SharedPtr find_object_service_server_;
  rclcpp::Service<vision_interfaces::srv::FindObjectsAll>::SharedPtr find_all_service_server_;

  std::string yolo_service_;
  std::string bbox_to_pose_service_;
  std::string find_object_service_;

  double min_confidence_;
  double min_bbox_area_;
  double service_wait_sec_;
  double yolo_timeout_sec_;
  double bbox_timeout_sec_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DetectObjectNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}