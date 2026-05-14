#include <chrono>
#include <memory>
#include <string>
#include <future>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
// Interfaces estándar para todos los modelos de visión
#include "vision_interfaces/srv/get_detections.hpp" 
#include "vision_interfaces/srv/bbox_to_pose.hpp"
#include "vision_interfaces/srv/detect_by_class.hpp"

using vision_interfaces::srv::GetDetections;
using vision_interfaces::srv::BboxToPose;
using vision_interfaces::srv::DetectByClass;

class DetectFusionNode : public rclcpp::Node
{
public:
  DetectFusionNode() : Node("detect_fusion_node")
  {
    declare_parameter<double>("timeout_sec", 5.0);
    timeout_ = std::chrono::duration<double>(get_parameter("timeout_sec").as_double());

    // Ahora ambos usan el mismo tipo de servicio: GetDetections
    detector_services_ = {
      {"YOLO", "/yolo/get_detections"},
      {"OWL",  "/owl/get_detections"} 
    };

    pc_srv_name_ = "/bbox_to_pose";

    cbg_srv_     = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cbg_clients_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Inicialización dinámica de clientes
    for (const auto& [model, service_name] : detector_services_){
      detectors_[model] = this->create_client<GetDetections>(service_name, rclcpp::ServicesQoS(), cbg_clients_);
      RCLCPP_INFO(get_logger(), "Cliente listo para %s en: %s", model.c_str(), service_name.c_str());
    }

    pc_client_ = this->create_client<BboxToPose>(pc_srv_name_, rclcpp::ServicesQoS(), cbg_clients_);

    srv_ = this->create_service<DetectByClass>(
      "/detect_by_class",
      std::bind(&DetectFusionNode::handle, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), cbg_srv_);
  }

private:
  void handle(const std::shared_ptr<DetectByClass::Request> req,
              std::shared_ptr<DetectByClass::Response> res)
  {
    res->success = false;

    // Buscar si el modelo solicitado (YOLO o OWL) existe en nuestro mapa
    auto it = detectors_.find(req->model);
    if (it == detectors_.end()) {
      res->message = "Modelo desconocido: " + req->model;
      return;
    }

    auto detector_client = it->second;

    if (!detector_client->wait_for_service(std::chrono::seconds(1)) || 
        !pc_client_->wait_for_service(std::chrono::seconds(1))) {
      res->message = "Servicios de visión no disponibles";
      return;
    }

    // 1. Llamada al detector (Estandarizado)
    auto dreq = std::make_shared<GetDetections::Request>();
    dreq->queries.push_back(req->class_name); // 

    auto d_future = detector_client->async_send_request(dreq);
    if (d_future.wait_for(timeout_) != std::future_status::ready) {
      res->message = "Timeout en el detector " + req->model;
      return;
    }

    auto dres = d_future.get();
    if (!dres->success || dres->detections.empty()) {
      res->message = "No se encontraron objetos con " + req->model;
      return;
    }

    // 2. Procesar la mejor detección (YoloDetection.msg) [cite: 3]
    // Tomamos la primera de la lista 
    auto &top_det = dres->detections[0]; 

    auto preq = std::make_shared<BboxToPose::Request>();
    // Asignamos coordenadas de la detección al servicio de Pose 
    preq->x1 = static_cast<int32_t>(top_det.x_min);
    preq->y1 = static_cast<int32_t>(top_det.y_min);
    preq->x2 = static_cast<int32_t>(top_det.x_max);
    preq->y2 = static_cast<int32_t>(top_det.y_max);
    preq->header.stamp = this->get_clock()->now();
    preq->header.frame_id = "camera_color_optical_frame"; 

    // 3. Obtener Pose 3D
    auto p_future = pc_client_->async_send_request(preq);
    if (p_future.wait_for(timeout_) != std::future_status::ready) {
      res->message = "Timeout en BboxToPose";
      return;
    }

    auto pres = p_future.get();
    if (!pres->success) {
      res->message = "Error calculando Pose 3D";
      return;
    }

    res->pose = pres->pose; // [cite: 1]
    res->success = true;
    res->message = "Éxito con " + req->model;
  }

  std::chrono::duration<double> timeout_;
  rclcpp::CallbackGroup::SharedPtr cbg_srv_;
  rclcpp::CallbackGroup::SharedPtr cbg_clients_;
  std::string pc_srv_name_;
  std::unordered_map<std::string, std::string> detector_services_;
  std::unordered_map<std::string, rclcpp::Client<GetDetections>::SharedPtr> detectors_;
  rclcpp::Client<BboxToPose>::SharedPtr pc_client_;
  rclcpp::Service<DetectByClass>::SharedPtr srv_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectFusionNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}