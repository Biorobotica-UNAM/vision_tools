#include "vision_tools/Vision.hpp"
using namespace std::chrono_literals;

Vision::Vision(const rclcpp::Node::SharedPtr &node) : node_(node) {
    // --- 1. Inicialización de Clientes de Rostros ---
    detect_face_client_ = node_->create_client<faces_interfaces::srv::FaceRecog>(
        "/vision/detect_person");
    identify_face_client_ = node_->create_client<faces_interfaces::srv::FaceRecog>(
        "/vision/identify_person");
    describe_face_client_ = node_->create_client<faces_interfaces::srv::FaceRecog>(
        "/vision/describe_person");
    register_face_client_ = node_->create_client<faces_interfaces::srv::FaceRecog>(
        "/vision/register_person");

    // --- 2. Inicialización de Cliente de Objetos (YOLO/PointCloud) ---
    detect_by_class_client_ = node_->create_client<vision_interfaces::srv::DetectByClass>(
        "/vision/detect_by_class");

    // --- 3. Suscriptor para Alineación de Mesa ---
    sub_table_alignment_ = node_->create_subscription<std_msgs::msg::Float64>(
        "camera/table_alignment", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(promise_mutex_);
            if (alignment_promise_) {
                alignment_promise_->set_value(msg->data);
                alignment_promise_ = nullptr; // Cumplir solo una vez por solicitud
            }
        });
}

// ---------------------------------------------------------
// Table Alignment (Sync wrapper for topic)
// ---------------------------------------------------------
double Vision::getTableAlignment() {
    auto prom = std::make_shared<std::promise<double>>();
    auto fut = prom->get_future();

    {
        std::lock_guard<std::mutex> lock(promise_mutex_);
        alignment_promise_ = prom;
    }

    // Esperamos 1 segundo a que llegue un dato del tópico
    if (fut.wait_for(1s) == std::future_status::ready) {
        return fut.get();
    }

    RCLCPP_WARN(node_->get_logger(), "[TIMEOUT] Table alignment topic not responding");
    return 0.0;
}

// ---------------------------------------------------------
// Detect Object By Class
// ---------------------------------------------------------
std::pair<bool, geometry_msgs::msg::PoseStamped> 
Vision::detectByClass(const std::string& class_name, const std::string& model) {
    if (!detect_by_class_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(node_->get_logger(), "Object detection service unavailable");
        return {false, {}};
    }

    auto request = std::make_shared<vision_interfaces::srv::DetectByClass::Request>();
    request->class_name = class_name;
    request->model = model;

    // Usamos la misma lógica de CV para no bloquear el executor si se llama desde el Worker
    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    std::pair<bool, geometry_msgs::msg::PoseStamped> result = {false, {}};

    auto callback = [&](rclcpp::Client<vision_interfaces::srv::DetectByClass>::SharedFuture future) {
        auto response = future.get();
        result.first = response->success;
        result.second = response->pose;
        done = true;
        cv.notify_one();
    };

    detect_by_class_client_->async_send_request(request, callback);

    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, 5s, [&] { return done; })) {
        RCLCPP_ERROR(node_->get_logger(), "Object detection service timed out");
        return {false, {}};
    }

    return result;
}

// ---------------------------------------------------------
// Person Services (Detect, Identify, Describe, Register)
// ---------------------------------------------------------

bool Vision::DetectPerson(const std::string &name) {
    if (!detect_face_client_->wait_for_service(1000ms)) {
        RCLCPP_ERROR(node_->get_logger(), "Detect service unavailable (/detect_person)");
        return false;
    }
    auto request = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
    request->name = name;

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false, success = false;
    std::string message;

    auto callback = [&](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future) {
        auto response = future.get();
        success = response->success;
        message = response->name;
        done = true;
        cv.notify_one();
    };

    detect_face_client_->async_send_request(request, callback);
    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, 10s, [&] { return done; })) return false;

    if (success) RCLCPP_INFO(node_->get_logger(), "Detect OK: %s", message.c_str());
    return success;
}

bool Vision::IdentifyPerson(const std::string &name) {
    if (!identify_face_client_->wait_for_service(1000ms)) {
        RCLCPP_ERROR(node_->get_logger(), "Identify service unavailable");
        return false;
    }
    auto request = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
    request->name = name;

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false, success = false;
    std::string message;

    auto callback = [&](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future) {
        auto response = future.get();
        success = response->success;
        message = response->name;
        done = true;
        cv.notify_one();
    };

    identify_face_client_->async_send_request(request, callback);
    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, 10s, [&] { return done; })) return false;

    if (success) RCLCPP_INFO(node_->get_logger(), "Identify OK: %s", message.c_str());
    return success;
}

bool Vision::DescribePerson(const std::string &name) {
    if (!describe_face_client_->wait_for_service(1000ms)) return false;
    auto request = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
    request->name = name;

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false, success = false;
    std::string message;

    auto callback = [&](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future) {
        auto response = future.get();
        success = response->success;
        message = response->description;
        done = true;
        cv.notify_one();
    };

    describe_face_client_->async_send_request(request, callback);
    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, 10s, [&] { return done; })) return false;

    return success;
}

bool Vision::RegisterPerson(const std::string &name) {
    if (!register_face_client_->wait_for_service(1000ms)) return false;
    auto request = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
    request->name = name;

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false, success = false;
    std::string message;

    auto callback = [&](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future) {
        auto response = future.get();
        success = response->success;
        message = response->name;
        done = true;
        cv.notify_one();
    };

    register_face_client_->async_send_request(request, callback);
    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, 10s, [&] { return done; })) return false;

    return success;
}