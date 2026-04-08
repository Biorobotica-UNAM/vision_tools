#include "vision_tools/Vision.hpp"
using namespace std::chrono_literals;

Vision::Vision(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
    // Inicialización de los 4 clientes
    detect_face_client_   = node_->create_client<faces_interfaces::srv::FaceRecog>("/vision/detect_person");
    identify_face_client_ = node_->create_client<faces_interfaces::srv::FaceRecog>("/vision/identify_person");
    describe_face_client_ = node_->create_client<faces_interfaces::srv::FaceRecog>("/vision/describe_person");
    register_face_client_ = node_->create_client<faces_interfaces::srv::FaceRecog>("/vision/register_person");
}

// ---------------------------------------------------------
// Detect Person    
// ---------------------------------------------------------
bool Vision::DetectPerson(const std::string& name)
{
    if (!detect_face_client_->wait_for_service(1000ms)) {
        RCLCPP_ERROR(node_->get_logger(), "Detect service unavailable (/detect_person)");
        return false;
    }

    auto request = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
    request->name = name;

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    bool success = false;
    std::string message;

    auto callback = [&](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future){
        auto response = future.get();
        success = response->success;
        message = response->name;
        done = true;
        cv.notify_one();
    };

    detect_face_client_->async_send_request(request, callback);

    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, 10s, [&]{ return done; })) {
        RCLCPP_ERROR(node_->get_logger(), "Detect service timed out");
        return false;
    }

    if (success)
        RCLCPP_INFO(node_->get_logger(), "Detect OK: %s", message.c_str());
    else
        RCLCPP_ERROR(node_->get_logger(), "Detect ERROR: %s", message.c_str());

    return success;
}

// ---------------------------------------------------------
// Identify Person    
// ---------------------------------------------------------

bool Vision::IdentifyPerson(const std::string& name)
{
    if (!identify_face_client_->wait_for_service(1000ms)) {
        RCLCPP_ERROR(node_->get_logger(), "Identify service unavailable (/identify_person)");
        return false;
    }

    auto request = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
    request->name = name;

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    bool success = false;
    std::string message;

    auto callback = [&](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future){
        auto response = future.get();
        success = response->success;
        message = response->name;
        done = true;
        cv.notify_one();
    };

    identify_face_client_->async_send_request(request, callback);

    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, 10s, [&]{ return done; })) {
        RCLCPP_ERROR(node_->get_logger(), "Identify service timed out");
        return false;
    }

    if (success)
        RCLCPP_INFO(node_->get_logger(), "Identify OK: %s", message.c_str());
    else
        RCLCPP_ERROR(node_->get_logger(), "Identify ERROR: %s", message.c_str());

    return success;
}

// ---------------------------------------------------------
// Describe Person    
// ---------------------------------------------------------

bool Vision::DescribePerson(const std::string& name)
{
    if (!describe_face_client_->wait_for_service(1000ms)) {
        RCLCPP_ERROR(node_->get_logger(), "Describe service unavailable (/describe_person)");
        return false;
    }

    auto request = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
    request->name = name;

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    bool success = false;
    std::string message;

    auto callback = [&](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future){
        auto response = future.get();
        success = response->success;
        message = response->description;
        done = true;
        cv.notify_one();
    };

    describe_face_client_->async_send_request(request, callback);

    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, 10s, [&]{ return done; })) {
        RCLCPP_ERROR(node_->get_logger(), "Describe service timed out");
        return false;
    }

    if (success)
        RCLCPP_INFO(node_->get_logger(), "Describe OK: %s", message.c_str());
    else
        RCLCPP_ERROR(node_->get_logger(), "Describe ERROR: %s", message.c_str());

    return success;
}

// ---------------------------------------------------------
// Register Person    
// ---------------------------------------------------------

bool Vision::RegisterPerson(const std::string& name)
{
    if (!register_face_client_->wait_for_service(1000ms)) {
        RCLCPP_ERROR(node_->get_logger(), "Register service unavailable (/register_person)");
        return false;
    }

    auto request = std::make_shared<faces_interfaces::srv::FaceRecog::Request>();
    request->name = name;

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    bool success = false;
    std::string message;

    auto callback = [&](rclcpp::Client<faces_interfaces::srv::FaceRecog>::SharedFuture future){
        auto response = future.get();
        success = response->success;
        message = response->name;
        done = true;
        cv.notify_one();
    };

    register_face_client_->async_send_request(request, callback);

    std::unique_lock<std::mutex> lock(mtx);
    if (!cv.wait_for(lock, 10s, [&]{ return done; })) {
        RCLCPP_ERROR(node_->get_logger(), "Register service timed out");
        return false;
    }

    if (success)
        RCLCPP_INFO(node_->get_logger(), "Register OK: %s", message.c_str());
    else
        RCLCPP_ERROR(node_->get_logger(), "Register ERROR: %s", message.c_str());

    return success;
}


