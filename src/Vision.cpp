#include "vision_tools/Vision.hpp"


Vision::Vision(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
    detect_by_class_client_ =
            node_->create_client<detector_interfaces::srv::DetectByClass>(
                "/detect_by_class");
}

bool Vision::waitForService(
    const rclcpp::ClientBase::SharedPtr& client,
    const std::chrono::seconds& timeout) const
{
    if (!client->wait_for_service(timeout))
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Service %s not available",
            client->get_service_name());
        return false;
    }
    return true;
}


std::pair<bool, geometry_msgs::msg::PoseStamped> Vision::detectByClass(
    const std::string& class_name, const std::string& model)
{
    geometry_msgs::msg::PoseStamped pose;

    if (!waitForService(detect_by_class_client_, std::chrono::seconds(5)))
        return {false, pose};

    auto request =
        std::make_shared<detector_interfaces::srv::DetectByClass::Request>();

    request->class_name = class_name;
    request->model = model;

    auto future =
        detect_by_class_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "DetectByClass service call failed");
        return {false, pose};
    }

    auto res = future.get();

    pose = res->pose;

    if (!res->success){
        RCLCPP_WARN(node_->get_logger(), res->message.c_str());
        return {false, pose};
    }
    
    return {true, pose};
}

bool Vision::DetectPerson(const std::string& name)
{
    if (!waitForService(detect_person_client_, std::chrono::seconds(5)))
        return false;

    auto request =
        std::make_shared<faces_interfaces::srv::FaceRecog::Request>();

    request->name = name;

    auto future =
        detect_person_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "FaceRecog service call failed");
        return false;
    }

    auto res = future.get();

    if (!res->success){
        RCLCPP_WARN(node_->get_logger(), res->message.c_str());
        return false;
    }
    
    return true;