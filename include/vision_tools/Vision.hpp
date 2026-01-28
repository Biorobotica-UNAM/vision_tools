#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <std_srvs/srv/trigger.hpp>
#include "detector_interfaces/srv/detect_by_class.hpp"


#include <string>
#include <memory>
#include <chrono>


class Vision
{
public:
    explicit Vision(const rclcpp::Node::SharedPtr& node);

    // Detect object by class name and return success and its pose using PointCloud data
    std::pair<bool, geometry_msgs::msg::PoseStamped>
        detectByClass(const std::string& class_name, const std::string& model);

private:

    bool waitForService(
        const rclcpp::ClientBase::SharedPtr& client,
        const std::chrono::seconds& timeout) const;

    // Clients
    rclcpp::Client<detector_interfaces::srv::DetectByClass>::SharedPtr
        detect_by_class_client_;

    rclcpp::Node::SharedPtr node_;

};