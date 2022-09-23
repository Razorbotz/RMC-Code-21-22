#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

#include "AutomationTypes.hpp"


class Automation{
    public:

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveLeftSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveRightSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > dumpBinSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > shoulderSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > excavationArmPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > excavationDrumPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool_<std::allocator<void> >, std::allocator<void> > > servoStatePublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Empty_<std::allocator<void> >, std::allocator<void> > > goPublisher;

    rclcpp::Node::SharedPtr node;
    Position position;
    Quaternion orientationQuaternion;
    EulerAngles orientation;
    float currentLeftSpeed=0;
    float currentRightSpeed=0;

    virtual void automate() = 0;

    void setNode(rclcpp::Node::SharedPtr node);

    void setPosition(Position position);

    void changeSpeed(float left, float right);

    EulerAngles toEulerAngles(Quaternion q); 

    void changeDumpBinSpeed(float speed);

    void changeShoulderSpeed(float speed);

    void changeArmSpeed(float speed);

    void changeDrumSpeed(float speed);

    void setDumpState(bool state);

    void setGo();
};
