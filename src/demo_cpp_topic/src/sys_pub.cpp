#include "rclcpp/rclcpp.hpp"
#include "demo_interface/msg/system.hpp"
#include <iostream>
#include <cstdio>
#include <array>
#include <chrono>

using system_msg = demo_interface::msg::System;
using namespace std;

class SysPub : public rclcpp::Node
{
private:
    string result = "无节点";
    array<char, 128> buffer;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<system_msg>::SharedPtr publisher_; // 发布者的智能指针
public:
    SysPub(const string &node_name) : Node(node_name)
    {
        publisher_ = this->create_publisher<system_msg>("system", 10);
        timer_ = this->create_wall_timer(1s, bind(&SysPub::timer_callback, this));
    }
    void timer_callback()
    {
        int a = 1;
        auto msg = system_msg();
        FILE *pipe = popen("ros2 node list", "r");
        if (!pipe)
        {
            msg.result = "未获取到节点信息";
        }
        else
        {
            while (fgets(buffer.data(), buffer.size(), pipe) != nullptr)
            {
                if (result != buffer.data())
                {
                    // result = "";
                    result = buffer.data();
                }
            }
            msg.node_name = result;
            msg.result = "获取到节点信息";
        }
        publisher_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<SysPub>("system_pub_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
