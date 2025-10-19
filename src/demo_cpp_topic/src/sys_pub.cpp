#include "rclcpp/rclcpp.hpp"
#include "demo_interface/msg/system.hpp"
#include <iostream>
#include <cstdio>
#include <array>
#include <chrono>

using system_msg = demo_interface::msg::System;
using namespace std;
std::vector<std::string> show_node, show_node_old;

class SysPub : public rclcpp::Node
{
private:
    string result = "";
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
        // int a = 1;
        auto msg = system_msg();
        FILE *pipe = popen("ros2 topic list", "r");
        if (!pipe)
        {
            msg.result = "未获取到节点信息";
        }
        else
        {
            while (fgets(buffer.data(), buffer.size(), pipe) != nullptr)
            {
                if (find(show_node.begin(), show_node.end(), buffer.data()) == show_node.end())
                {
                    show_node.push_back(buffer.data());
                }
                /*if (result != buffer.data())
                {
                    // result = "";
                    result += buffer.data();
                }*/
            }
            if (show_node != show_node_old)
            {
                show_node_old.clear();
                result.clear();
                show_node_old = show_node;
                for (const auto &node_name : show_node_old)
                {
                    result += node_name + '\n';
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
