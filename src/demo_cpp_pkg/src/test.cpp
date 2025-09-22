#include "rclcpp/rclcpp.hpp"
using namespace std;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 必须
    auto node = make_shared<rclcpp::Node>("test_node");
    RCLCPP_INFO(node->get_logger(), "test_node success");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}