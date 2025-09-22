#include <rclcpp/rclcpp.hpp>
#include "demo_interface/msg/cin_cout.hpp"
#include <chrono>
#include <regex> // 需要包含这个头文件
using namespace std;
using namespace std::chrono_literals;

class turtle_cin : public rclcpp::Node
{
private:
    double a;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<demo_interface::msg::CinCout>::SharedPtr publisher_; // 发布者的智能指针
public:
    turtle_cin(const string &node_name) : Node(node_name)
    {
        publisher_ = this->create_publisher<demo_interface::msg::CinCout>("turtle_cin", 10);
    }
    void cin_publisher()
    {
        string input;
        regex number_regex("^\\s*[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?\\s*$");
        while (rclcpp::ok())
        {
            getline(cin, input); // 读取整行
            // 使用正则表达式验证输入
            if (regex_match(input, number_regex))
            {
                try
                {
                    a = stod(input);
                    auto msg = demo_interface::msg::CinCout();
                    msg.a = a;
                    publisher_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "发布数字: %f", a);
                }
                catch (const exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "转换错误: %s", e.what());
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "输入 '%s' 包含无效字符，请输入纯数字", input.c_str());
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<turtle_cin>("turtle_cin_node");
    node->cin_publisher();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
