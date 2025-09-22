#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <chrono>
using namespace std;
using namespace std::chrono_literals;

class turtle_control : public rclcpp::Node
{
private:
    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // 发布者的智能指针
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    double target_x_ = 1.0, targer_y_ = 1.0, k_ = 1.0, max_speed_ = 3.0; // k是比例系数

public:
    turtle_control(const string &node_name) : Node(node_name)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, bind(&turtle_control::response_callback, this, placeholders::_1));
        // timer_ = this->create_wall_timer(1000ms, bind(&turtle_control::timer_callback, this));
    }
    void response_callback(const turtlesim::msg::Pose::SharedPtr pose) // 参数: 收到数据的共享指针
    {
        auto current_x = pose->x;
        auto current_y = pose->y;
        auto current_angle = pose->theta;
        RCLCPP_INFO(get_logger(), "当前：x=%f,当前：y=%f,当前：angle=%f", current_x, current_y, current_angle);

        auto distance = sqrt((target_x_ - current_x) * (target_x_ - current_x) + (targer_y_ - current_y) * (targer_y_ - current_y));
        auto angle = atan2((targer_y_ - current_y), (target_x_ - current_x)) - pose->theta;

        auto msg = geometry_msgs::msg::Twist();
        if (distance > 0.1)
        {
            if (fabs(angle) > 0.2)
            {
                msg.angular.z = fabs(angle);
            }
            else
            {
                msg.linear.x = k_ * distance;
            }
        }
        if (msg.linear.x > max_speed_)
        {
            msg.linear.x = max_speed_;
        }
        publisher_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<turtle_control>("turtle_control_node");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
