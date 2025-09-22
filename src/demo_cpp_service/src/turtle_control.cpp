#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <demo_interface/srv/partol.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <chrono>

using SetparamterResult = rcl_interfaces::msg::SetParametersResult;
using Partol = demo_interface::srv::Partol;
using namespace std;
using namespace std::chrono_literals;

class turtle_control : public rclcpp::Node
{
private:
    OnSetParametersCallbackHandle::SharedPtr paramter_callback_handle_;
    rclcpp::Service<Partol>::SharedPtr partol_service_;
    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // 发布者的智能指针
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    double target_x_ = 1.0, target_y_ = 1.0, k_ = 1.0, max_speed_ = 3.0; // k是比例系数

public:
    turtle_control(const string &node_name) : Node(node_name)
    {
        this->declare_parameter("k", 1.0);
        this->declare_parameter("max_speed", 1.0);
        this->get_parameter("k", k_);
        this->get_parameter("max_speed", max_speed_);
        paramter_callback_handle_ = this->add_on_set_parameters_callback(bind(&turtle_control::parameters_callback, this, placeholders::_1));
        partol_service_ = this->create_service<Partol>("Partol", bind(&turtle_control::service_callback, this, placeholders::_1, placeholders::_2));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, bind(&turtle_control::response_callback, this, placeholders::_1));
        // timer_ = this->create_wall_timer(1000ms, bind(&turtle_control::timer_callback, this));
    }
    // 更新参数的回调函数
    rcl_interfaces::msg::SetParametersResult parameters_callback(const vector<rclcpp::Parameter> &parameter)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &parameter : parameter)
        {
            RCLCPP_INFO(this->get_logger(), "更新参数的值%s=%f,%d", parameter.get_name().c_str(), parameter.as_double(), result.successful);
            if (parameter.get_name() == "k")
            {
                k_ = parameter.as_double();
            }
            if (parameter.get_name() == "max_speed")
            {
                max_speed_ = parameter.as_double();
            }
        }
        return result;
    }
    void service_callback(const Partol::Request::SharedPtr request, Partol::Response::SharedPtr response)
    {
        if ((0 < request->target_x && request->target_x < 12.0f) && (0 < request->target_y && request->target_y < 12.0f))
        {
            this->target_x_ = request->target_x;
            this->target_y_ = request->target_y;
            response->result = Partol::Response::SUCCESS;
        }
        else
        {
            response->result = Partol::Response::FAIL;
        }
    }
    void response_callback(const turtlesim::msg::Pose::SharedPtr pose) // 参数: 收到数据的共享指针
    {
        auto current_x = pose->x;
        auto current_y = pose->y;
        auto current_angle = pose->theta;
        RCLCPP_INFO(get_logger(), "当前：x=%f,当前：y=%f,当前：angle=%f", current_x, current_y, current_angle);

        auto distance = sqrt((target_x_ - current_x) * (target_x_ - current_x) + (target_y_ - current_y) * (target_y_ - current_y));
        auto angle = atan2((target_y_ - current_y), (target_x_ - current_x)) - pose->theta;

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
    auto node = make_shared<turtle_control>("turtle_controller");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
