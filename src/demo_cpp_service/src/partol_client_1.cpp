#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/partol.hpp>
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include <chrono>
#include <ctime>
using SetP = rcl_interfaces::srv::SetParameters;
using Partol = demo_interface::srv::Partol;
using namespace std;
using namespace std::chrono_literals;

class PartolClient : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Partol>::SharedPtr partol_client_;

public:
    PartolClient(const string &node_name) : Node(node_name)
    {

        srand(time(NULL));
        partol_client_ = this->create_client<Partol>("Partol");
        timer_ = this->create_wall_timer(1s, bind(&PartolClient::wall_timer_callback, this));
    }
    void wall_timer_callback()
    {
        // 检测服务端是否上线
        while (!this->partol_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "等待服务上线过程中，rclcpp挂了");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务上线过程中");
        }
        // 构造请求的对象
        auto request = std::make_shared<Partol::Request>();
        request->target_x = rand() % 15;
        request->target_y = rand() % 15;
        RCLCPP_INFO(this->get_logger(), "准备好目标点%f,%f", request->target_x, request->target_y);
        // 发送请求给服务端（异步请求）
        this->partol_client_->async_send_request(request, bind(&PartolClient::async_send_request_callback, this, placeholders::_1));
    }
    void async_send_request_callback(rclcpp::Client<Partol>::SharedFuture result_future)
    {
        auto response = result_future.get();
        if (response->result == Partol::Response::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "请求巡逻目标点成功");
        }
        else if (response->result == Partol::Response::FAIL)
        {
            RCLCPP_INFO(this->get_logger(), "请求巡逻目标点失败");
        }
    }
    // 创建一个客户端，发送请求，返回结果
    SetP::Response::SharedPtr call_set_parameter(const rcl_interfaces::msg::Parameter &parameter)
    {
        auto parameter_client_ = this->create_client<SetP>("/turtle_controller/set_parameters");
        // 检测服务端是否上线
        while (!parameter_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "等待服务上线过程中，rclcpp挂了");
                return nullptr;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务上线过程中1");
        }
        // 构造请求的对象
        auto request = std::make_shared<SetP::Request>();
        request->parameters.push_back(parameter);
        // 发送请求给服务端（同步请求）
        // this->partol_client_->async_send_request(request, bind(&PartolClient::async_send_request_callback, this, placeholders::_1));
        auto future = parameter_client_->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        auto response = future.get();
        return response;
    }
    // 更新参数
    void update_server_parameter_k(double k)
    {
        // 创建参数对象
        auto parameter = rcl_interfaces::msg::Parameter();
        parameter.name = "k";
        // 创建参数值
        auto parameter_value = rcl_interfaces::msg::ParameterValue();
        parameter_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        parameter_value.double_value = k;
        parameter.value = parameter_value;
        // 请求更新参数并处理
        auto response = this->call_set_parameter(parameter);
        if (response == NULL)
        {
            RCLCPP_INFO(this->get_logger(), "参数更新失败");
            return;
        }
        for (auto result : response->results)
        {
            if (result.successful == false)
            {
                RCLCPP_INFO(this->get_logger(), "参数更新失败,原因：%s,%d", result.reason.c_str(), result.successful);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "参数更新成功");
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<PartolClient>("partol_client_1");
    node->update_server_parameter_k(4.0);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}