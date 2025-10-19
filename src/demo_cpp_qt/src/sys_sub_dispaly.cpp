#include <QApplication>
#include <QLabel>
#include <QString>
#include "rclcpp/rclcpp.hpp"
#include "demo_interface/msg/system.hpp"
// #include "ui_widget.h"
//   #include "ui_mainwindow.hpp"
#include "demo_cpp_qt/Widget.h"

using system_msg = demo_interface::msg::System;
using namespace std;

class SysSub : public rclcpp::Node
{
private:
    rclcpp::Subscription<system_msg>::SharedPtr subscriber_;
    // QLabel *label_;
    // QString qmsg;

public:
    Widget w;
    QString qmsg;
    SysSub(const string &node_name) : Node(node_name)
    {

        // label_ = new QLabel();
        subscriber_ = this->create_subscription<system_msg>("system", 10, bind(&SysSub::response_callback, this, placeholders::_1));
        w.setlabel(get_qstr_from_msg(std::make_shared<system_msg>()));
        w.show();
        // label_->setText(get_qstr_from_msg(std::make_shared<system_msg>()));
        // label_->show();
    }
    void response_callback(const system_msg::SharedPtr msg)
    {
        w.setlabel(get_qstr_from_msg(msg));
    }
    QString get_qstr_from_msg(const system_msg::SharedPtr msg)
    {
        stringstream show_str;
        show_str << "节点信息:\t" << msg->node_name << "\t\n"
                 << "状态信息:\t" << msg->result << "\t\n";
        return QString::fromStdString(show_str.str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto node = make_shared<SysSub>("system_sub_node");

    thread spin_thread([&]() -> void
                       { rclcpp::spin(node); });
    spin_thread.detach();
    app.exec();
    rclcpp::shutdown();
    return 0;
}