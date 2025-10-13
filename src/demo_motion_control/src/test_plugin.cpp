#include "demo_motion_control/motion_control_interface.hpp"
#include "pluginlib/class_loader.hpp"
using namespace std;

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        return 0;
    }
    // 通过命令行参数传入要加载的插件名称
    string controller_name = argv[1];
    // 通过功能包名和基类名创建控制加载器
    pluginlib::ClassLoader<motioncontroller::MotionController> controller_loader("demo_motion_control", "motioncontroller::MotionController");
    // 使用加载器加载指定名称的插件，返回指定插件类的对象指针
    auto controller = controller_loader.createSharedInstance(controller_name);
    // 调用插件的方法
    controller->start();
    controller->stop();
    return 0;
}