#include <rclcpp/rclcpp.hpp>
using namespace std;

class PersonNode : public rclcpp::Node
{
private:
    string name_;
    int age_;

public:
    PersonNode(const string &node_name, const string &name, const int &age)
        : Node(node_name)
    {
        this->name_ = name;
        this->age_ = age;
    };

    void eat(const string &food_name)
    {
        RCLCPP_INFO(this->get_logger(), "我是%s，%d岁，爱吃%s", this->name_.c_str(), this->age_, food_name.c_str());
    };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 必须
    auto node = make_shared<PersonNode>("person_node", "鱼香", 18);
    RCLCPP_INFO(node->get_logger(), "person_node success");
    node->eat("LLLLROS");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}