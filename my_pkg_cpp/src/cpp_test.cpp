#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("cpp_test")
    {
        RCLCPP_INFO(this->get_logger(), "Lol");
        this->timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyNode::timerCb, this));
    }

private:
    void timerCb()
    {
        RCLCPP_INFO(this->get_logger(), "Count: %d", this->counter++);
    }

    int counter = 0;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
}
