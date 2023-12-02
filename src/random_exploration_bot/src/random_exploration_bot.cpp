#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RandomExploration : public rclcpp::Node
{
public:
    RandomExploration() : Node("random_exploration")
    {
        timer_ = create_wall_timer(1s, std::bind(&RandomExploration::velocity_publisher_timer_callback, this));
        publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        gen_ = std::mt19937(42);
    }

private:
    void velocity_publisher_timer_callback()
    {
        std::uniform_int_distribution<> distribution(1, 3);
        auto message = geometry_msgs::msg::Twist();

        const int choice = distribution(gen_);
        assert(choice >= 1 && choice <= 3);

        if (choice == 1)
        {
            message.linear.x = 0.22;
        }
        else if (choice == 2)
        {
            message.angular.z = 0.1;
        }
        else if (choice == 3)
        {
            message.angular.z = -0.1;
        }
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    std::mt19937 gen_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomExploration>());
    rclcpp::shutdown();
    return 0;
}
