#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std;
using namespace std::chrono_literals;

static const string WORLD_FRAME = "world";

geometry_msgs::msg::PoseStamped generate_default_pose_message()
{
    auto message = geometry_msgs::msg::PoseStamped();
    message.header.frame_id = WORLD_FRAME;

    message.pose.position.x = 0;
    message.pose.position.y = 0;
    message.pose.position.z = 0;

    return message;
}

class RotationsVisPublisher : public rclcpp::Node
{
public:
    RotationsVisPublisher() : Node("rotations_vis_publisher")
    {
        pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
        update_timer = create_wall_timer(500ms, bind(&RotationsVisPublisher::update, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::TimerBase::SharedPtr update_timer;

    void update()
    {
        pose_publisher->publish(geometry_msgs::msg::PoseStamped());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<RotationsVisPublisher>());
    rclcpp::shutdown();
}
