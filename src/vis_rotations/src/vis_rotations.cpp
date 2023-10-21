#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std;
using namespace std::chrono_literals;

static const string WORLD_FRAME = "world";

class RotationsVisPublisher : public rclcpp::Node
{
public:
    RotationsVisPublisher() : Node("rotations_vis_publisher")
    {
        pose.setData(Eigen::Affine3d::Identity());
        pose.frame_id_ = WORLD_FRAME;
        pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
        update_timer = create_wall_timer(500ms, bind(&RotationsVisPublisher::update, this));
    }

private:
    tf2::Stamped<Eigen::Affine3d> pose;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::TimerBase::SharedPtr update_timer;

    void update()
    {
        pose.rotate(Eigen::AngleAxisd(tf2Radians(90.0), Eigen::Vector3d::UnitZ()));
        pose_publisher->publish(tf2::toMsg(pose));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<RotationsVisPublisher>());
    rclcpp::shutdown();
}
