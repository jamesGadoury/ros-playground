#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_eigen/tf2_eigen.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

static const std::string WORLD_FRAME = "world";

Eigen::Vector3d transform_point(const Eigen::Affine3d &transform, const Eigen::Vector3d &point)
{
    Eigen::Vector4d homogeneous_point;
    homogeneous_point.head<3>() = point;
    homogeneous_point[3] = 1;
    return (transform.affine() * homogeneous_point).head<3>();
}

class BodyPointTransformToLocation : public rclcpp::Node
{
public:
    BodyPointTransformToLocation() : Node("body_point_transform_to_location")
    {
        initial_body_pose.setData(Eigen::Affine3d::Identity());
        initial_body_pose.translate(Eigen::Vector3d(0.5, 0.5, 0.5));

        initial_body_pose.frame_id_ = WORLD_FRAME;

        initial_pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("initial_body_pose", 10);
        pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("body_pose", 10);
        initial_point_publisher = create_publisher<geometry_msgs::msg::PointStamped>("initial_body_point", 10);
        point_publisher = create_publisher<geometry_msgs::msg::PointStamped>("body_point", 10);
        update_timer = create_wall_timer(100ms, std::bind(&BodyPointTransformToLocation::update, this));
    }

private:
    tf2::Stamped<Eigen::Affine3d> initial_body_pose;
    const Eigen::Vector3d body_relative_point = Eigen::Vector3d(0.5, 0.5, -0.2);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr initial_point_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher;
    rclcpp::TimerBase::SharedPtr update_timer;

    void update()
    {
        initial_pose_publisher->publish(tf2::toMsg(initial_body_pose));
        initial_point_publisher->publish(tf2::toMsg([this]()
                                            {
            tf2::Stamped<Eigen::Vector3d> point;
            point.setData(transform_point(initial_body_pose, body_relative_point));
            point.frame_id_ = WORLD_FRAME;
            return point; }()));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BodyPointTransformToLocation>());
    rclcpp::shutdown();
}
