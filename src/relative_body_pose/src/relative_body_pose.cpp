#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std;
using namespace std::chrono_literals;

static const string WORLD_FRAME = "world";

// Eigen::Vector3d toVec3(const Eigen::Vector4d &in)
// {
//     Eigen::Vector4d out;
//     out.head<3>() = in;
//     out[3] = 1;
//     return out;
// }

// Eigen::Vector4d toVec4(const Eigen::Vector3d &in)
// {
//     Eigen::Vector4d out;
//     out.head<3>() = in;
//     out[3] = 1;
//     return out;
// }

Eigen::Vector3d transform_point(const Eigen::Affine3d &transform, const Eigen::Vector3d &point)
{
    Eigen::Vector4d homogeneous_point;
    homogeneous_point.head<3>() = point;
    homogeneous_point[3] = 1;
    return (transform.affine() * homogeneous_point).head<3>();
}

class RelativeBodyPoseNode : public rclcpp::Node
{
public:
    RelativeBodyPoseNode() : Node("relative_body_pose")
    {
        body_pose.setData(Eigen::Affine3d::Identity());
        body_pose.frame_id_ = WORLD_FRAME;

        pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("body_pose", 10);
        point_publisher = create_publisher<geometry_msgs::msg::PointStamped>("body_point", 10);
        update_timer = create_wall_timer(500ms, bind(&RelativeBodyPoseNode::update, this));
    }

private:
    tf2::Stamped<Eigen::Affine3d> body_pose;
    const Eigen::Vector3d body_relative_point = Eigen::Vector3d(0.5, 0.5, -0.2);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher;
    rclcpp::TimerBase::SharedPtr update_timer;

    void update()
    {
        pose_publisher->publish(tf2::toMsg(body_pose));
        point_publisher->publish(tf2::toMsg([this]() {
            tf2::Stamped<Eigen::Vector3d> point;
            point.setData(transform_point(body_pose, body_relative_point));
            point.frame_id_ = WORLD_FRAME;
            return point;
        }()));
        
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<RelativeBodyPoseNode>());
    rclcpp::shutdown();
}
