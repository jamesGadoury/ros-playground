#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

static const std::string WORLD_FRAME = "world";

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
        update_timer = create_wall_timer(500ms, std::bind(&RelativeBodyPoseNode::update, this));

        makeButton("RotateByXAxis", {0, 0, 2}, std::bind(&RelativeBodyPoseNode::rotateByXButtonClick, this, _1));
        makeButton("RotateByYAxis", {0, 0.5, 2}, std::bind(&RelativeBodyPoseNode::rotateByYButtonClick, this, _1));
        makeButton("RotateByZAxis", {0, 1, 2}, std::bind(&RelativeBodyPoseNode::rotateByZButtonClick, this, _1));
        interactive_marker_server->applyChanges();
    }

private:
    tf2::Stamped<Eigen::Affine3d> body_pose;
    const Eigen::Vector3d body_relative_point = Eigen::Vector3d(0.5, 0.5, -0.2);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher;
    rclcpp::TimerBase::SharedPtr update_timer;

    std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server =
        std::make_unique<interactive_markers::InteractiveMarkerServer>("interactive_markers", get_node_base_interface(),
                                                                       get_node_clock_interface(),
                                                                       get_node_logging_interface(),
                                                                       get_node_topics_interface(),
                                                                       get_node_services_interface());

    void update()
    {
        pose_publisher->publish(tf2::toMsg(body_pose));
        point_publisher->publish(tf2::toMsg([this]()
                                            {
            tf2::Stamped<Eigen::Vector3d> point;
            point.setData(transform_point(body_pose, body_relative_point));
            point.frame_id_ = WORLD_FRAME;
            return point; }()));
    }

    void rotateByXButtonClick(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        if (feedback->event_type == feedback->BUTTON_CLICK)
        {
            RCLCPP_INFO(get_logger(), "Rotate By X Axis Button Clicked...");
            body_pose.rotate(Eigen::AngleAxisd(tf2Radians(15), Eigen::Vector3d::UnitX()));
        }
    }

    void rotateByYButtonClick(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        if (feedback->event_type == feedback->BUTTON_CLICK)
        {
            RCLCPP_INFO(get_logger(), "Rotate By Y Axis Button Clicked...");
            body_pose.rotate(Eigen::AngleAxisd(tf2Radians(15), Eigen::Vector3d::UnitY()));
        }
    }

    void rotateByZButtonClick(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        if (feedback->event_type == feedback->BUTTON_CLICK)
        {
            RCLCPP_INFO(get_logger(), "Rotate By Z Axis Button Clicked...");
            body_pose.rotate(Eigen::AngleAxisd(tf2Radians(15), Eigen::Vector3d::UnitZ()));
        }
    }
    //! @todo should I just use tf2 vectors or eigen...?
    void makeButton(const std::string &name, const tf2::Vector3 &position, const interactive_markers::InteractiveMarkerServer::FeedbackCallback &callback)
    {
        visualization_msgs::msg::InteractiveMarker button;
        button.header.frame_id = WORLD_FRAME;
        button.pose.position.x = position.getX();
        button.pose.position.y = position.getY();
        button.pose.position.z = position.getZ();
        button.scale = 0.5;

        button.name = name;
        button.description = name;

        visualization_msgs::msg::InteractiveMarkerControl control;

        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
        control.name = "button_control";

        visualization_msgs::msg::Marker marker = [&button]()
        {
            visualization_msgs::msg::Marker marker;

            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = button.scale * 0.45;
            marker.scale.y = button.scale * 0.45;
            marker.scale.z = button.scale * 0.45;
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
            marker.color.a = 1.0;

            return marker;
        }();
        control.markers.push_back(marker);
        control.always_visible = true;
        button.controls.push_back(control);

        interactive_marker_server->insert(button);
        interactive_marker_server->setCallback(button.name, callback);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelativeBodyPoseNode>());
    rclcpp::shutdown();
}
