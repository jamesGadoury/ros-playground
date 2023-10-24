#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <rcpputils/asserts.hpp>

void example_one()
{
    Eigen::Affine3d body_relative_point = Eigen::Affine3d::Identity();
    body_relative_point.translate(Eigen::Vector3d(0.1, 0.1, -0.2));
    std::cout << "body_relative_point" << std::endl;
    std::cout << body_relative_point.matrix() << std::endl;

    Eigen::Affine3d desired_body_point_pose = Eigen::Affine3d::Identity();
    desired_body_point_pose.translate(Eigen::Vector3d(2, 3, 4));
    desired_body_point_pose.rotate(Eigen::AngleAxisd(tf2Radians(90), Eigen::Vector3d::UnitZ()));
    std::cout << "desired_body_point_pose" << std::endl;
    std::cout << desired_body_point_pose.matrix() << std::endl;

    Eigen::Affine3d desired_body_pose = desired_body_point_pose * body_relative_point.inverse();
    std::cout << "desired_body_pose" << std::endl;
    std::cout << desired_body_pose.matrix() << std::endl;

    Eigen::Affine3d recovered_body_point_pose = desired_body_pose * body_relative_point;
    std::cout << "recovered_body_point_pose" << std::endl;
    std::cout << recovered_body_point_pose.matrix() << std::endl;

    rcpputils::assert_true(recovered_body_point_pose.isApprox(desired_body_point_pose));
}

struct BodyPoseComputation
{
    Eigen::Affine3d body_relative_point = Eigen::Affine3d::Identity();
    Eigen::Affine3d desired_body_point_pose = Eigen::Affine3d::Identity();

    BodyPoseComputation &setBodyRelativePoint(Eigen::Affine3d pose)
    {
        body_relative_point = pose;

        std::cout << "body_relative_point set to: " << std::endl;
        std::cout << body_relative_point.matrix() << std::endl;

        return *this;
    }

    BodyPoseComputation &setDesiredBodyPointPose(Eigen::Affine3d pose)
    {
        desired_body_point_pose = pose;

        std::cout << "desired_body_point_pose set to: " << std::endl;
        std::cout << desired_body_point_pose.matrix() << std::endl;

        return *this;
    }

    Eigen::Affine3d computeDesiredBodyPose()
    {
        Eigen::Affine3d desired_body_pose = desired_body_point_pose * body_relative_point.inverse();

        std::cout << "desired_body_pose computed as: " << std::endl;
        std::cout << desired_body_pose.matrix() << std::endl;

        return desired_body_pose;
    }

    void assertComputedDesiredBodyPoseIsCorrect()
    {
        Eigen::Affine3d recovered_body_point_pose = computeDesiredBodyPose() * body_relative_point;

        std::cout << "recovered_body_point_pose computed as: " << std::endl;
        std::cout << recovered_body_point_pose.matrix() << std::endl;

        rcpputils::assert_true(recovered_body_point_pose.isApprox(desired_body_point_pose));
    }
};

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "--------------------------------------------------------------------" << std::endl;
    std::cout << "example 1" << std::endl;
    BodyPoseComputation()
        .setBodyRelativePoint(
            Eigen::Affine3d(Eigen::Affine3d::Identity()).translate(Eigen::Vector3d(0.1, 0.1, -0.2)))
        .setDesiredBodyPointPose(
            Eigen::Affine3d(Eigen::Affine3d::Identity()).translate(Eigen::Vector3d(2.0, 3.0, 4.0)).rotate(Eigen::AngleAxisd(tf2Radians(90), Eigen::Vector3d::UnitZ())))
        .assertComputedDesiredBodyPoseIsCorrect();

    std::cout << std::endl << "--------------------------------------------------------------------" << std::endl;
    std::cout << "example 2" << std::endl;
    BodyPoseComputation()
        .setBodyRelativePoint(
            Eigen::Affine3d(Eigen::Affine3d::Identity()).translate(Eigen::Vector3d(0.1, 0.1, 0.2)))
        .setDesiredBodyPointPose(
            Eigen::Affine3d(Eigen::Affine3d::Identity()).translate(Eigen::Vector3d(2.0, 3.0, 4.0)).rotate(Eigen::AngleAxisd(tf2Radians(90), Eigen::Vector3d::UnitZ())))
        .assertComputedDesiredBodyPoseIsCorrect();

    std::cout << std::endl << "--------------------------------------------------------------------" << std::endl;
    std::cout << "example 3" << std::endl;
    BodyPoseComputation()
        .setBodyRelativePoint(
            Eigen::Affine3d(Eigen::Affine3d::Identity()).translate(Eigen::Vector3d(0.1, 0.1, -0.2)))
        .setDesiredBodyPointPose(
            Eigen::Affine3d(Eigen::Affine3d::Identity()).translate(Eigen::Vector3d(2.0, 3.0, 4.0)).rotate(Eigen::AngleAxisd(tf2Radians(-90), Eigen::Vector3d::UnitZ())))
        .assertComputedDesiredBodyPoseIsCorrect();

    std::cout << std::endl << "--------------------------------------------------------------------" << std::endl;
    std::cout << "example 4" << std::endl;
    BodyPoseComputation()
        .setBodyRelativePoint(
            Eigen::Affine3d(Eigen::Affine3d::Identity()).translate(Eigen::Vector3d(0.1, 0.1, -0.2)))
        .setDesiredBodyPointPose(
            Eigen::Affine3d(Eigen::Affine3d::Identity()).translate(Eigen::Vector3d(2.0, 3.0, 4.0)).rotate(Eigen::AngleAxisd(tf2Radians(-90), Eigen::Vector3d::UnitZ())).rotate(Eigen::AngleAxisd(tf2Radians(30), Eigen::Vector3d::UnitX())))
        .assertComputedDesiredBodyPoseIsCorrect();
}
