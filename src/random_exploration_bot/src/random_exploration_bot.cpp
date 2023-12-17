#include <memory>
#include <random>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RandomExploration : public rclcpp::Node
{
public:
    RandomExploration() : Node("random_exploration")
    {
        publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        subscription_ = create_subscription<sensor_msgs::msg::LaserScan>("lidar", 10, std::bind(&RandomExploration::laser_scan_callback, this, _1));
        last_update_ = std::chrono::steady_clock::now();
        data_out_ = std::ofstream([this]()
                                  {
            std::stringstream ss;
            ss << "random_exploration_data_" << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ".csv";
            return ss.str(); }());
        gen_ = std::mt19937(42);

        data_out_ << "timestamp_ns,update_interval_ns,angle_min,angle_max,angle_increment,time_increment,scan_time,range_min,range_max";
        for (int i = 0; i < 360; ++i)
        {
            data_out_ << "range_" << i << ",";
        }
        for (int i = 0; i < 360; ++i)
        {
            data_out_ << "intensity_" << i << ",";
        }
        data_out_ << "linear_velocity_x,rotational_velocity_z,\n";
    }

private:
    void laser_scan_callback(const sensor_msgs::msg::LaserScan &msg)
    {
        auto start = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(start - last_update_) < std::chrono::duration_cast<std::chrono::milliseconds>(action_update_interval_))
        {
            return;
        }

        data_out_ << msg.header.stamp.nanosec << ","
                  << action_update_interval_.count() << ","
                  << msg.angle_min << ","
                  << msg.angle_max << ","
                  << msg.angle_increment << ","
                  << msg.time_increment << ","
                  << msg.scan_time << ","
                  << msg.range_min << ","
                  << msg.range_max << ",";

        for (const auto &range : msg.ranges)
        {
            data_out_ << range << ",";
        }

        for (const auto &intensity : msg.intensities)
        {
            data_out_ << intensity << ",";
        }

        auto message = geometry_msgs::msg::Twist();

        //! @todo Enumerate action space (go forward / turn)
        const int choice = std::uniform_int_distribution<>(0, 1)(gen_);
        assert(choice == 0 || choice == 1);
        if (choice == 0)
        {
            // message.linear.x = std::uniform_real_distribution<>(min_lin_vel_x_, max_lin_vel_x_)(gen_);
            message.linear.x = TURTLEBOT_3_BURGER_MAX_LINEAR_VELOCITY_X;
        }
        else
        {
            message.angular.z = std::uniform_real_distribution<>(min_ang_vel_z_, max_ang_vel_z_)(gen_);
        }

        publisher_->publish(message);
        data_out_ << message.linear.x << "," << message.angular.z;

        data_out_ << "\n";
        auto dur_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        RCLCPP_INFO(get_logger(), "I took %li ms to process laser scan...", dur_ms);

        last_update_ = std::chrono::steady_clock::now();
    }

    static constexpr double TURTLEBOT_3_BURGER_MAX_LINEAR_VELOCITY_X = 0.22;
    static constexpr double TURTLEBOT_3_BURGER_MIN_ANGULAR_VELOCITY_Z = -2.84;
    static constexpr double TURTLEBOT_3_BURGER_MAX_ANGULAR_VELOCITY_Z = 2.84;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::chrono::time_point<std::chrono::steady_clock> last_update_;
    std::ofstream data_out_;

    //! @todo update these so they are params
    double min_lin_vel_x_ = 0.1;
    double max_lin_vel_x_ = TURTLEBOT_3_BURGER_MAX_LINEAR_VELOCITY_X;
    double min_ang_vel_z_ = TURTLEBOT_3_BURGER_MIN_ANGULAR_VELOCITY_Z;
    double max_ang_vel_z_ = TURTLEBOT_3_BURGER_MAX_ANGULAR_VELOCITY_Z;
    std::chrono::nanoseconds action_update_interval_ = 1s;

    std::mt19937 gen_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomExploration>());
    rclcpp::shutdown();
    return 0;
}
