#include <memory>
#include <random>
#include <fstream>

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
        //! @todo configurable name param?
        data_out_ = std::ofstream("bot_data_out.csv");
        gen_ = std::mt19937(42);
        last_update_ = std::chrono::steady_clock::now();
    }

private:
    void laser_scan_callback(const sensor_msgs::msg::LaserScan &msg)
    {
        auto start = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(start - last_update_) < 1s)
        {
            return;
        }

        data_out_ << msg.header.stamp.nanosec << "," 
                        << msg.header.frame_id << ","
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
        message.linear.x = std::uniform_real_distribution<>(0.0, 0.22)(gen_);
        message.angular.z = std::uniform_real_distribution<>(-2.84, 2.84)(gen_);
        publisher_->publish(message);
        data_out_ << message.linear.x << "," << message.angular.z << ",";

        data_out_ << "\n";
        auto dur_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();
        RCLCPP_INFO(get_logger(), "I took %li ms to process laser scan...", dur_ms);

        last_update_ = std::chrono::steady_clock::now();
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::chrono::time_point<std::chrono::steady_clock> last_update_;
    std::ofstream data_out_;

    std::mt19937 gen_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomExploration>());
    rclcpp::shutdown();
    return 0;
}
