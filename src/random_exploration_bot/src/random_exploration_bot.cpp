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
        timer_ = create_wall_timer(3s, std::bind(&RandomExploration::timer_callback, this));
        publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        subscription_ = create_subscription<sensor_msgs::msg::LaserScan>("lidar", 10, std::bind(&RandomExploration::laser_scan_callback, this, _1));
        //! @todo configurable name param?
        laser_scan_out_ = std::ofstream("laser_scan.csv");
        gen_ = std::mt19937(42);
    }

private:
    void timer_callback()
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

    void laser_scan_callback(const sensor_msgs::msg::LaserScan &msg)
    {
        auto start = std::chrono::steady_clock::now();
        laser_scan_out_ << msg.header.stamp.nanosec << "," 
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
            laser_scan_out_ << range << ",";
        }

        for (const auto &intensity : msg.intensities)
        {
            laser_scan_out_ << intensity << ",";
        }

        laser_scan_out_ << "\n";
        auto dur_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();

        RCLCPP_INFO(get_logger(), "I took %li ms to process laser scan...", dur_ms);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::ofstream laser_scan_out_;

    std::mt19937 gen_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomExploration>());
    rclcpp::shutdown();
    return 0;
}
