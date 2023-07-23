#include "rclcpp/rclcpp.hpp"
#include <shape_interfaces/msg/renderable_circle.hpp>

#include <SFML/Graphics.hpp>

#include <chrono>
#include <iostream>
#include <random>
#include <functional>
#include <thread>
#include <string>

using namespace std;
using std::placeholders::_1;

namespace sf {

Color convert(const std_msgs::msg::ColorRGBA &color) {
    return sf::Color(color.r, color.g, color.b, color.a);
}

CircleShape convert(const shape_interfaces::msg::RenderableCircle &circle) {
    CircleShape shape(circle.radius);
    shape.setFillColor(convert(circle.color));
    shape.setPosition(circle.position.x, circle.position.y);
    return shape;
}

}

struct Circle {
    string id;
    sf::CircleShape shape;
};

Circle convert(const shape_interfaces::msg::RenderableCircle &circle) {
    return Circle {
        circle.id,
        sf::convert(circle)
    };
}

static const string NODE_NAME = "CircleVisualizer";
class CircleVisualizer : public rclcpp::Node {
public:
    CircleVisualizer() : rclcpp::Node(NODE_NAME), window(sf::VideoMode(1000.f, 1000.f), NODE_NAME) {
        circle_subscription = create_subscription<shape_interfaces::msg::RenderableCircle>("renderable_circles", 10, bind(&CircleVisualizer::circle_callback, this, _1));
        render_timer = create_wall_timer(chrono::milliseconds(200), bind(&CircleVisualizer::render_callback, this));
    } 

private:
    rclcpp::Subscription<shape_interfaces::msg::RenderableCircle>::SharedPtr circle_subscription;
    rclcpp::TimerBase::SharedPtr render_timer;

    sf::RenderWindow window;
    unordered_map<string, Circle> circles;

    void circle_callback(const shape_interfaces::msg::RenderableCircle &msg) {
        RCLCPP_INFO_STREAM(get_logger(), "Heard id: " << msg.id << ", position: (" << msg.position.x << "," << msg.position.y << ").");
        circles[msg.id] = convert(msg);
    }

    void render_callback() {
        if (!window.isOpen()) {
            RCLCPP_INFO_STREAM(get_logger(), "Window closed... skipping render.");
            return;
        }

        const auto start = chrono::steady_clock::now();
        sf::Event event;

        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                RCLCPP_INFO_STREAM(get_logger(), "Closing window.");
                window.close();
                return;
            }
        }

        window.clear(sf::Color::White);

        for (const auto &[id, circle] : circles) {
            RCLCPP_DEBUG(get_logger(), "Processed circle.");
            window.draw(circle.shape);
        }

        window.display();
        RCLCPP_DEBUG_STREAM(
            get_logger(),
            "Render loop took: " <<
            chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start).count() <<
            " ms."
        );
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<CircleVisualizer>());
    rclcpp::shutdown();

    return 0;
}
