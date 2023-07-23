#include "rclcpp/rclcpp.hpp"
#include <node_interfaces/msg/node.hpp>

#include <SFML/Graphics.hpp>

#include <chrono>
#include <iostream>
#include <random>
#include <functional>
#include <thread>

using namespace std;
using std::placeholders::_1;

struct VisualNode {
    sf::Color color = sf::Color::Blue;
    float x = 0.f;
    float y = 0.f;
    float radius = 0.f;
};

sf::CircleShape convert(const VisualNode &node) {
    sf::CircleShape shape(node.radius);
    shape.setFillColor(node.color);
    shape.setPosition(node.x, node.y);
    return shape;
}

VisualNode convert(const node_interfaces::msg::Node &msg) {
    return {
        sf::Color(msg.color.r, msg.color.g, msg.color.b, msg.color.a),
        static_cast<float>(msg.position.x),
        static_cast<float>(msg.position.y),
        static_cast<float>(msg.radius)
    };
}

static const string NODE_NAME = "NodeVisualizer";
class NodeVisualizer : public rclcpp::Node {
public:
    NodeVisualizer() : rclcpp::Node(NODE_NAME), window(sf::VideoMode(1000.f, 1000.f), NODE_NAME) {
        node_subscription = create_subscription<node_interfaces::msg::Node>("node_topic", 10, bind(&NodeVisualizer::node_callback, this, _1));
        render_timer = create_wall_timer(chrono::milliseconds(200), bind(&NodeVisualizer::render_callback, this));
    } 

private:
    rclcpp::Subscription<node_interfaces::msg::Node>::SharedPtr node_subscription;
    rclcpp::TimerBase::SharedPtr render_timer;

    sf::RenderWindow window;
    vector<VisualNode> nodes;

    void node_callback(const node_interfaces::msg::Node &msg) {
        RCLCPP_INFO_STREAM(get_logger(), "Heard with position: " << msg.position.x << "," << msg.position.y);
        nodes.push_back(convert(msg));
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

        for (const auto &node : nodes) {
            RCLCPP_DEBUG(get_logger(), "Processed node.");
            window.draw(convert(node));
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
    rclcpp::spin(make_shared<NodeVisualizer>());
    rclcpp::shutdown();

    return 0;
}
