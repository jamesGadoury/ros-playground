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

class NodeVisualizer : public rclcpp::Node {
public:
    NodeVisualizer() : rclcpp::Node("NodeVisualizer") {
        node_subscription = create_subscription<node_interfaces::msg::Node>("node_topic", 10, bind(&NodeVisualizer::node_callback, this, _1));
    
    sf::RenderWindow window(
        sf::VideoMode(1000.f, 1000.f),
        "SFML FML."
    );

    while (window.isOpen()) {
        auto start = chrono::steady_clock::now();
        sf::Event event;

        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::White);

        for (const auto &node : nodes) {
            window.draw(convert(node));
        }

        window.display();

        static constexpr auto MINIMUM_UPDATE_DUR = chrono::milliseconds(200);
        if (
            auto update_dur = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start);
            update_dur < MINIMUM_UPDATE_DUR
        ) {
            this_thread::sleep_for(MINIMUM_UPDATE_DUR - update_dur);
        }
    }

    }

    vector<VisualNode> nodes;
private:
    rclcpp::Subscription<node_interfaces::msg::Node>::SharedPtr node_subscription;

    void node_callback(const node_interfaces::msg::Node &msg) {
        RCLCPP_INFO_STREAM(get_logger(), "Heard with position: " << msg.position.x << "," << msg.position.y);
        nodes.push_back(convert(msg));
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<NodeVisualizer>());
    rclcpp::shutdown();

    return 0;
}
