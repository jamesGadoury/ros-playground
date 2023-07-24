#include "rclcpp/rclcpp.hpp"

#include <SFML/Graphics.hpp>
#include <search/node.hpp>
#include <search/example_problems/grid_problem.hpp>
#include <grid_search_interfaces/msg/grid_node.hpp>
#include <grid_search_interfaces/msg/grid_search_event.hpp>

#include <iostream>
#include <functional>
#include <memory>
#include <queue>

using namespace std;
using namespace search;
using namespace search::example_problems;

using std::placeholders::_1;

ProblemNode<GridProblem> convert(const grid_search_interfaces::msg::GridNode &node) {
    ProblemNode<GridProblem> new_node;
    new_node.action = node.action;
    new_node.state = node.state;
    new_node.path_cost = node.path_cost;
    return new_node;
}

grid_search_interfaces::msg::GridNode convert(const ProblemNode<GridProblem> &node) {
    grid_search_interfaces::msg::GridNode node_msg;
    node_msg.action = node.action;
    node_msg.state = node.state;
    node_msg.path_cost = node.path_cost;
    return node_msg;
}

class GridSearchVisualizer : public rclcpp::Node {
public:
    GridSearchVisualizer() : rclcpp::Node("GridSearchVisualizer"), problem({20, 20, GridEntry {0, 0}, GridEntry {17, 18}}) {
        search_event_subscription = create_subscription<grid_search_interfaces::msg::GridSearchEvent>("grid_search_event", 10, bind(&GridSearchVisualizer::search_event_callback, this, _1));
        window = std::make_unique<sf::RenderWindow>(
            sf::VideoMode(
                2000,
                2000
            ),
            "GridSearchVisualizer"
        );
        render_timer = create_wall_timer(std::chrono::milliseconds(100), bind(&GridSearchVisualizer::render_callback, this));

        initial_render();
    }
private:
    rclcpp::Subscription<grid_search_interfaces::msg::GridSearchEvent>::SharedPtr search_event_subscription;
    GridProblem problem;
    rclcpp::TimerBase::SharedPtr render_timer;
    std::unique_ptr<sf::RenderWindow> window;
    queue<grid_search_interfaces::msg::GridSearchEvent> event_queue;

    void search_event_callback(const grid_search_interfaces::msg::GridSearchEvent &event) {
        event_queue.push(event);
    }

    void initial_render() {
        window->clear(sf::Color::White);

        //! @todo use the problem to set these bounds...
        const int space_x_per_node = window->getSize().x / 20;
        const int space_y_per_node = window->getSize().y / 20;
        const int radius = space_x_per_node / 2;

        for (int i = 0; i < 20; ++i) {
            for (int j = 0; j < 20; ++j) {
                sf::CircleShape shape;
                shape.setRadius(radius);
                shape.setOutlineColor(sf::Color::Blue);
                shape.setOutlineThickness(1.f);
                shape.setPosition(i * space_x_per_node, j * space_y_per_node);
                window->draw(shape);
            }
        }

        window->display();
    }

    void render_callback() {
        if (!window->isOpen()) {
            RCLCPP_INFO_STREAM(get_logger(), "Window closed... skipping render.");
            return;
        }

        const auto start = std::chrono::steady_clock::now();
        sf::Event event;

        while (window->pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                RCLCPP_INFO_STREAM(get_logger(), "Closing window.");
                window->close();
                return;
            }
        }

        if (event_queue.empty()) {
            RCLCPP_INFO_STREAM(get_logger(), "Event queue is empty... skipping render.");
            return;
        }

        const auto search_event = event_queue.front();
        event_queue.pop();

        //! @todo use the problem to set these bounds...
        const int space_x_per_node = window->getSize().x / 20;
        const int space_y_per_node = window->getSize().y / 20;
        const int radius = space_x_per_node / 2;

        const GridEntry node_position = to_grid_entry(search_event.node.state);
        sf::CircleShape circle;
        circle.setPosition(node_position.col * space_x_per_node, node_position.row * space_y_per_node);
        circle.setRadius(radius);

        if (search_event.event_name == "POP") {
            circle.setFillColor(sf::Color::Blue);
        } else if (search_event.event_name == "EXPAND") {
            circle.setFillColor(sf::Color::Cyan);
        } else if (search_event.event_name == "COMPLETE") {
            circle.setFillColor(sf::Color::Green);
        } else {
            RCLCPP_ERROR_STREAM(get_logger(), "Unexpected event name: " << search_event.event_name);
            return;
        }

        window->draw(circle);
        window->display();
        RCLCPP_DEBUG_STREAM(
            get_logger(),
            "Render loop took: " <<
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() <<
            " ms."
        );
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridSearchVisualizer>());
    rclcpp::shutdown();

    return 0;
}
