#include "rclcpp/rclcpp.hpp"

#include <search/example_problems/grid_problem.hpp>
#include <search/informed/a_star_search.hpp>
#include <grid_search_interfaces/msg/grid_node.hpp>
#include <grid_search_interfaces/msg/grid_search_event.hpp>

#include <iostream>
#include <functional>
#include <memory>

using namespace std;
using namespace search;
using namespace search::informed;
using namespace search::example_problems;

static const function<int(const GridProblem &problem, const ProblemNode<GridProblem> &node)> HEURISTIC =
    [](const auto &problem, const auto &node) {
        auto current = to_grid_entry(node.state);
        auto goal = to_grid_entry(problem.goal_state());

        return abs(static_cast<int>(current.row-goal.row)) + abs(static_cast<int>(current.col-goal.col));
    };

grid_search_interfaces::msg::GridNode convert(const ProblemNode<GridProblem> &node) {
    grid_search_interfaces::msg::GridNode node_msg;
    node_msg.action = node.action;
    node_msg.state = node.state;
    node_msg.path_cost = node.path_cost;
    return node_msg;
}

class GridSearchExample : public rclcpp::Node {
public:
    GridSearchExample() : rclcpp::Node("GridSearchExample") {
        search_event_publisher = create_publisher<grid_search_interfaces::msg::GridSearchEvent>("grid_search_event", 10);
        const GridProblem problem({
            20,
            20,
            GridEntry { 0, 0},
            GridEntry { 17, 18 }});

        this_thread::sleep_for(chrono::seconds(3));

        auto dispatcher = std::make_unique<eventpp::EventDispatcher<Event, void (const ProblemNode<GridProblem> &)>>();
        dispatcher->appendListener(Event::POP, [this](const ProblemNode<GridProblem> &node){
            grid_search_interfaces::msg::GridSearchEvent event;
            event.event_name = "POP";
            event.node = convert(node);

            search_event_publisher->publish(event);
        });
        
        dispatcher->appendListener(Event::EXPAND, [this](const ProblemNode<GridProblem> &node){
            grid_search_interfaces::msg::GridSearchEvent event;
            event.event_name = "EXPAND";
            event.node = convert(node);

            search_event_publisher->publish(event);
        });

        dispatcher->appendListener(Event::COMPLETE, [this](const ProblemNode<GridProblem> &node){
            grid_search_interfaces::msg::GridSearchEvent event;
            event.event_name = "COMPLETE";
            event.node = convert(node);

            search_event_publisher->publish(event);
        });

        RCLCPP_INFO(get_logger(), "Starting search.");
        a_star_search(problem, HEURISTIC, dispatcher.get());
        RCLCPP_INFO(get_logger(), "Finished search.");
    }
private:
    rclcpp::Publisher<grid_search_interfaces::msg::GridSearchEvent>::SharedPtr search_event_publisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridSearchExample>());
    rclcpp::shutdown();

    return 0;
}
