#include <search/example_problems/grid_problem.hpp>
#include <search/informed/a_star_search.hpp>

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

int main(int, char *[]) {
    const GridProblem problem({
        .rows = 1000,
        .cols = 1000,
        .initial = GridEntry { .row=0, .col=0},
        .goal = GridEntry { .row=839, .col=943 }});

    cout << "Starting state: " << problem.initial_state() << endl;
    cout << "Goal state: " << problem.goal_state() << endl;

    auto dispatcher = make_unique<eventpp::EventDispatcher<Event, void (const ProblemNode<GridProblem> &)>>();
    dispatcher->appendListener(Event::POP, [](const ProblemNode<GridProblem> &node){
        cout << "POPPED: " << endl << node << endl;
    });
    
    dispatcher->appendListener(Event::EXPAND, [](const ProblemNode<GridProblem> &node){
        cout << "EXPANDED: " << endl << node << endl;
    });

    dispatcher->appendListener(Event::COMPLETE, [](const ProblemNode<GridProblem> &node){
        cout << "COMPLETED: " << endl << node << endl;
    });

    cout << "---------------------------------"  << endl;
    cout << "Executing a_star_search..." << endl;
    cout << a_star_search(problem, HEURISTIC, dispatcher.get()) << endl;
    cout << endl;
    cout << endl;
}