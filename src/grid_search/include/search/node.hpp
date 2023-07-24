#pragma once

#include "search/problem.hpp"

#include <memory>

namespace search {

template <typename State, typename Action, typename ActionCost>
struct NodeTemplate {
    State state;
    std::shared_ptr<NodeTemplate> parent;
    Action action;
    ActionCost path_cost;
};

template <typename ProblemInterface>
using ProblemNode = NodeTemplate<typename ProblemInterface::State, typename ProblemInterface::Action, typename ProblemInterface::ActionCost>;

}
