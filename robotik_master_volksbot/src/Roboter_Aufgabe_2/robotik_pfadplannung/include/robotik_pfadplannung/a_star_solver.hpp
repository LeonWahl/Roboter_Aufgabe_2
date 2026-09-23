#ifndef A_STAR_SOLVER_HPP
#define A_STAR_SOLVER_HPP

#include "graph_structure.hpp"
#include <vector>

class AStarSolver {
public:
    static std::vector<int> findPath(const TopologicalGraph& graph, int start_idx, int target_idx);
};

#endif // A_STAR_SOLVER_HPP