#ifndef D_TRIANG_PLANNER_H
#define D_TRIANG_PLANNER_H

#include <iostream>
#include <vector>
#include <memory>

#include "d_triang_types.hpp"

typedef DT::Node Node;
typedef DT::Pose Pose;

class DTriangPlanner {

public:

    void expand(std::shared_ptr<Node> current_node);


private:



};












#endif