#ifndef D_TRIANG_PLANNER_H
#define D_TRIANG_PLANNER_H

#include <iostream>
#include <vector>
#include <memory>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/point_generators_2.h>
#include <yaml-cpp/yaml.h>
#include <limits>
#include <cmath>

#include "d_triang_types.hpp"

// typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
// typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;
// typedef Kernel::Point_2 Point_2;
// typedef DelaunayTriangulation::Edge Edge;
// typedef DelaunayTriangulation::Edge_circulator Edge_circulator;

typedef DT::Node Node;
typedef DT::Pose Pose;

class DTriangPlanner {

public:
    DTriangPlanner(){}

    void set_cones(std::vector<Point_2> points_local);
    
    void plan();

    /*
     * Perform triangulation and return the nearest edge to start with
     * Return a copied edge to navigate inside the triangulation. 
     * 
     * In CGAL, even though nearest_edge is a copy, it still represents a valid edge in the triangulation. 
     * The copied edge retains the necessary information (face handle and index) to identify the specific edge 
     * in the triangulation, allowing you to use it for further operations or navigation within the triangulation structure.
    */
    Edge triangulate();

    void contruct_graph();

    /*
     * Explore 2 incident edges of the current edge
    */
    std::pair<Edge, Edge> expand(const Edge& input_edge);



    // void expand(std::shared_ptr<Node> current_node);


protected:
    DelaunayTriangulation _dt;
    std::vector<Point_2> _points_local;

    std::vector<Node> _node_list;

private:
    
    


};



#endif