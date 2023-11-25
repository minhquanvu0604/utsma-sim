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
#include <queue>

#include "d_triang_types.hpp"

// typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
// typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;
// typedef Kernel::Point_2 Point_2;
// typedef DelaunayTriangulation::Edge Edge;
// typedef DelaunayTriangulation::Edge_circulator Edge_circulator;

// typedef DT::Node Node;


/**
 * @brief Base class for path planners
 * 
 * CONTRACT
 *      INPUT : A list of cones to be used for planning at that instance
 *      OUTPUT : A path
 *          ---- Find a good data structure for them ----
 *      BEHAVIOUR : ....
 * 
 * ANALYSIS
 *      Compromise the computer vision section
 *          Missing cones
 *          Incorrect color
 * 
 *      Loss function : based on compromisation of vision 
 *          Color
 *          Cone lining up in a reasonable curve
 *          Path is of reasonable curve
 *          Distance between consecutive cones
 *          Have cones on both side
 * 
 * TO RESEARCH
 *      How to refine the path certainty other time
 *      Metrics for performance evaluation
 *      The path planner takes input from SLAM, but still stores meaningful
 *          data itself regarding past path
 * 
 * NOTE 
 *      The path always stems from the car body
 * 
 */
class DTriangPlanner {

public:
    DTriangPlanner(){}

    void set_cones(std::vector<Point_2> points_local);
    
    /*
     * Main function to perform planning
    */
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

    // /*
    //  * Explore 2 incident edges of the current edge
    // */
    // std::pair<Edge, Edge> expand(const Edge& input_edge);

    void expand(Edge start);
    
    std::vector<Edge> get_next_edges(Edge current_edge, Edge previous_edge);

    DelaunayTriangulation* get_triangulation_ptr();


protected:
    DelaunayTriangulation _dt;
    std::vector<Point_2> _points_local;
    
    // All the possible paths found, each vector of Point_2 represents a path
    std::vector<std::vector<Point_2>> _paths;

private:
    Point_2 get_midpoint_of_edge(const Edge& edge);
    // double angle_difference(const DT::Pose& last_node_pose, const Point_2& new_midpoint);       
    double compute_orientation(const Point_2& p1, const Point_2& p2);
    std::vector<Point_2> backtrack_path(const std::shared_ptr<DT::Node>& leaf_node);

    // Normalize the angle difference to the range (-π, π)
    double normalize(double angle); // static?

    // Testing function 
    void print_face_vertices(DelaunayTriangulation::Face_handle face);

};



#endif