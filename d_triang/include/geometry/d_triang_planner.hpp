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

/**
 * @brief Base class for path planners
 * 
 * CONTRACT
 *      INPUT : A list of cones to be used for planning at that instance
 *      OUTPUT : A path
 *          ---- Find a good data structure for them ----
 *      BEHAVIOUR : ....
 * 
 * USAGE
 *      set_cones() -> plan() -> get_best_path()/other getters
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
 * EXPERIMENT RESULT
 *      The best path doesn't always have the most points (esp. in case where of missing cones) 
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


    /*
     * Expand all the possible paths in a bread-first fashion
    */
    void expand(Edge start);

    /*
     * 
    */
    std::vector<Edge> get_next_edges(Edge current_edge, Edge previous_edge);   




    /*
     * Get a list of complete paths
    */    
    std::vector<std::vector<Point_2>> get_paths();

    /*
     * Get the best path from all complete paths, which is meant to be the chosen path 
     * for the car to follow
    */
    std::vector<Point_2> get_best_path();

    /*
    * Get other complete paths besides the best one
    */
    std::vector<std::vector<Point_2>> get_other_paths();

    
    // Testing function 
    DelaunayTriangulation* get_triangulation_ptr();
    Point_2 transform_to_car_frame(const Point_2& global_pt, double car_x, double car_y, double car_yaw);
    void print_face_vertices(DelaunayTriangulation::Face_handle face);
    void print_paths();
    void print_edge_vertices(const Edge& edge);
    bool is_approx_equal(const Point_2& p1, const Point_2& p2, double tolerance); // Also used by test fixture

protected:

    DelaunayTriangulation _dt;
    std::vector<Point_2> _points_local;
    
    // All the possible paths found, each vector of Point_2 represents a path
    std::vector<std::vector<Point_2>> _paths;
    std::vector<Point_2> _best_path;
    std::vector<std::vector<Point_2>> _other_paths;


private:

    void choose_best_path();

    Point_2 get_midpoint_of_edge(const Edge& edge);
    
    /*
    * p1 is current node, p2 is next node
    */
    double compute_orientation(const Point_2& p1, const Point_2& p2);
    std::vector<Point_2> backtrack_path(const std::shared_ptr<DT::Node>& leaf_node);

    // Normalize the angle difference to the range (-π, π)
    double normalize(double angle); // static?


    bool are_edges_equal(const Edge& e1, const Edge& e2);

};



#endif