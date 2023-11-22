#ifndef D_TRIANG_TYPES_H
#define D_TRIANG_TYPES_H

#include <vector>
#include <memory>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;
typedef DelaunayTriangulation::Edge Edge;
typedef DelaunayTriangulation::Edge_circulator Edge_circulator;
// DelaunayTriangulation::Face_handle Face_handle; // somehow doesn't work

namespace DT{

    struct Pose{
        Point_2 position;
        double yaw;

        Pose(double x, double y, double yaw) : position{Point_2(x, y)}, yaw{yaw} {}
    };

    struct Node {
        Pose pose;

        // double depth = 0; // DO I NEED THIS?
        // double absTurning = 0; // DO I NEED THIS?
        // double reward = 0; // DO I NEED THIS? REWARD IS FOR FULL PATH
        // double accumulate_angle_diff = -1.0;

        Edge edge; // Each Node is a mid point associated with an edge

        // std::vector<std::shared_ptr<Node>> children;
        // std::shared_ptr<Node> parent;

        Node(const Pose& p) : pose{p} {}
        Node(double x, double y, double yaw) : pose{Pose(x, y , yaw)} {}
            
        // double angle_difference() const {
        //     if (parent) {
        //         double angle_diff = std::atan2(std::sin(pose.yaw - parent->pose.yaw), std::cos(pose.yaw - parent->pose.yaw));
        //         return std::abs(angle_diff);
        //     }
        //     return -100.0; 
        // }
    };


    /*
     * What parameters to represent a path?
     * - Associate edge for each point?
     * - A map to represent which nodes the path goes between (good path should unlock more cones)?
     * - Per-path basis or per-node basis: angle difference, confident point (depends on path eval metrics)?
     * - Depth 
     * - etc
    */
    // struct Path {
    //     std::vector<std::shared_ptr<Point_2>> order;
    //     int depth = -1;
    // };

}
#endif