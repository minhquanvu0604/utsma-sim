#include <iostream>
#include <vector>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/point_generators_2.h>

#include "d_triang_visualise.hpp"





Point_2 transform_to_car_frame(const Point_2& global_pt) {
    const double car_x = 20.990251;
    const double car_y = 13.411934;
    const double car_yaw = 0.013218;

    // Translate the point
    double translated_x = CGAL::to_double(global_pt.x()) - car_x;
    double translated_y = CGAL::to_double(global_pt.y()) - car_y;

    // Rotate the point
    double rotated_x = translated_x * cos(car_yaw) + translated_y * sin(car_yaw);
    double rotated_y = -translated_x * sin(car_yaw) + translated_y * cos(car_yaw);

    return Point_2(rotated_x, rotated_y);
}


int main(int argc, char *argv[]) {
    // Generate a set of random points
    std::vector<Point_2> points_global;
    std::vector<Point_2> points_local;

    points_global.push_back(Point_2(26.872997, 9.791506)); 
    points_global.push_back(Point_2(38.371532, 9.348429));
    points_global.push_back(Point_2(49.792950, 8.017611));
    points_global.push_back(Point_2(26.821278, 17.784601));
    points_global.push_back(Point_2(38.313610, 17.302580));
    points_global.push_back(Point_2(49.748211, 16.086134));
    points_global.push_back(Point_2(60.851669, 13.144850));

    // Create map for global and corresponding local point
    std::map<Point_2,Point_2> local_global_map;
    // Convert to car's frame
    for (auto point_global : points_global){
        auto p_local = transform_to_car_frame(point_global);
        local_global_map[p_local] = point_global;
        points_local.push_back(p_local);
    }

    // Compute Delaunay Triangulation
    DelaunayTriangulation dt;
    dt.insert(points_local.begin(), points_local.end());

    // Get the correct order of global points
    points_global.clear(); 
    for (const auto& point_local : points_local) {
        Point_2 point_global = local_global_map[point_local];
        points_global.push_back(point_global);
    }

    // Get the edges
    std::vector<std::pair<Point_2, Point_2>> edges;
    for (auto it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
        auto segment = dt.segment(it);
        edges.emplace_back(segment.start(), segment.end());
    }

    // Qt
    QApplication app(argc, argv);
    PlotWidget widget(points_global, points_local, edges);
    widget.resize(800, 900); 
    widget.show();

    return app.exec();


    // // Output the vertices of the Delaunay Triangulation
    // std::cout << "Delaunay Triangulation vertices:" << std::endl;
    // for (auto vertex = dt.finite_vertices_begin(); vertex != dt.finite_vertices_end(); ++vertex) {
    //     std::cout << vertex->point() << std::endl;
    // }

    // // Plotting all edges including boundary edges
    // // You need to modify your approach to handle the infinite edges as well (@TODO)
    // for (auto edge = dt.all__edgesbegin(); edge != dt.all__edgesend(); ++edge) {
    //     auto face = edge->first;
    //     int i = edge->second;

    //     if (!dt.is_infinite(face)) {
    //         auto vertex1 = face->vertex((i + 1) % 3)->point();
    //         auto vertex2 = face->vertex((i + 2) % 3)->point();

    //         // Convert for ploting
    //         std::vector<double> edge_x = {CGAL::to_double(vertex1.x()), CGAL::to_double(vertex2.x())};
    //         std::vector<double> edge_y = {CGAL::to_double(vertex1.y()), CGAL::to_double(vertex2.y())};
    //     }
    // }

    // // Compute midpoints of internal edges and store in a vector
    // std::vector<Point_2> midpoints;
    // for (auto edge = dt.finite__edgesbegin(); edge != dt.finite__edgesend(); ++edge) { 
    //     // Finite edges are those edges that are not part of the unbounded face of the triangulation.
        
    //     // Check if the face containing the edge is infinite
    //     // In CGAL, triangulations can have infinite faces, especially when dealing with Delaunay triangulations 
    //     // This check ensures that only internal edges (edges of finite faces) are considered.
    //     if (!dt.is_infinite(edge->first)) {
    //         // Compute the midpoint of the edge and store it in the vector
    //         // The edge->second gives the index of the vertex opposite the edge
    //         // (edge->second + 1) % 3 and (edge->second + 2) % 3 give the indices of the vertices forming the edge.
    //         Point_2 midpoint = CGAL::midpoint(edge->first->vertex((edge->second + 1) % 3)->point(),
    //                               edge->first->vertex((edge->second + 2) % 3)->point());
    //         midpoints.push_back(midpoint);
    //     }
    // }

    // // Output the midpoints
    // std::cout << "Midpoints of internal edges:" << std::endl;
    // for (const auto& midpoint : midpoints) {
    //     std::cout << midpoint << std::endl;
    // }

    return 0;
}
