#include <iostream>
#include <vector>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/point_generators_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;

int main() {
    // Generate a set of random points
    std::vector<Point_2> points;
    CGAL::Random_points_in_square_2<Point_2> gen(1.0);
    for (int i = 0; i < 10; ++i) {
        points.push_back(*gen++);
    }

    // Compute Delaunay Triangulation
    DelaunayTriangulation dt;
    dt.insert(points.begin(), points.end());

    // Output the vertices of the Delaunay Triangulation
    std::cout << "Delaunay Triangulation vertices:" << std::endl;
    for (auto vertex = dt.finite_vertices_begin(); vertex != dt.finite_vertices_end(); ++vertex) {
        std::cout << vertex->point() << std::endl;
    }

    // Compute midpoints of internal edges and store in a vector
    std::vector<Point_2> midpoints;
    for (auto edge = dt.finite_edges_begin(); edge != dt.finite_edges_end(); ++edge) {
        // Check if the face containing the edge is infinite
        if (!dt.is_infinite(edge->first)) {
            // Compute the midpoint of the edge and store it in the vector
            Point_2 midpoint = CGAL::midpoint(edge->first->vertex((edge->second + 1) % 3)->point(),
                                  edge->first->vertex((edge->second + 2) % 3)->point());
            midpoints.push_back(midpoint);
        }
    }

    // Output the midpoints
    std::cout << "Midpoints of internal edges:" << std::endl;
    for (const auto& midpoint : midpoints) {
        std::cout << midpoint << std::endl;
    }

    return 0;
}
