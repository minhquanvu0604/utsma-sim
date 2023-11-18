#include "d_triang_planner.hpp"

// typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;
// typedef DelaunayTriangulation::Edge_circulator Edge_circulator;

void DTriangPlanner::plan(){

    Edge nearest_edge = triangulate();
}


void DTriangPlanner::set_cones(std::vector<Point_2> points_local){
        _points_local = points_local;
    }

Edge DTriangPlanner::triangulate(){

    // Triangulation computed here
    _dt.insert(_points_local.begin(), _points_local.end()); 

    // The two nearest points are ones with smallest x
    DelaunayTriangulation::Vertex_handle smallest, second_smallest;
    double smallest_x = std::numeric_limits<double>::max(), second_smallest_x = std::numeric_limits<double>::max();

    for (auto v = _dt.finite_vertices_begin(); v != _dt.finite_vertices_end(); ++v) {
        double x_value = CGAL::to_double(v->point().x());
        if (x_value < smallest_x) {
            second_smallest = smallest;
            second_smallest_x = smallest_x;
            smallest = v;
            smallest_x = x_value;
        } else if (x_value < second_smallest_x) {
            second_smallest = v;
            second_smallest_x = x_value;
        }
    }
            
    // std::cout << "Two nearest points: " << std::endl;
    // std::cout << smallest->point().x() << " , " << smallest->point().y() << std::endl;
    // std::cout << second_smallest->point().x() << " , " << second_smallest->point().y() << std::endl;

    // From 2 nearest vertices, find the corresponding edge
    Edge nearest_edge;
    bool edge_found = false;

    Edge_circulator c = _dt.incident_edges(smallest), done(c);
    if (c != 0) {
        do {
            if ((c->first->vertex((c->second + 1) % 3) == second_smallest) || (c->first->vertex((c->second + 2) % 3) == second_smallest)) {
                nearest_edge = *c;
                edge_found = true;
                break;
            }
        } while (++c != done);
    }

    if (!edge_found) {
        throw std::runtime_error("Vertices are not connected by an edge.");
    }

    return nearest_edge;
}

void DTriangPlanner::contruct_graph(){
    
}




// ON HOLD
std::pair<Edge, Edge> DTriangPlanner::expand(const Edge& input_edge) {
    Edge_circulator circulator = _dt.incident_edges(input_edge.first->vertex(input_edge.second));
    Edge_circulator done(circulator);

    Edge first_incident_edge, second_incident_edge;

    if (circulator != 0) {
        do {
            if (*circulator != input_edge) {
                if (first_incident_edge == Edge()) {
                    first_incident_edge = *circulator;
                } else {
                    second_incident_edge = *circulator;
                    break;
                }
            }
        } while (++circulator != done);
    }

    // Connecting mid points
    // NOTE - EDGE:
    // ---nearest_edge.first: This is a Face_handle pointing to one of the faces (triangle) adjacent to the edge. 
    // ---nearest_edge.second: This is an integer (0, 1, or 2) that specifies which edge of the triangle 
    // (nearest_edge.first) we are referring to. It helps to identify the exact edge within the triangle.
    
    // Find 2 vertices that are not the vertex opposite to the edge
    Point_2 p1 = input_edge.first->vertex((input_edge.second + 1) % 3)->point();
    Point_2 p2 = input_edge.first->vertex((input_edge.second + 2) % 3)->point();
    Point_2 midpoint = CGAL::midpoint(p1, p2);




    return {first_incident_edge, second_incident_edge};
}