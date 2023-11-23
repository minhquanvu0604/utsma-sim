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

    // Logic error checking
    if (!edge_found) 
        throw std::runtime_error("Vertices are not connected by an edge.");

    return nearest_edge;
}


// /*
//  * Depth first manner
// */
// void DTriangPlanner::contruct_graph(){
//     std::vector<Point_2> traversing_progress;

// }


/*
 * Expand all the possible paths in a bread-first fashion
*/
void DTriangPlanner::expand(Edge start) {
    
    // All the possible paths found  
    std::vector<std::vector<Point_2>> paths;

    // A path is represented as a vector of Edge
    // A queue to store the instantaneous traversing progress 
    std::queue<std::vector<Edge>> queue;
    queue.push({start});

    while (!queue.empty()) {

        // The current path in the traversing progress
        std::vector<Edge> current_path = queue.front();
        queue.pop();

        // The current leaf of that path is the last edge
        Edge last_edge = current_path.back();
        // If the current path has only the starting edge, use default constructor for second to last edge
        Edge second_last_edge = current_path.size() > 1 ? current_path[current_path.size() - 2] : Edge(); 
        
        // Expand on the last edge
        std::vector<Edge> next_edges = get_next_edges(last_edge, second_last_edge);

        if (next_edges.empty()) {
            // Convert the dead-end path to points and store it
            std::vector<Point_2> path_pts = edge_to_point(current_path);
            paths.push_back(path_pts);    
        } 
        else {
            // Extend the current path with each of the next edges
            for (const Edge& next_edge : next_edges) {
                // if (std::find(current_path.begin(), current_path.end(), next_edge) == current_path.end()) {
                std::vector<Edge> new_path = current_path;
                new_path.push_back(next_edge);
                queue.push(new_path); // Push the extended path for further exploration
                // }
            }
        }
    }

    // return paths;
}




// std::pair<Edge, Edge> DTriangPlanner::expand(const Edge& input_edge) {
//     Edge_circulator circulator = _dt.incident_edges(input_edge.first->vertex(input_edge.second));
//     Edge_circulator done(circulator);

//     Edge first_incident_edge, second_incident_edge;

//     if (circulator != 0) {
//         do {
//             if (*circulator != input_edge) {
//                 if (first_incident_edge == Edge()) {
//                     first_incident_edge = *circulator;
//                 } else {
//                     second_incident_edge = *circulator;
//                     break;
//                 }
//             }
//         } while (++circulator != done);
//     }

//     // Connecting mid points

    
//     // Find 2 vertices that are not the vertex opposite to the edge
//     Point_2 p1 = input_edge.first->vertex((input_edge.second + 1) % 3)->point();
//     Point_2 p2 = input_edge.first->vertex((input_edge.second + 2) % 3)->point();
//     Point_2 midpoint = CGAL::midpoint(p1, p2);




//     return {first_incident_edge, second_incident_edge};
// }


// UTILITY FUNCTIONS //

/*
 * Get next 2 edges from the current edge, moving to the next face
 * Returns empty vector means the path meets the dead end
 * 
 * If the edge is a starting position of the path, previous_edge will be created by the defautl contructor
 * 
 * Both edge must refer to the same face (current_edge.first == previous_edge.first)
 * 
 * Returns 3 edges, the first one is the current_edge with respect to new _face
 * The two other edges are others of the new_face
*/
std::vector<Edge> DTriangPlanner::get_next_edges(Edge current_edge, Edge previous_edge) {
    
    std::vector<Edge> next_edges;


    // Recognise first edge
    if (previous_edge.first == DelaunayTriangulation::Face_handle()) {
        std::cout << "First edge" << std::endl;

        DelaunayTriangulation::Face_handle new_face = current_edge.first;
        if (_dt.is_infinite(new_face)){
            // new_face = current_edge.first->neighbor(current_edge.second);
            throw std::runtime_error("Infinite face from first edge");
        }
        Edge edge0(new_face, current_edge.second);
        Edge edge1(new_face, (current_edge.second + 1) % 3);
        Edge edge2(new_face, (current_edge.second + 2) % 3);

        next_edges.push_back(edge0);
        next_edges.push_back(edge1);
        next_edges.push_back(edge2);

        return next_edges;
    }

    // The previous face is the one that contains both current_edge and previous_edge
    DelaunayTriangulation::Face_handle previous_face = current_edge.first; // (current_edge.first == previous_edge.first)
    DelaunayTriangulation::Face_handle new_face = previous_face->neighbor(current_edge.second);
    // // Debugging
    // std::cout << "Print new_face vertices" << std::endl;
    // print_face_vertices(new_face);

    int new_face_edge_index = new_face->index(previous_face);

    Edge edge0(new_face, new_face_edge_index);
    Edge edge1(new_face, (new_face_edge_index + 1) % 3);
    Edge edge2(new_face, (new_face_edge_index + 2) % 3);

    next_edges.push_back(edge0);
    next_edges.push_back(edge1);
    next_edges.push_back(edge2);

    return next_edges;
}


std::vector<Point_2> DTriangPlanner::edge_to_point(const std::vector<Edge>& path) {
    std::vector<Point_2> points;
    for (const Edge& edge : path) {
        Point_2 p1 = edge.first->vertex((edge.second + 1) % 3)->point();
        Point_2 p2 = edge.first->vertex((edge.second + 2) % 3)->point();
        points.push_back(CGAL::midpoint(p1, p2));
    }
    return points;
}


// TESTING FUNCTION 
void DTriangPlanner::print_face_vertices(DelaunayTriangulation::Face_handle face) {
    if (!face->is_valid()) {
        std::cerr << "Invalid face handle." << std::endl;
        return;
    }

    for (int i = 0; i < 3; ++i) {
        Point_2 vertex = face->vertex(i)->point();
        std::cout << "Vertex " << i << ": (" << vertex.x() << ", " << vertex.y() << ")" << std::endl;
    }
}

bool DTriangPlanner::are_edges_equivalent(const Edge& edge1, const Edge& edge2) {
    Point_2 p1_edge1 = _dt.segment(edge1).source();
    Point_2 p2_edge1 = _dt.segment(edge1).target();

    Point_2 p1_edge2 = _dt.segment(edge2).source();
    Point_2 p2_edge2 = _dt.segment(edge2).target();

    // Ensure consistent ordering of points
    if (p1_edge1 > p2_edge1) std::swap(p1_edge1, p2_edge1);
    if (p1_edge2 > p2_edge2) std::swap(p1_edge2, p2_edge2);

    return (p1_edge1 == p1_edge2) && (p2_edge1 == p2_edge2);
}