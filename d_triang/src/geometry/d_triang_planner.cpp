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
std::vector<std::vector<Point_2>> DTriangPlanner::expand(Edge start) {
    
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

    return paths;
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
//     // NOTE - EDGE:
//     // ---nearest_edge.first: This is a Face_handle pointing to one of the faces (triangle) adjacent to the edge. 
//     // ---nearest_edge.second: This is an integer (0, 1, or 2) that specifies which edge of the triangle 
//     // (nearest_edge.first) we are referring to. It helps to identify the exact edge within the triangle.
    
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
*/
std::vector<Edge> DTriangPlanner::get_next_edges(Edge current_edge, Edge previous_edge) {
    
    std::vector<Edge> next_edges;

    // Find the opposite face of the current edge
    DelaunayTriangulation::Face_handle new_face = current_edge.first;

    // Logic check - Recognise first edge
    if (previous_edge.first == DelaunayTriangulation::Face_handle()) {
        std::cout << "First edge" << std::endl;
    }

    // The new face must not contain the previous edge
    // The function is assumed to be called with finite edges, so new_face is hence finite face
    // if (new_face == previous_edge.first || new_face == previous_edge.first->neighbor(previous_edge.second) || _dt.is_infinite(new_face))
    if (new_face == previous_edge.first || new_face == previous_edge.first->neighbor(previous_edge.second) || _dt.is_infinite(new_face))
        new_face = current_edge.first->neighbor(current_edge.second);

    std::cout << "1" << std::endl;

    // Logic check - The new face should be finite 
    if (_dt.is_infinite(new_face)) 
        throw std::runtime_error("The new face is infinite - should not happen");
    
    std::cout << "2" << std::endl;

    // Debuggin
    print_face_vertices(new_face);
    // std::cout << "current_edge.first: " << current_edge.first << std::endl;


    // The two other edges of the new face
    int new_face_index = new_face->index(current_edge.first);
    std::cout << "3" << std::endl;

    Edge edge1(new_face, (new_face_index + 1) % 3);
    Edge edge2(new_face, (new_face_index + 2) % 3);
    std::cout << "4" << std::endl;

    next_edges.push_back(edge1);
    next_edges.push_back(edge2);

    std::cout << "" << std::endl;

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