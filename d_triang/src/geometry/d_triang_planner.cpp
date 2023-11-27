#include "d_triang_planner.hpp"
#include <cmath>


void DTriangPlanner::plan(){
    Edge first_edge = triangulate();
    expand(first_edge);
    choose_best_path();
}


void DTriangPlanner::set_cones(std::vector<Point_2> points_local){
        _points_local.clear();
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
        throw std::runtime_error("Vertices are not connected by an edge");

    if (_dt.is_infinite(nearest_edge))
        throw std::runtime_error("The nearest edge is infinite");

    // std::cout << "The edge:";
    // print_edge_vertices(nearest_edge);

    return nearest_edge;
}

/*
 * 
 * NOTE: need to confirm that start_edge is not infinite 
 * 
 * POTENTIAL FEATURES
 *      Save rejected paths -> prevent the case where the good path is rejected by the last segment
*/
void DTriangPlanner::expand(Edge start_edge) {

    // A queue to store the instantaneous traversing progress 
    // Each has 2 edges represents a state of traverse progress
    // They represent the previous and current edge respectively
    std::deque<DT::TraverseState> traverse_progress;

    std::shared_ptr<DT::Node> car_node_ptr = std::make_shared<DT::Node>(DT::Pose(0,0,0));

    Point_2 first_node_midpoint = get_midpoint_of_edge(start_edge);
    double first_node_angle_diff = compute_orientation(car_node_ptr->pose.position, first_node_midpoint);
    first_node_angle_diff = abs(normalize(first_node_angle_diff));
    
    std::shared_ptr<DT::Node> first_node_ptr = std::make_shared<DT::Node>(DT::Pose(first_node_midpoint, first_node_angle_diff));

    // DEBUGGING
    std::cout << "FIRST NODE: " << std::endl;
    print_edge_vertices(start_edge);
    std::cout << "Midpoint: " << first_node_midpoint << std::endl;
    std::cout << "Previous point: " << car_node_ptr->pose.position << std::endl;
    std::cout << "Angle diff: " << first_node_angle_diff << std::endl; 
    std::cout << "=====================================" << std::endl; 

    int passed = 0; // number of next nodes that pass the validating condtion

    
    DT::TraverseState initial_state;
    initial_state.node_ptr = first_node_ptr;
    initial_state.current_edge = start_edge;
    initial_state.previous_edge = Edge();

    traverse_progress.push_back(initial_state);

    while (!traverse_progress.empty()) {

        DT::TraverseState current_state = traverse_progress.front();
        traverse_progress.pop_front();
        
        // Expand on the last state
        std::vector<Edge> next_edges = get_next_edges(current_state.current_edge, current_state.previous_edge);
        std::shared_ptr<DT::Node> last_node_ptr = current_state.node_ptr;
        Edge& current_new_edge = next_edges.at(0);
        
        std::cout << "Current point: " << last_node_ptr->pose.position << std::endl;
        std::cout << "Current edge: ";
        print_edge_vertices(current_new_edge);
        
        std::cout << "-----------------------------------" << std::endl;

        // Examine 2 edges
        for (int i = 1; i < 3; i++){

            Edge& next_new_edge = next_edges.at(i);

            // If boundary edge, finish the tree 
            if (_dt.is_infinite(next_new_edge)){                       
                std::cout << "END PATH" << std::endl;
                std::vector<Point_2> path = backtrack_path(last_node_ptr);
                _paths.push_back(path);
                
                break; // Break otherwise the same path will be pushed back twice
            }

            Point_2 midpoint = get_midpoint_of_edge(next_new_edge);     
            double midpoint_angle = compute_orientation(last_node_ptr->pose.position, midpoint);
            double angle_diff = abs(normalize(last_node_ptr->pose.yaw - midpoint_angle));


            print_edge_vertices(next_new_edge);
            std::cout << "Midpoint: " << midpoint << std::endl;
            std::cout << "Previous point: " << last_node_ptr->pose.position << std::endl;
            std::cout << "Angle diff: " << angle_diff << std::endl; 


            // Validating condition to accept the next node 
            if (abs(angle_diff) > M_PI/4){
                std::cout << "REJECTED" << std::endl;
                std::cout << "=====================================" << std::endl;
                continue;
            }

            std::cout << "PASSED" << std::endl;
            passed++;

            // Create new node, the node is created here and only here
            std::shared_ptr<DT::Node> new_node_ptr = std::make_shared<DT::Node>(DT::Pose(midpoint, midpoint_angle));
            // Connect this node to last node
            new_node_ptr->parent_ptr = last_node_ptr;
            last_node_ptr->children_ptrs.push_back(new_node_ptr);


            // Otherwise add to new state
            DT::TraverseState new_state(new_node_ptr, next_new_edge, current_new_edge);
            traverse_progress.push_back(new_state);                

            std::cout << "=====================================" << std::endl; 
        }    
    }
    std::cout << "FINISHED GENERATING PATHS" << std::endl;
    std::cout << "Passed: " << passed << std::endl;
    print_paths();
}


std::vector<std::vector<Point_2>> DTriangPlanner::get_paths(){
    return _paths;
}

std::vector<Point_2> DTriangPlanner::get_best_path(){
    return _best_path;
}

std::vector<std::vector<Point_2>> DTriangPlanner::get_other_paths(){
    return _other_paths;
}


DelaunayTriangulation* DTriangPlanner::get_triangulation_ptr() {
    return &_dt;
}



/////////////////// PRIVATE FUNCTIONS ///////////////////////////////////////

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
    Edge edge0;
    Edge edge1;
    Edge edge2;

    // Recognise first edge
    if (previous_edge.first == DelaunayTriangulation::Face_handle()) {
        std::cout << "FIRST EDGE" << std::endl;

        DelaunayTriangulation::Face_handle new_face = current_edge.first;


        std::cout << "NEXT" << std::endl;
        // There may be circumstances where the current_edge is not inifinite but 
        // current_edge.first is inifinite 
        if (_dt.is_infinite(new_face)){
            // new_face = current_edge.first->neighbor(current_edge.second);
            new_face = current_edge.first->neighbor(current_edge.second);
            std::cout << "new_face:" << std::endl;
            print_face_vertices(new_face);

            int new_index = -1;
            for (int i = 0; i < 3; ++i) {
                Edge possible_edge(new_face, i);
                if (are_edges_equal(possible_edge, current_edge)) {
                    new_index = i;
                    break;
                }
            }
            if (new_index == -1) {
                throw std::runtime_error("Could not find the edge in the new face.");
            }

            edge0 = Edge(new_face, new_index);
            edge1 = Edge(new_face, (new_index + 1) % 3);
            edge2 = Edge(new_face, (new_index + 2) % 3);

        }
        else{
            edge0 = Edge(new_face, current_edge.second);
            edge1 = Edge(new_face, (current_edge.second + 1) % 3);
            edge2 = Edge(new_face, (current_edge.second + 2) % 3);
        }

        next_edges.push_back(edge0);
        next_edges.push_back(edge1);
        next_edges.push_back(edge2);

        std::cout << "Print edge: " << std::endl;
        for (auto& edge : next_edges){
            print_edge_vertices(edge);
        }

        return next_edges;
    }

    // The previous face is the one that contains both current_edge and previous_edge
    DelaunayTriangulation::Face_handle previous_face = current_edge.first; // (current_edge.first == previous_edge.first)
    DelaunayTriangulation::Face_handle new_face = previous_face->neighbor(current_edge.second);
    // // Debugging
    // std::cout << "Print new_face vertices" << std::endl;
    // print_face_vertices(new_face);

    int new_face_edge_index = new_face->index(previous_face);

    edge0 = Edge(new_face, new_face_edge_index);
    edge1 = Edge(new_face, (new_face_edge_index + 1) % 3);
    edge2 = Edge(new_face, (new_face_edge_index + 2) % 3);

    next_edges.push_back(edge0);
    next_edges.push_back(edge1);
    next_edges.push_back(edge2);

    return next_edges;
}


void DTriangPlanner::choose_best_path(){
    int max_size = 0;
    int index_of_longest = -1;

    for (int i = 0; i < _paths.size(); ++i) {
        if (_paths[i].size() > max_size) {
            max_size = _paths[i].size();
            index_of_longest = i;
        }
    }

    if (index_of_longest != -1) {
        _best_path = _paths[index_of_longest];
        for (int i = 0; i < _paths.size(); ++i) {
            if (i != index_of_longest) {
                _other_paths.push_back(_paths[i]);
            }
        }
    }
}


Point_2 DTriangPlanner::get_midpoint_of_edge(const Edge& edge) {
    Point_2 p1 = edge.first->vertex((edge.second + 1) % 3)->point();
    Point_2 p2 = edge.first->vertex((edge.second + 2) % 3)->point();
    return CGAL::midpoint(p1, p2);
}


double DTriangPlanner::compute_orientation(const Point_2& p1, const Point_2& p2) {
    double deltaX = CGAL::to_double(p2.x()) - CGAL::to_double(p1.x());
    double deltaY = CGAL::to_double(p2.y()) - CGAL::to_double(p1.y());

    // std::cout << "deltaX: " << deltaX << std::endl;
    // std::cout << "deltaY: " << deltaY << std::endl;
    // std::cout << "std::atan2(deltaY, deltaX): " << std::atan2(deltaY, deltaX) << std::endl;

    return std::atan2(deltaY, deltaX); // Returns angle in radians
}


double DTriangPlanner::normalize(double angle){

    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }    
    return angle;
}


std::vector<Point_2> DTriangPlanner::backtrack_path(const std::shared_ptr<DT::Node>& leaf_node) {
    std::vector<Point_2> path;
    std::shared_ptr<DT::Node> current_node = leaf_node;

    while (current_node) {
        path.push_back(current_node->pose.position);
        current_node = current_node->parent_ptr;
    }

    // Reverse the path to get it from root to leaf
    std::reverse(path.begin(), path.end());

    return path;
}

bool DTriangPlanner::are_edges_equal(const Edge& e1, const Edge& e2) {
    double tolerance = 1e-3;
    auto seg1 = _dt.segment(e1);
    auto seg2 = _dt.segment(e2);

    return (is_approx_equal(seg1.source(), seg2.source(), tolerance) && is_approx_equal(seg1.target(), seg2.target(), tolerance)) ||
           (is_approx_equal(seg1.source(), seg2.target(), tolerance) && is_approx_equal(seg1.target(), seg2.source(), tolerance));
}

bool DTriangPlanner::is_approx_equal(const Point_2& p1, const Point_2& p2, double tolerance) {
    return CGAL::squared_distance(p1, p2) < tolerance * tolerance;
}

//////////////////////////////// TESTING FUNCTION ///////////////////////////////////////

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

void DTriangPlanner::print_paths() {
    std::cout << "Number of paths: " << _paths.size() << std::endl;

    for (const auto& path : _paths) {
        std::cout << "Path: ";
        for (const auto& point : path) {
            std::cout << "(" << point.x() << ", " << point.y() << ") ";
        }
        std::cout << std::endl;
    }
}


void DTriangPlanner::print_edge_vertices(const Edge& edge) {
    // Extract the face and index from the edge
    auto face = edge.first;
    int index = edge.second;

    // The vertices of the edge are the next two vertices in the face
    Point_2 vertex1 = face->vertex((index + 1) % 3)->point();
    Point_2 vertex2 = face->vertex((index + 2) % 3)->point();

    std::cout << "Edge vertices: (" 
            << vertex1 << "), (" 
            << vertex2 << ")" << std::endl;
}


Point_2 DTriangPlanner::transform_to_car_frame(const Point_2& global_pt, double car_x, double car_y, double car_yaw) {

    // Translate the point
    double translated_x = CGAL::to_double(global_pt.x()) - car_x;
    double translated_y = CGAL::to_double(global_pt.y()) - car_y;

    // Rotate the point
    double rotated_x = translated_x * cos(car_yaw) + translated_y * sin(car_yaw);
    double rotated_y = -translated_x * sin(car_yaw) + translated_y * cos(car_yaw);

    return Point_2(rotated_x, rotated_y);
}


