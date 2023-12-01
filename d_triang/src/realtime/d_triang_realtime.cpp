#include "d_triang_realtime.hpp"

bool DTRealTime::plan_one_step(const std::vector<Point_2>& cones){

    // std::cout << "MATCHING_THRESHOLD" << MATCHING_THRESHOLD << std::endl;

    bool match = match_new_cones(cones);
    if (match) reuse_path();
    else replan();

    return match;
}


bool DTRealTime::match_new_cones(const std::vector<Point_2>& new_cones){
    
    // If the last step saw too few cones when just recompute
    if (_cones.size() < 4){
        _cones = new_cones;
        // _cones.clear();
        return false;
    }

    for (const auto& last_cone : _cones){
        // Distance from car to a last cone
        double car_to_last_cone_dist_sq = pow(CGAL::to_double(last_cone.x()),2) + pow(CGAL::to_double(last_cone.y()),2);
        bool inside_critical_zone = car_to_last_cone_dist_sq < CRITICAL_RANGE_SQ;
        
        if (!inside_critical_zone)
            continue;

        // Find corresponding cones in the new pattern
        // Find nearest new cone
        Point_2 closest_cone;
        double min_dist_sq = std::numeric_limits<double>::max();        
        for (const auto& new_cone : new_cones) {
            // Distance from that last cone to one new cone
            double last_cone_to_new_dist_sq = CGAL::to_double(CGAL::squared_distance(last_cone, new_cone));           
            if (last_cone_to_new_dist_sq < min_dist_sq) {
                min_dist_sq = last_cone_to_new_dist_sq;
                closest_cone = new_cone;
            }
        }

        if (min_dist_sq > MATCHING_THRESHOLD){
            // _cones.clear();
            _cones = new_cones;
            return false; // New patter doesn't match
        }       
    }
    // Matched    
    _cones = new_cones;
    return true;
}

void DTRealTime::replan(){
    set_cones(_cones);
    plan();

    _path.clear();
    std::vector<std::pair<Point_2, std::array<Point_2, 2>>> path = get_best_path_2();
    for (auto& waypoint : path){
        Point_2 point = waypoint.first;
        _path.push_back(point);
    }
}

void DTRealTime::reuse_path(){

    _path.clear();

    std::vector<std::pair<Point_2, std::array<Point_2, 2>>> path = get_best_path_2();

    for (auto& waypoint : path){

        std::array<Point_2, 2> old_cones = waypoint.second;
        std::array<Point_2, 2> new_cones;

        // Loop over 2 old cones on 2 sides of the current waypoint
        for (int i = 0; i < 2; i++){
            // Find the corresponding new cone of the current old cone
            for (auto& new_cone : _cones){
                double old_new_dist = CGAL::to_double(CGAL::squared_distance(old_cones[i], new_cone));
                if (old_new_dist < MATCHING_THRESHOLD){
                    new_cones[i] = new_cone;
                }
            }
        }
        Point_2 new_waypoint = CGAL::midpoint(new_cones[0],new_cones[1]);
        _path.push_back(new_waypoint);
    }
}


double DTRealTime::get_steering(){

    // std::vector<std::pair<Point_2, std::array<Point_2, 2>>> path = get_best_path_2();

    Point_2 car_pos(0,0);
    Point_2 goal = _path.at(1); // Goal is the midpoint of starting edge
    double car_to_goal_dist = CGAL::to_double(CGAL::squared_distance(car_pos, goal));
    // If too near, next point
    if (car_to_goal_dist < MIN_DIST_GOAL)
        goal = _path.at(2); 

    double angle_diff = atan2(CGAL::to_double(goal.y()), CGAL::to_double(goal.x()));
    if (angle_diff > M_PI_2)
        throw std::runtime_error("Turning angle too large.");

    // Convert angle different to turning angle (eufs default : -0.52 -> 0.52)

    return angle_diff; // JUST FOR TESTING
}