#ifndef DT_REALTIME_H
#define DT_REALTIME_H

#include "d_triang_planner.hpp"


/**
match_new_cones will be called every time step at the rate of the path planner
Inside critical zone, all the cones from last step must be matched
Otherwise, trigger adaptive behavior
    Recompute triangulation 
    Reject new cones

Use custome Point or Point_2?

USAGE: 
    match_new_cones()
        -> true (matched) : reuse_path() 
        -> false : plan new path : replan() 
    get_steering(): Ackerman()

TO RESEARCH:
    Save cone pattern from one last step or multiple last step and compute moving average/use moving median
        Re-plan the path would then be possibly less heavy


**/
class DTRealTime : public DTriangPlanner {

public:

    /*
    Planning workflow
    */
    bool plan_one_step(const std::vector<Point_2>& cones);


    /*
    Get the steering value to publish
    */
    double get_steering();


protected:
    bool match_new_cones(const std::vector<Point_2>& cones);

    /*
    If matched
    Find the new corresponding waypoint for each previous path's waypoint
    How: examine the last best path, for each waypoint, get 2 cones, find the corresponding
    2 cones from the new cone pattern, calculate the midpoint, that will be the new corresponding waypoint
    */
    void reuse_path();

    /*
    If not matched, perform triangulation planner from scratch 
    */
    void replan();



private:


    std::vector<Point_2> _cones;
    //std::vector<Point_2> _last_cones;
    
    std::vector<Point_2> _path;



private:
    const double CONST_CAR_VEL = 5.0; // (m/s)
    const int PATH_PLANNING_RATE = 30; // (Hz)


    // This should limit by 4 nearest cones ahead
    const double CRITICAL_RANGE = 20.0;   
    const double CRITICAL_RANGE_SQ = pow(CRITICAL_RANGE,2);   

    // Account for both movement of car and error in computervision
    const double error_by_movement_ratio = 1.5;
    const double max_error_by_movement = (CONST_CAR_VEL / PATH_PLANNING_RATE) * error_by_movement_ratio; // (m)
    
    const double MAX_ERROR_BY_COMPUTERVISION = 0.2; // (m)

    const double MATCHING_THRESHOLD = max_error_by_movement + MAX_ERROR_BY_COMPUTERVISION; 
    // = 0.45

    // Goal must not be too near the car
    const double MIN_DIST_GOAL = 5.0;
};







#endif