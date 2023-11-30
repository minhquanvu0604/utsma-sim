#ifndef D_TRIANG_PLANNER_REALTIME_H
#define D_TRIANG_PLANNER_REALTIME_H

#include "d_triang_planner.hpp"



/*



Use custome Point or Point_2?
*/
class DTriangPlannerRealTime {

public:
    void set_new_cones(std::vector<Point_2> cones){

    }


private:
    DTriangPlanner _planner;

    std::vector<Point_2> _approved_last_cones;


};







#endif