#include "geometry_test_fixture.cpp"


TEST_F(DTriangPlannerTestFixture, ExpandTest1) {

    // Edge first_edge = _planner.triangulate();
    // Edge previous_edge;
    
    // _planner.expand(first_edge);

    _planner.plan();

    std::vector<std::vector<Point_2>> paths = _planner.get_paths();

    auto best_path_2 = _planner.get_best_path_2();


    std::cout << "best_path_2.size(): " << best_path_2.size() << std::endl;
    _planner.print_path_2(best_path_2);

    

    ASSERT_TRUE(paths.size() > 0);
}