#include "geometry_test_fixture.cpp"


TEST_F(DTriangPlannerTestFixture, ExpandTest1) {

    Edge first_edge = _planner.triangulate();
    Edge previous_edge;
    
    _planner.expand(first_edge);

    std::vector<std::vector<Point_2>> paths = _planner.get_paths();
    

    ASSERT_TRUE(paths.size() > 0);
}