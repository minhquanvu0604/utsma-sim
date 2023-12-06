#include "d_triang_planner_color_light_test_fixture.cpp"

TEST_F(DTriangPlannerColorLightTest, Blue) {
    
    std::vector<DTCL::Cone> group_1 = {
        DTCL::Cone(Point_2(2, 1), 0),
        DTCL::Cone(Point_2(3, 2.2), 0),
        DTCL::Cone(Point_2(8, 3.5), 0)
    };
    double track_width = 3.0;

    std::sort(group_1.begin(), group_1.end(), [](const DTCL::Cone& a, const DTCL::Cone& b) {
        return a.point.x() < b.point.x();
    });

    std::vector<Point_2> path_group_1 = planner->process_group_1(group_1, 0);


    // Define your expected result based on the offset_to_left direction
    std::vector<Point_2> expected_points = {
        Point_2(0,0),
        Point_2(2, 1 - track_width/2),
        Point_2(3, 2.2 - track_width/2),
        Point_2(8, 3.5 - track_width/2)
    };

    ASSERT_EQ(path_group_1.size(), expected_points.size());

    for (size_t i = 0; i < path_group_1.size(); ++i) {
        EXPECT_DOUBLE_EQ(CGAL::to_double(path_group_1[i].x()), CGAL::to_double(expected_points[i].x()));
        EXPECT_DOUBLE_EQ(CGAL::to_double(path_group_1[i].y()), CGAL::to_double(expected_points[i].y()));
    }

    // std::cout << "Printing offset line:" << std::endl;
    // for (auto& waypt : path_group_1){
    //     std::cout << waypt << std::endl;
    // }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
