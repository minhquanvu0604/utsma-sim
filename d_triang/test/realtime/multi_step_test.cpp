#include "realtime_test_fixture.cpp"

TEST_F(DTRealTimeTestFixture, MultiStepTest) {

    // std::vector<Point_2> step;
    // bool step1match = planner.plan_one_step(step);

    std::vector<Point_2> step1 = get_cone_layout("step1_1.yaml");
    std::vector<Point_2> step2 = get_cone_layout("step2_1.yaml");
    std::vector<Point_2> step3 = get_cone_layout("step3_1.yaml");

    bool step1_match = planner.plan_one_step(step1);
    bool step2_match = planner.plan_one_step(step2);
    bool step3_match = planner.plan_one_step(step3);

    ASSERT_TRUE(step1_match);
    ASSERT_TRUE(step2_match);
    ASSERT_TRUE(step3_match);

    // _planner.plan();

    // std::vector<std::vector<Point_2>> paths = _planner.get_paths();

    // std::cout << "Print best path 2: " << std::endl;
    // auto best_path_2 = _planner.get_best_path_2();
    // _planner.print_path_2(best_path_2);

    
    // ASSERT_TRUE(paths.size() > 0);
}