#include <gtest/gtest.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include "../../include/geometry/d_triang_planner.hpp"

// typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
// typedef CGAL::Delaunay_triangulation_2<K> DelaunayTriangulation;
// typedef DelaunayTriangulation::Edge Edge;
typedef DelaunayTriangulation::Vertex_handle Vertex_handle;
typedef DelaunayTriangulation::Face_handle Face_handle;

// typedef DelaunayTriangulation::Point Point;


class DTriangPlannerTest : public ::testing::Test {
protected:
    DelaunayTriangulation dt;
    DTriangPlanner planner;

    virtual void SetUp() {
        std::vector<Point_2> points_local;

        // Cones config from near_steep_turn.yaml, converted to car's local frame 
        points_local.push_back(Point_2(8.14437, -4.28801));
        points_local.push_back(Point_2(15.3254, -5.44316));
        points_local.push_back(Point_2(19.8133, -10.0369));
        points_local.push_back(Point_2(17.6274, -16.8421));
        points_local.push_back(Point_2(9.23534, 5.02372));
        points_local.push_back(Point_2(18.8647, 2.8406));
        points_local.push_back(Point_2(27.2126, -3.01572));
        points_local.push_back(Point_2(29.0814, -14.024));

        dt.insert(points_local.begin(), points_local.end()); 
    }

    bool are_points_vertices_of_edge(const DelaunayTriangulation::Edge& edge, const Point_2& p1, const Point_2& p2) {
        auto face = edge.first;
        int i = edge.second;
        Point_2 v1 = face->vertex((i + 1) % 3)->point();
        Point_2 v2 = face->vertex((i + 2) % 3)->point();

        // The == operator checks for exact equality of the coordinates of these points.
        return ((v1 == p1 && v2 == p2) || (v1 == p2 && v2 == p1));
    }

    Face_handle find_face_with_vertices(const Point_2& v1, const Point_2& v2, const Point_2& v3) {
        for (auto fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); ++fit) {
            Face_handle face = fit;
            std::vector<Point_2> face_vertices = {
                face->vertex(0)->point(),
                face->vertex(1)->point(),
                face->vertex(2)->point()
            };

            if (std::find(face_vertices.begin(), face_vertices.end(), v1) != face_vertices.end() &&
                std::find(face_vertices.begin(), face_vertices.end(), v2) != face_vertices.end() &&
                std::find(face_vertices.begin(), face_vertices.end(), v3) != face_vertices.end()) {
                return face;
            }
        }

        return nullptr; // Face not found
    }

    Edge get_edge_from_face_and_vertices(Face_handle face, const Point_2& v1, const Point_2& v2) {
        for (int i = 0; i < 3; ++i) {
            Point_2 p1 = face->vertex((i + 1) % 3)->point(); // Vertex after i
            Point_2 p2 = face->vertex((i + 2) % 3)->point(); // Vertex after (i+1)

            if ((p1 == v1 && p2 == v2) || (p1 == v2 && p2 == v1)) {
                // std::cout << "p1 : " << p1 << std::endl;
                // std::cout << "p2 : " << p2 << std::endl;
                return Edge(face, i);
            }
        }
        throw std::runtime_error("Edge not found in the given face.");
    }

    void print_face_vertices(DelaunayTriangulation::Face_handle face) {
        if (!face->is_valid()) {
            std::cerr << "Invalid face handle." << std::endl;
            return;
        }

        for (int i = 0; i < 3; ++i) {
            Point_2 vertex = face->vertex(i)->point();
            std::cout << "Vertex " << i << ": (" << vertex.x() << ", " << vertex.y() << ")" << std::endl;
        }
    }

    void print_edge_vertices(const Edge& edge) {
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
};

TEST_F(DTriangPlannerTest, GetNextEdges1) {

    Point_2 starting_edge_p1(15.3254, -5.44316);
    Point_2 starting_edge_p2(27.2126, -3.01572);
    
    Point_2 previous_edge_p1(18.8647, 2.8406);
    Point_2 previous_edge_p2(15.3254, -5.44316);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, previous_edge_p1);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge = get_edge_from_face_and_vertices(previous_face, previous_edge_p1, previous_edge_p2);

    // These to check functions in test class
    {
        // Both edge belong to previous_face
        ASSERT_TRUE((current_edge.first == previous_face) && (previous_edge.first == previous_face));

        Point_2 starting_edge_vertex1 = previous_face->vertex((current_edge.second + 1) % 3)->point();
        Point_2 starting_edge_vertex2 = previous_face->vertex((current_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(current_edge, starting_edge_vertex1, starting_edge_vertex2));

        Point_2 previous_edge_vertex1 = previous_face->vertex((previous_edge.second + 1) % 3)->point();
        Point_2 previous_edge_vertex2 = previous_face->vertex((previous_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(previous_edge, previous_edge_vertex1, previous_edge_vertex2));
    }

    std::vector<Edge> next_edges = planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());


    Point_2 next_edge1_p1(15.3254, -5.44316);
    Point_2 next_edge1_p2(19.8133, -10.0369);

    Point_2 next_edge2_p1(27.2126, -3.01572);
    Point_2 next_edge2_p2(19.8133, -10.0369);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}


TEST_F(DTriangPlannerTest, GetNextEdges2) {

    Point_2 starting_edge_p1(27.2126, -3.01572);
    Point_2 starting_edge_p2(19.8133, -10.0369);

    Point_2 previous_edge_p1(15.3254, -5.44316);    
    Point_2 previous_edge_p2(27.2126, -3.01572);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, previous_edge_p1);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge = get_edge_from_face_and_vertices(previous_face, previous_edge_p1, previous_edge_p2);

    // These to check functions in test class
    {
        // Both edge belong to previous_face
        ASSERT_TRUE((current_edge.first == previous_face) && (previous_edge.first == previous_face));

        Point_2 starting_edge_vertex1 = previous_face->vertex((current_edge.second + 1) % 3)->point();
        Point_2 starting_edge_vertex2 = previous_face->vertex((current_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(current_edge, starting_edge_vertex1, starting_edge_vertex2));

        Point_2 previous_edge_vertex1 = previous_face->vertex((previous_edge.second + 1) % 3)->point();
        Point_2 previous_edge_vertex2 = previous_face->vertex((previous_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(previous_edge, previous_edge_vertex1, previous_edge_vertex2));
    }

    std::vector<Edge> next_edges = planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(27.2126, -3.01572);
    Point_2 next_edge1_p2(29.0814, -14.024);

    Point_2 next_edge2_p1(29.0814, -14.024);
    Point_2 next_edge2_p2(19.8133, -10.0369);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTest, GetNextEdges3) {

    Point_2 starting_edge_p1(27.2126, -3.01572);
    Point_2 starting_edge_p2(19.8133, -10.0369);

    Point_2 previous_edge_p1(29.0814, -14.024);    
    Point_2 previous_edge_p2(19.8133, -10.0369);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, previous_edge_p1);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge = get_edge_from_face_and_vertices(previous_face, previous_edge_p1, previous_edge_p2);

    // These to check functions in test class
    {
        // Both edge belong to previous_face
        ASSERT_TRUE((current_edge.first == previous_face) && (previous_edge.first == previous_face));

        Point_2 starting_edge_vertex1 = previous_face->vertex((current_edge.second + 1) % 3)->point();
        Point_2 starting_edge_vertex2 = previous_face->vertex((current_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(current_edge, starting_edge_vertex1, starting_edge_vertex2));

        Point_2 previous_edge_vertex1 = previous_face->vertex((previous_edge.second + 1) % 3)->point();
        Point_2 previous_edge_vertex2 = previous_face->vertex((previous_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(previous_edge, previous_edge_vertex1, previous_edge_vertex2));
    }

    std::vector<Edge> next_edges = planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(27.2126, -3.01572);
    Point_2 next_edge1_p2(15.3254, -5.44316);

    Point_2 next_edge2_p1(15.3254, -5.44316);
    Point_2 next_edge2_p2(19.8133, -10.0369);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTest, GetNextEdges4) {

    Point_2 starting_edge_p1(15.3254, -5.44316);
    Point_2 starting_edge_p2(17.6274, -16.8421);

    Point_2 previous_edge_p1(19.8133, -10.0369);    
    Point_2 previous_edge_p2(15.3254, -5.44316);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, previous_edge_p1);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge = get_edge_from_face_and_vertices(previous_face, previous_edge_p1, previous_edge_p2);

    // These to check functions in test class
    {
        // Both edge belong to previous_face
        ASSERT_TRUE((current_edge.first == previous_face) && (previous_edge.first == previous_face));

        Point_2 starting_edge_vertex1 = previous_face->vertex((current_edge.second + 1) % 3)->point();
        Point_2 starting_edge_vertex2 = previous_face->vertex((current_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(current_edge, starting_edge_vertex1, starting_edge_vertex2));

        Point_2 previous_edge_vertex1 = previous_face->vertex((previous_edge.second + 1) % 3)->point();
        Point_2 previous_edge_vertex2 = previous_face->vertex((previous_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(previous_edge, previous_edge_vertex1, previous_edge_vertex2));
    }

    std::vector<Edge> next_edges = planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(8.14437, -4.28801);
    Point_2 next_edge1_p2(15.3254, -5.44316);

    Point_2 next_edge2_p1(17.6274, -16.8421);
    Point_2 next_edge2_p2(8.14437, -4.28801);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTest, GetNextEdges5) {

    Point_2 starting_edge_p1(15.3254, -5.44316);
    Point_2 starting_edge_p2(19.8133, -10.0369);

    Point_2 previous_edge_p1(17.6274, -16.8421);
    Point_2 previous_edge_p2(15.3254, -5.44316);    

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, previous_edge_p1);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge = get_edge_from_face_and_vertices(previous_face, previous_edge_p1, previous_edge_p2);

    // These to check functions in test class
    {
        // Both edge belong to previous_face
        ASSERT_TRUE((current_edge.first == previous_face) && (previous_edge.first == previous_face));

        Point_2 starting_edge_vertex1 = previous_face->vertex((current_edge.second + 1) % 3)->point();
        Point_2 starting_edge_vertex2 = previous_face->vertex((current_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(current_edge, starting_edge_vertex1, starting_edge_vertex2));

        Point_2 previous_edge_vertex1 = previous_face->vertex((previous_edge.second + 1) % 3)->point();
        Point_2 previous_edge_vertex2 = previous_face->vertex((previous_edge.second + 2) % 3)->point();
        ASSERT_TRUE(are_points_vertices_of_edge(previous_edge, previous_edge_vertex1, previous_edge_vertex2));
    }

    std::vector<Edge> next_edges = planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(27.2126, -3.01572);
    Point_2 next_edge1_p2(15.3254, -5.44316);

    Point_2 next_edge2_p1(27.2126, -3.01572);
    Point_2 next_edge2_p2(19.8133, -10.0369);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTest, FirstEdge1) {

    Point_2 starting_edge_p1(9.23534, 5.02372);
    Point_2 starting_edge_p2(8.14437, -4.28801);  
    Point_2 another_p_inside(15.3254, -5.44316);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, another_p_inside);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge;
    

    std::vector<Edge> next_edges = planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(9.23534, 5.02372);
    Point_2 next_edge1_p2(15.3254, -5.44316);

    Point_2 next_edge2_p1(8.14437, -4.28801);
    Point_2 next_edge2_p2(15.3254, -5.44316);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTest, FirstEdge2) {

    Point_2 starting_edge_p1(29.0814, -14.024);
    Point_2 starting_edge_p2(17.6274, -16.8421);  
    Point_2 another_p_inside(19.8133, -10.0369);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, another_p_inside);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge;


    std::vector<Edge> next_edges = planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(29.0814, -14.024);
    Point_2 next_edge1_p2(19.8133, -10.0369);

    Point_2 next_edge2_p1(17.6274, -16.8421);
    Point_2 next_edge2_p2(19.8133, -10.0369);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

TEST_F(DTriangPlannerTest, FirstEdge3) {

    Point_2 starting_edge_p1(18.8647, 2.8406);
    Point_2 starting_edge_p2(27.2126, -3.01572);  
    Point_2 another_p_inside(15.3254, -5.44316);

    Face_handle previous_face = find_face_with_vertices(starting_edge_p1, starting_edge_p2, another_p_inside);
    // print_face_vertices(previous_face);

    Edge current_edge = get_edge_from_face_and_vertices(previous_face, starting_edge_p1, starting_edge_p2);
    Edge previous_edge;
    

    std::vector<Edge> next_edges = planner.get_next_edges(current_edge, previous_edge);

    // Check the number of next edges
    EXPECT_EQ(3, next_edges.size());

    Point_2 next_edge1_p1(18.8647, 2.8406);
    Point_2 next_edge1_p2(15.3254, -5.44316);

    Point_2 next_edge2_p1(27.2126, -3.01572);
    Point_2 next_edge2_p2(15.3254, -5.44316);

    int true_time = 0; 
    for (int i = 1; i < 3; i++){
        bool one = are_points_vertices_of_edge(next_edges.at(i), next_edge1_p1, next_edge1_p2);
        bool two = are_points_vertices_of_edge(next_edges.at(i), next_edge2_p1, next_edge2_p2);
        if (one || two)
            true_time++;

        // Print out
        // print_edge_vertices(next_edge);

    }
    EXPECT_EQ(2, true_time);
}

