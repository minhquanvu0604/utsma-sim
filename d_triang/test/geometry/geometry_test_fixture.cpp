#ifndef GEOMETRY_TEST_FIXTURE_H
#define GEOMETRY_TEST_FIXTURE_H

#include <gtest/gtest.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <yaml-cpp/yaml.h>


#include "../../include/geometry/d_triang_planner.hpp"

typedef DelaunayTriangulation::Face_handle Face_handle;


class DTriangPlannerTestFixture : public ::testing::Test {
protected:
    DelaunayTriangulation* _dt;
    DTriangPlanner _planner;

    virtual void SetUp() {

        _dt = _planner.get_triangulation_ptr();
        std::vector<Point_2> points_local = get_cone_layout("near_steep_turn.yaml");

        // // Print out cone layout
        // for(auto p : points_local){
        //     std::cout << p << std::endl;
        // }
        // std::cout << std::endl;

        _planner.set_cones(points_local);
    }

    std::vector<Point_2> get_cone_layout(std::string yaml_path){
        
        std::vector<Point_2> points_global;
        std::vector<Point_2> points_local;

        std::string base_path = "../plotting/cones_layout/";  
        std::string full_path = base_path + yaml_path;
        YAML::Node config = YAML::LoadFile(full_path);
        const auto& points_node = config["points"];
        for (const auto& point_node : points_node) {
            double x = point_node["x"].as<double>();
            double y = point_node["y"].as<double>();
            points_global.push_back(Point_2(x, y));
        }

        double car_x = 0.0, car_y = 0.0, car_yaw = 0.0;
        if (config["car_pose"]) {
            auto car_pose = config["car_pose"];
            if (car_pose["position"]) {
                car_x = car_pose["position"]["x"].as<double>();
                car_y = car_pose["position"]["y"].as<double>();
            }
            if (car_pose["orientation"]) 
                car_yaw = car_pose["orientation"]["yaw"].as<double>();
        } else 
            std::cerr << "Car pose not found in YAML file." << std::endl;

        // Convert to car's frame
        for (Point_2 point_global : points_global){
            Point_2 p_local = transform_to_car_frame(point_global,car_x,car_y,car_yaw);

            // // Print out to write unit test
            // std::cout << p_local.x() << " , " << p_local.y() << std::endl;

            points_local.push_back(p_local);
        }
        return points_local;
    }


    Point_2 transform_to_car_frame(const Point_2& global_pt, double car_x, double car_y, double car_yaw) {

        // Translate the point
        double translated_x = CGAL::to_double(global_pt.x()) - car_x;
        double translated_y = CGAL::to_double(global_pt.y()) - car_y;

        // Rotate the point
        double rotated_x = translated_x * cos(car_yaw) + translated_y * sin(car_yaw);
        double rotated_y = -translated_x * sin(car_yaw) + translated_y * cos(car_yaw);

        return Point_2(rotated_x, rotated_y);
    }

    bool are_points_vertices_of_edge(const DelaunayTriangulation::Edge& edge, const Point_2& p1, const Point_2& p2) {
        
        double tolerance = 1e-3;
        
        auto face = edge.first;

        int i = edge.second;
        Point_2 v1 = face->vertex((i + 1) % 3)->point();
        Point_2 v2 = face->vertex((i + 2) % 3)->point();

        bool close_p1_v1 = CGAL::squared_distance(p1, v1) < tolerance * tolerance;
        bool close_p2_v2 = CGAL::squared_distance(p2, v2) < tolerance * tolerance;
        bool close_p1_v2 = CGAL::squared_distance(p1, v2) < tolerance * tolerance;
        bool close_p2_v1 = CGAL::squared_distance(p2, v1) < tolerance * tolerance;

        return (close_p1_v1 && close_p2_v2) || (close_p1_v2 && close_p2_v1);
    }

    bool is_approx_equal(const Point_2& p1, const Point_2& p2, double tolerance) {
        return CGAL::squared_distance(p1, p2) < tolerance * tolerance;
    }

    Face_handle find_face_with_vertices(const Point_2& v1, const Point_2& v2, const Point_2& v3) {
        double tolerance = 1e-3;
        for (auto fit = _dt->finite_faces_begin(); fit != _dt->finite_faces_end(); ++fit) {
            Face_handle face = fit;
            std::vector<Point_2> face_vertices = {
                face->vertex(0)->point(),
                face->vertex(1)->point(),
                face->vertex(2)->point()
            };

            if (std::any_of(face_vertices.begin(), face_vertices.end(), [&](const Point_2& p) { return is_approx_equal(p, v1, tolerance); }) &&
                std::any_of(face_vertices.begin(), face_vertices.end(), [&](const Point_2& p) { return is_approx_equal(p, v2, tolerance); }) &&
                std::any_of(face_vertices.begin(), face_vertices.end(), [&](const Point_2& p) { return is_approx_equal(p, v3, tolerance); })) {
                return face;
            }
        }
        throw std::runtime_error("Face not found in the given vertices.");
        // return nullptr; // Face not found
    }

    Edge get_edge_from_face_and_vertices(Face_handle face, const Point_2& v1, const Point_2& v2) {
        double tolerance = 1e-3;
        for (int i = 0; i < 3; ++i) {
            Point_2 p1 = face->vertex((i + 1) % 3)->point(); // Vertex after i
            Point_2 p2 = face->vertex((i + 2) % 3)->point(); // Vertex after (i+1)

            if ((is_approx_equal(p1, v1, tolerance) && is_approx_equal(p2, v2, tolerance)) || 
                (is_approx_equal(p1, v2, tolerance) && is_approx_equal(p2, v1, tolerance))) {
                // Found the edge with tolerance
                return Edge(face, i);
            }
        }
        throw std::runtime_error("Edge not found in the given face with tolerance.");
    }

    void print_face_vertices(DelaunayTriangulation::Face_handle face) {
        if (!face->is_valid()) {
            std::cerr << "Invalid face handle." << std::endl;
            return;
        }

        std::cout << "Print face vertices: " << std::endl;

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

#endif