#include "d_triang_plot.hpp"

int main(int argc, char *argv[]) {

    QApplication app(argc, argv); 

    DTriangPlot plot = DTriangPlot();
    
    int config_number = 1;
    if (argc > 1)
        config_number = std::stoi(argv[1]);

    plot.set_config_num(config_number);    
    plot.plot();
    
    return app.exec();
}

Point_2 DTriangPlot::transform_to_car_frame(const Point_2& global_pt, double car_x, double car_y, double car_yaw) {

    // Translate the point
    double translated_x = CGAL::to_double(global_pt.x()) - car_x;
    double translated_y = CGAL::to_double(global_pt.y()) - car_y;

    // Rotate the point
    double rotated_x = translated_x * cos(car_yaw) + translated_y * sin(car_yaw);
    double rotated_y = -translated_x * sin(car_yaw) + translated_y * cos(car_yaw);

    return Point_2(rotated_x, rotated_y);
}

void DTriangPlot::plot(){
    
    read_cone_config();
    triangulate();
    update_global_points();
    std::vector<std::pair<Point_2, Point_2>> edges = get_edges();
                                                
    // PlotWidget widget(_points_global, _points_local, edges);
    _widget = std::make_shared<PlotWidget>(_points_global, _points_local, edges);    
    _widget->resize(800, 900); 
    _widget->show();
}

void DTriangPlot::set_config_num(int config_num){
    _config_num = config_num;
}


/*
* Convert global cones position to local car's frame
* In real code the car's pose is already in the origin
*/
void DTriangPlot::read_cone_config(){
    
    if (PATH_MAP.find(_config_num) == PATH_MAP.end()) {
        throw std::runtime_error("No cones layout config number " + std::to_string(_config_num));
    }
    std::string yaml_path = PATH_MAP.at(_config_num);

    // Read YAML file
    std::string base_path = "../../plotting/cones_layout/";  
    std::string full_path = base_path + yaml_path;
    YAML::Node config = YAML::LoadFile(full_path);
    const auto& points_node = config["points"];
    for (const auto& point_node : points_node) {
        double x = point_node["x"].as<double>();
        double y = point_node["y"].as<double>();
        _points_global.push_back(Point_2(x, y));
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
    for (Point_2 point_global :_points_global){
        Point_2 p_local = transform_to_car_frame(point_global,car_x,car_y,car_yaw);
        // Print out to write unit test
        std::cout << p_local.x() << " , " << p_local.y() << std::endl;
        _local_global_map[p_local] = point_global;
        _points_local.push_back(p_local);
    }
}

void DTriangPlot::update_global_points(){
    // Get the correct order of -- GLOBAL POINTS -- 
    _points_global.clear(); 
    for (const auto& point_local : _points_local) {
        Point_2 point_global = _local_global_map[point_local];
        _points_global.push_back(point_global);
    }
}

std::vector<std::pair<Point_2, Point_2>> DTriangPlot::get_edges(){
    // Get the edges
    std::vector<std::pair<Point_2, Point_2>> edges;
    for (auto it = _dt.finite_edges_begin(); it != _dt.finite_edges_end(); ++it) {
        Kernel::Segment_2 segment = _dt.segment(it);
        edges.emplace_back(segment.start(), segment.end());
    }
    return edges;
}
