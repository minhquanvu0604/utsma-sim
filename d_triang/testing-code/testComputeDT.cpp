#include <iostream>
#include <vector>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/point_generators_2.h>

#include <QApplication>
#include <QWidget>
#include <QPainter>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;


Point_2 transform_to_car_frame(const Point_2& global_pt) {
    const double car_x = 20.990251;
    const double car_y = 13.411934;
    const double car_yaw = 0.013218;

    // Translate the point
    double translated_x = CGAL::to_double(global_pt.x()) - car_x;
    double translated_y = CGAL::to_double(global_pt.y()) - car_y;

    // Rotate the point
    double rotated_x = translated_x * cos(car_yaw) + translated_y * sin(car_yaw);
    double rotated_y = -translated_x * sin(car_yaw) + translated_y * cos(car_yaw);

    return Point_2(rotated_x, rotated_y);
}

class PlotWidget : public QWidget {
public:
    PlotWidget(
        const std::vector<Point_2>& points_global, 
        const std::vector<Point_2>& points_local, 
        const std::vector<std::pair<Point_2, Point_2>>& edges, 
        QWidget* parent = nullptr
        )
        : QWidget(parent), _points_global(points_global), _points_local(points_local), _edges(edges){

            

            std::vector<Point_2> tr_pts;
            // Location to plot the points on Qt Windows
            for (const auto& point : _points_local){
                auto tr_p = change_frame(point);
                double mirrored_x = -CGAL::to_double(tr_p.x()); // Mirror the x-coordinate
                double y = CGAL::to_double(tr_p.y());
                tr_pts.push_back(Point_2(mirrored_x, y));
            }

            update_bounds(tr_pts);
    
            std::cout << "height:  " << height() << std::endl;
            // Scale 
            if (_x_max - _x_min > _y_max - _y_min){
                _scale = width() / (_x_max - _x_min);
                std::cout << "Scale width" << std::endl;
            }
            else 
                _scale = 900 / (_y_max - _y_min);


            // Scale points to fit within the axis limits
            for (int i = 0; i < tr_pts.size(); i++){            
                double x = (CGAL::to_double(tr_pts.at(i).x()) - _x_min) * _scale;
                double y = (CGAL::to_double(tr_pts.at(i).y()) - _y_min) * _scale;
                // std::cout << "Location plotted: x = " << x << " y = " << y << std::endl; 
                _plot_location_pts_p.push_back(QPointF(x,y));
            }
        }
            

protected:
    /*
     * Convert (x y) from Qt frame (right down) to robot frame (up left)
     * It is a conversion from global frame (initial robot frame) to local frame (Qt frame)
    */
    Point_2 change_frame(Point_2 pt_global_coord) {
        double new_x = 1000;
        double new_y = 1000;
        double new_yaw = M_PI/2;

        // Translate the point
        double translated_x = CGAL::to_double(pt_global_coord.x()) - new_x;
        double translated_y = CGAL::to_double(pt_global_coord.y()) - new_y;

        // Rotate the point
        double rotated_x = translated_x * cos(new_yaw) + translated_y * sin(new_yaw);
        double rotated_y = -translated_x * sin(new_yaw) + translated_y * cos(new_yaw);

        return Point_2(rotated_x, rotated_y);
    }

    /*
     * Set the min max of x and y to later make the window bounds all the points nicely
    */    
    void update_bounds(const std::vector<Point_2>& tr_pts) {
        for (const auto pt : tr_pts) {
            _x_min = std::min(_x_min, CGAL::to_double(pt.x()));
            _x_max = std::max(_x_max, CGAL::to_double(pt.x()));
            _y_min = std::min(_y_min, CGAL::to_double(pt.y()));
            _y_max = std::max(_y_max, CGAL::to_double(pt.y()));
        }
        // Adjust bounds slightly for padding
        _x_min -= 10; _x_max += 10;
        _y_min -= 10; _y_max += 10;
    }

    void paintEvent(QPaintEvent *) override {

        // A QPainter object is typically created and used within the scope of a painting function like paintEvent. 
        // This is because QPainter is designed to be used only during painting operations, and its lifecycle is managed accordingly.
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);        
        
        // Plot the vertices
        int vertice_R = 5;
        for (int i = 0; i < _plot_location_pts_p.size(); i++){

            // Plot cones
            painter.drawEllipse(_plot_location_pts_p.at(i), vertice_R, vertice_R);
            
            // Write local coordinates
            QString text = QString("(%1, %2)").arg(CGAL::to_double(_points_local.at(i).x())).arg(CGAL::to_double(_points_local.at(i).y()));
            // Measure the text size
            QFontMetrics metrics(painter.font());
            int textWidth = metrics.horizontalAdvance(text);
            int textHeight = metrics.height();
            // Adjust the text position to be above and centered over the point
            QPointF textPos_local = _plot_location_pts_p.at(i) - QPointF(textWidth / 2, textHeight + vertice_R);
            painter.drawText(textPos_local, text);        

            // Write global coordinates
            text = QString("(%1, %2)").arg(CGAL::to_double(_points_global.at(i).x())).arg(CGAL::to_double(_points_global.at(i).y()));
            // Adjust the text position to be above and centered over the point
            QPointF textPos_global = _plot_location_pts_p.at(i) - QPointF(textWidth / 2, textHeight*2 + vertice_R);
            painter.drawText(textPos_global, text);             
            }

        // Calculate plot location for edges and plot them
        for (const auto& edge : _edges){
            
            // First point
            double mirrored_x1 = -CGAL::to_double(change_frame(edge.first).x());
            double y1 = CGAL::to_double(change_frame(edge.first).y());

            double x_1 = (mirrored_x1 - _x_min) * _scale;
            double y_1 = (y1 - _y_min) * _scale;            
            QPointF start(x_1, y_1);

            // Second point
            double mirrored_x2 = -CGAL::to_double(change_frame(edge.second).x());
            double y2 = CGAL::to_double(change_frame(edge.second).y());

            double x_2 = (mirrored_x2 - _x_min) * _scale;
            double y_2 = (y2 - _y_min) * _scale; 
            QPointF end(x_2, y_2);

            painter.drawLine(start, end);
        }


}

private:
    std::vector<Point_2> _points_global;
    std::vector<Point_2> _points_local;
    std::vector<std::pair<Point_2, Point_2>> _edges;

    // std::vector<Point_2> _plot_location_pts_p;
    std::vector<QPointF> _plot_location_pts_p;

    double _x_min = std::numeric_limits<double>::max();
    double _x_max = std::numeric_limits<double>::lowest();
    double _y_min = _x_min;
    double _y_max = _x_max;

    double _scale = 1.0;
};


int main(int argc, char *argv[]) {
    // Generate a set of random points
    std::vector<Point_2> points_global;
    std::vector<Point_2> points_local;

    points_global.push_back(Point_2(26.872997, 9.791506)); 
    points_global.push_back(Point_2(38.371532, 9.348429));
    points_global.push_back(Point_2(49.792950, 8.017611));
    points_global.push_back(Point_2(26.821278, 17.784601));
    points_global.push_back(Point_2(38.313610, 17.302580));
    points_global.push_back(Point_2(49.748211, 16.086134));
    points_global.push_back(Point_2(60.851669, 13.144850));

    // Create map for global and corresponding local point
    std::map<Point_2,Point_2> local_global_map;
    // Convert to car's frame
    for (auto point_global : points_global){
        auto p_local = transform_to_car_frame(point_global);
        local_global_map[p_local] = point_global;
        points_local.push_back(p_local);
    }

    // Compute Delaunay Triangulation
    DelaunayTriangulation dt;
    dt.insert(points_local.begin(), points_local.end());

    // Get the correct order of global points
    points_global.clear(); 
    for (const auto& point_local : points_local) {
        Point_2 point_global = local_global_map[point_local];
        points_global.push_back(point_global);
    }

    // Get the edges
    std::vector<std::pair<Point_2, Point_2>> edges;
    for (auto it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
        auto segment = dt.segment(it);
        edges.emplace_back(segment.start(), segment.end());
    }

    // Qt
    QApplication app(argc, argv);
    PlotWidget widget(points_global, points_local, edges);
    widget.resize(800, 900); 
    widget.show();

    return app.exec();


    // // Output the vertices of the Delaunay Triangulation
    // std::cout << "Delaunay Triangulation vertices:" << std::endl;
    // for (auto vertex = dt.finite_vertices_begin(); vertex != dt.finite_vertices_end(); ++vertex) {
    //     std::cout << vertex->point() << std::endl;
    // }

    // // Plotting all edges including boundary edges
    // // You need to modify your approach to handle the infinite edges as well (@TODO)
    // for (auto edge = dt.all__edgesbegin(); edge != dt.all__edgesend(); ++edge) {
    //     auto face = edge->first;
    //     int i = edge->second;

    //     if (!dt.is_infinite(face)) {
    //         auto vertex1 = face->vertex((i + 1) % 3)->point();
    //         auto vertex2 = face->vertex((i + 2) % 3)->point();

    //         // Convert for ploting
    //         std::vector<double> edge_x = {CGAL::to_double(vertex1.x()), CGAL::to_double(vertex2.x())};
    //         std::vector<double> edge_y = {CGAL::to_double(vertex1.y()), CGAL::to_double(vertex2.y())};
    //     }
    // }

    // // Compute midpoints of internal edges and store in a vector
    // std::vector<Point_2> midpoints;
    // for (auto edge = dt.finite__edgesbegin(); edge != dt.finite__edgesend(); ++edge) { 
    //     // Finite edges are those edges that are not part of the unbounded face of the triangulation.
        
    //     // Check if the face containing the edge is infinite
    //     // In CGAL, triangulations can have infinite faces, especially when dealing with Delaunay triangulations 
    //     // This check ensures that only internal edges (edges of finite faces) are considered.
    //     if (!dt.is_infinite(edge->first)) {
    //         // Compute the midpoint of the edge and store it in the vector
    //         // The edge->second gives the index of the vertex opposite the edge
    //         // (edge->second + 1) % 3 and (edge->second + 2) % 3 give the indices of the vertices forming the edge.
    //         Point_2 midpoint = CGAL::midpoint(edge->first->vertex((edge->second + 1) % 3)->point(),
    //                               edge->first->vertex((edge->second + 2) % 3)->point());
    //         midpoints.push_back(midpoint);
    //     }
    // }

    // // Output the midpoints
    // std::cout << "Midpoints of internal edges:" << std::endl;
    // for (const auto& midpoint : midpoints) {
    //     std::cout << midpoint << std::endl;
    // }

    return 0;
}
