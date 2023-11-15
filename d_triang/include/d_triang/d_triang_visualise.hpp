#ifndef D_TRIANG_VISUALISE_H
#define D_TRIANG_VISUALISE_H


#include <iostream>
#include <vector>

#include <QApplication>
#include <QWidget>
#include <QPainter>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Delaunay_triangulation_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Delaunay_triangulation_2<Kernel> DelaunayTriangulation;

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
    
            // NEEDS REVISION ------------------------- NNNNNNNNNN
            // std::cout << "height:  " << height() << std::endl;
            // Scale 
            if (_x_max - _x_min > _y_max - _y_min){
                _scale = width() / (_x_max - _x_min);
                std::cout << "Scaled by width" << std::endl;
            }
            else 
                _scale = 900 / (_y_max - _y_min);


            // Scale points to fit within the axis limits
            for (int i = 0; i < tr_pts.size(); i++){            
                double x = (CGAL::to_double(tr_pts.at(i).x()) - _x_min) * _scale;
                double y = (CGAL::to_double(tr_pts.at(i).y()) - _y_min) * _scale;
                // std::cout << "Location plotted: x = " << x << " y = " << y << std::endl; 
                _plot_location_pts.push_back(QPointF(x,y));
            }

            // Calculate plot location for edges 
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

                std::pair<QPointF, QPointF> edge_QPointF = std::make_pair(start, end);
                _plot_location_edges.push_back(edge_QPointF);
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
        const int vertice_R = 5;
        for (int i = 0; i < _plot_location_pts.size(); i++){

            // Plot cones
            painter.drawEllipse(_plot_location_pts.at(i), vertice_R, vertice_R);
            
            // Write local coordinates
            QString text = QString("(%1, %2)").arg(CGAL::to_double(_points_local.at(i).x())).arg(CGAL::to_double(_points_local.at(i).y()));
            // Measure the text size
            QFontMetrics metrics(painter.font());
            int textWidth = metrics.horizontalAdvance(text);
            int textHeight = metrics.height();
            // Adjust the text position to be above and centered over the point
            QPointF textPos_local = _plot_location_pts.at(i) - QPointF(textWidth / 2, textHeight + vertice_R);
            painter.drawText(textPos_local, text);        

            // Write global coordinates
            text = QString("(%1, %2)").arg(CGAL::to_double(_points_global.at(i).x())).arg(CGAL::to_double(_points_global.at(i).y()));
            // Adjust the text position to be above and centered over the point
            QPointF textPos_global = _plot_location_pts.at(i) - QPointF(textWidth / 2, textHeight*2 + vertice_R);
            painter.drawText(textPos_global, text);             
            }


        // Plot the edges
        for (const auto& edge : _plot_location_edges){
            painter.drawLine(edge.first, edge.second);
        }
    }

private:
    // Input geometries
    std::vector<Point_2> _points_global;
    std::vector<Point_2> _points_local;
    std::vector<std::pair<Point_2, Point_2>> _edges;
    
    // Geometries ready to plot
    std::vector<QPointF> _plot_location_pts;
    std::vector<std::pair<QPointF, QPointF>> _plot_location_edges;

    double _x_min = std::numeric_limits<double>::max();
    double _x_max = std::numeric_limits<double>::lowest();
    double _y_min = _x_min;
    double _y_max = _x_max;

    double _scale = 1.0;
};

#endif