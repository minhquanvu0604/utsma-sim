#include <iostream>
#include <vector>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <QApplication>
#include <QWidget>
#include <QPainter>


class PlotWidget : public QWidget {
public:
    PlotWidget(const std::vector<Point_2>& pts, QWidget* parent = nullptr) : QWidget(parent), points_(pts) {}

protected:

    std::vector<Point_2> transformPointsToLocalFrame() {
        
        double new_x = 0;
        double new_y = 0;
        double new_yaw = -M_PI/2;

        std::vector<Point_2> transformedPoints;
        for (auto& point : points_) {
            // Translate the point
            double translatedX = CGAL::to_double(point.x()) - new_x;
            double translatedY = CGAL::to_double(point.y()) - new_y;

            // Rotate the point
            double rotatedX = translatedX * cos(new_yaw) + translatedY * sin(new_yaw);
            double rotatedY = -translatedX * sin(new_yaw) + translatedY * cos(new_yaw);

            // Update the point with the new coordinates
            transformedPoints.push_back(Point_2(rotatedX, rotatedY));
            std::cout << "Point transformed: x = " << rotatedX << " y = " << rotatedY << std::endl; 
        }
        return transformedPoints;
    }

    void updateBounds(const std::vector<Point_2>& transformedPoints) {

        for (const auto point : transformedPoints) {
            xMin_ = std::min(xMin_, CGAL::to_double(point.x()));
            xMax_ = std::max(xMax_, CGAL::to_double(point.x()));
            yMin_ = std::min(yMin_, CGAL::to_double(point.y()));
            yMax_ = std::max(yMax_, CGAL::to_double(point.y()));
        }

        // Adjust bounds slightly for padding
        xMin_ -= 10; xMax_ += 10;
        yMin_ -= 10; yMax_ += 10;
    }

    void paintEvent(QPaintEvent *) override {

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        // Define the radius of the points
        int pointRadius = 5;
        
        // Define axis limits
        std::vector<Point_2> transformedPoints = transformPointsToLocalFrame();
        updateBounds(transformedPoints);
    
        // Scale factor based on widget size and axis limits
        double xScale = width() / (xMax_ - xMin_);
        double yScale = height() / (yMax_ - yMin_);

        // Draw the points
        // for (const auto& point : transformedPoints) {
        for (int i = 0; i < transformedPoints.size(); i++){
            // Scale points to fit within the axis limits
            double x = (CGAL::to_double(transformedPoints.at(i).x()) - xMin_) * xScale;
            double y = (CGAL::to_double(transformedPoints.at(i).y()) - yMin_) * yScale;


            QPointF qtPoint = QPointF(x,height() - y);
            painter.drawEllipse(qtPoint, pointRadius, pointRadius);
            
            // Prepare the text to display
            QString text = QString("(%1, %2)").arg(CGAL::to_double(points_.at(i).x())).arg(CGAL::to_double(points_.at(i).y()));
            // Measure the text size
            QFontMetrics metrics(painter.font());
            int textWidth = metrics.horizontalAdvance(text);
            int textHeight = metrics.height();
            // Adjust the text position to be above and centered over the point
            QPointF textPos = qtPoint - QPointF(textWidth / 2, textHeight + pointRadius);

            painter.drawText(textPos, text);        
            }
}

private:
    std::vector<Point_2> points_;

    double xMin_ = std::numeric_limits<double>::max();
    double xMax_ = std::numeric_limits<double>::lowest();
    double yMin_ = xMin_;
    double yMax_ = xMax_;
};
