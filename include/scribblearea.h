#ifndef SCRIBBLEAREA_H
#define SCRIBBLEAREA_H

#include <iostream>

#include <QColor>
#include <QImage>
#include <QPoint>
#include <QWidget>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "trajectory.h"
#include "trajopt_ros.h"
#include "lqr.h"

using namespace Eigen;
using namespace std;

typedef vector<Vector3d, aligned_allocator<Vector3d>> trajVec;
class TrajectorySmoother;

class ScribbleArea : public QWidget
{
    Q_OBJECT

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ScribbleArea(QWidget *parent = 0);

    void setPenColor(const QColor &newColor);
    void setPenWidth(int newWidth);

    QColor penColor() const { return pen_color_; }
    int penWidth() const { return pen_width_; }
    const trajVec& getRoughTrajectory() { return rough_trajectory_; }
    void plotSmoothTrajectory(const trajVec& smooth_traj);

    static const double room_width_;
    static const double room_height_;


public slots:
    void clearImage();
    void print();

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    inline void setAltitude(double alt) { trajectory_altitude_ = -alt; }


private:
    void drawBackground();
    void addPoint(const Vector3d &point);
    void addPoint(const QPoint &point);
    void drawLineTo(const QPoint &endPoint, QColor& pen_color, bool draw_point=false);
    void resizeImage(QImage *image_, const QSize &newSize);

    int pen_width_;
    QColor pen_color_;
    QImage image_;
    QPoint last_point_;
    double trajectory_altitude_;

    double pixel_to_meters_;
    int midpixel_x_;
    int midpixel_y_;

    bool locked_screen_;

    trajVec rough_trajectory_;

};

#endif
