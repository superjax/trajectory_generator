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
class MainWindow;

class ScribbleArea : public QWidget
{
    Q_OBJECT

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ScribbleArea(MainWindow *parent);

    void setPenColor(const QColor &newColor);
    void setPenWidth(int newWidth);

    QColor penColor() const { return pen_color_; }
    int penWidth() const { return pen_width_; }
    const trajVec& getRoughTrajectory() { return rough_trajectory_; }
    void plotSmoothTrajectory(const trajVec& smooth_traj);
    void lockScreen(bool setLocked) { locked_screen_ = setLocked; }

    static const double room_width_;
    static const double room_height_;


public slots:
    void clearImage();
    void clearTrajectory();
    void deletePoint();
    void print();
    void setAltitude(double alt);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;


private:
    void drawBackground();
    void addPoint(const QPoint &point);
    void movePoint(const QPoint& point, int id);
    void drawRoughTraj();
    void drawLineTo(const QPoint &endPoint, const QColor &pen_color, bool draw_point=false);
    void drawPoint(const Vector2d &pix, const QColor &color);
    void drawPoint(const QPoint& pt, const QColor &color);
    void resizeImage(QImage *image_, const QSize &newSize);

    MainWindow* parent_;
    int pen_width_;
    const QColor pen_color_ = Qt::cyan;
    const QColor selected_color_ = Qt::red;
    QImage image_;
    QPoint last_point_;
    double trajectory_altitude_;

    double pixel_to_meters_;
    int midpixel_x_;
    int midpixel_y_;

    bool locked_screen_;

    int hovered_point_idx_ = -1;
    int selected_point_idx_ = -1;
    bool point_clicked_ = false;
    const int point_radius_ = 5;

    trajVec rough_trajectory_;

};

#endif
