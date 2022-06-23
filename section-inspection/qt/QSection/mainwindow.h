#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "geometry/point.hpp"
#include "logger/logger.h"
#include <QMainWindow>
#include <QMap>
#include <QTableWidget>
#include <QtDataVisualization/Q3DScatter>

QT_BEGIN_NAMESPACE
namespace Ui {
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    Q3DScatter *create3DScatterGraph();

    void addDataSeries(Q3DScatter *graph, const ns_geo::PointSet3d &pts, const QColor &color);

    ns_geo::PointSet3d readPointsLaser(const std::string &filename);

    ns_geo::PointSet3d readPointsStation(const std::string &filename);

    void displayPtsInfo(QTableWidget *tab, const ns_geo::PointSet3d &pts);

    void connection();

    void init();

  private:
    Ui::MainWindow *ui;
    Q3DScatter *graphLaser;
    Q3DScatter *graphStation;
    Q3DScatter *graphCoord;
    ns_geo::PointSet3d laserPts;
    ns_geo::PointSet3d stationPts;
    ns_geo::PointSet3d newStationPts;
};
#endif // MAINWINDOW_H
