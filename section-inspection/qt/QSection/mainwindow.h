#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "geometry/point.hpp"
#include "pcl-1.12/pcl/kdtree/kdtree_flann.h"
#include "sline.h"
#include <QMainWindow>
#include <QMap>
#include <QTableWidget>
#include <QtDataVisualization/Q3DScatter>

struct SectionPair {
  public:
    /**
     * @brief the members
     */
    ns_geo::Point2d lasLeft;
    ns_geo::Point2d lasRight;
    ns_geo::Point2d staLeft;
    ns_geo::Point2d staRight;

  public:
    /**
     * @brief construct a new SectionPair object
     */
    SectionPair(const ns_geo::Point2d &lasLeft, const ns_geo::Point2d &lasRight,
                const ns_geo::Point2d &staLeft, const ns_geo::Point2d &staRight)
        : lasLeft(lasLeft),
          lasRight(lasRight),
          staLeft(staLeft),
          staRight(staRight) {}

    SectionPair() = default;
};

/**
 * @brief override operator '<<' for type 'SectionPair'
 */
static std::ostream &operator<<(std::ostream &os, const SectionPair &obj) {
    os << '{';
    os << "'lasLeft': " << obj.lasLeft << ", 'lasRight': " << obj.lasRight
       << ", 'staLeft': " << obj.staLeft << ", 'staRight': " << obj.staRight;
    os << '}';
    return os;
}

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

    void addDataSeries(Q3DScatter *graph, const ns_geo::PointSet3d &pts,
                       const QColor &color, float size);

    ns_geo::PointSet3d readPointsLaser(const std::string &filename);

    ns_geo::PointSet3d readPointsStation(const std::string &filename);

    void displayPtsInfo(QTableWidget *tab, const ns_geo::PointSet3d &pts);

    void connection();

    void init();

    void selectRadiusPoints(const pcl::PointXYZ &p);

  private:
    Ui::MainWindow *ui;
    Q3DScatter *graphLaser;
    Q3DScatter *graphStation;
    Q3DScatter *graphCoord;
    Q3DScatter *graphICP;
    ns_geo::PointSet3d laserPts;
    ns_geo::PointSet3d stationPts;
    ns_geo::PointSet3d newStationPts;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    ns_geo::PointSet3d selectedPts;
    int selectedMainPtIdx;
    int selectedLeftPtIdx;
    int selectedRightPtIdx;
    std::vector<SectionPair> pairs;
};

#endif // MAINWINDOW_H
