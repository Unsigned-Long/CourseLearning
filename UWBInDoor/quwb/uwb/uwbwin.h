#ifndef UWBWIN_H
#define UWBWIN_H

#include "QtCharts/QChart"
#include "QtCharts/QChartView"
#include "QtCharts/QScatterSeries"
#include "ceresSolver.h"
#include "linear.h"
#include "newtonLS.h"
#include "sequential.h"
#include "taylorSeries.h"
#include <QMainWindow>
#include <QTimer>
#include "QLineSeries"

QT_BEGIN_NAMESPACE
namespace Ui {
class UWBWin;
}
QT_END_NAMESPACE

class UWBWin : public QMainWindow {
    Q_OBJECT

public:
    using algorithm = ns_point::Point3d (*)(std::vector<ns_uwb::UWBContext::DataItem>::const_iterator start,
        std::vector<ns_uwb::UWBContext::DataItem>::const_iterator end);
    enum class CAL_MODE {
        CAL_ALL,
        CALL_RABGE_STEP
    };

public:
    UWBWin(QWidget* parent = nullptr);
    ~UWBWin();

    void connection();
    void init();
    void setHeaderInfo();
    void setStationInfo();
    void setDataItemInfo();
    void setSolveState(const QColor& color);
    QString outputSolveState(std::vector<ns_uwb::UWBContext::DataItem>::const_iterator start,
        std::vector<ns_uwb::UWBContext::DataItem>::const_iterator end);

    void run();

private slots:
    void on_actionopen_triggered();

    void on_actionexit_triggered();

    void on_actionline_triggered();

    void on_actionscatter_triggered();

    void on_actionceres_triggered();

    void on_actionnewtonLs_triggered();

    void on_actiontaylorSeries_triggered();

    void on_actionlinear_triggered();

    void on_actionsequential_triggered();

    void on_actionall_triggered();

    void on_actionrange_step_triggered();

    void on_actionrun_triggered();

    void on_actionstop_triggered();

    void on_actionclear_triggered();

    void on_actionback_triggered();

    void on_actionsave_triggered();

    void on_actionshow_station_triggered(bool checked);

private:
    Ui::UWBWin* ui;
    QString _lastFilePath;
    QTimer _timer;
    algorithm _curAlg;
    CAL_MODE _calMode;
    std::vector<ns_uwb::UWBContext::DataItem>::const_iterator _curStart;
    std::vector<ns_uwb::UWBContext::DataItem>::const_iterator _curEnd;
    QChart _chart;
    QXYSeries *_pos_range;
};
#endif // UWBWIN_H
