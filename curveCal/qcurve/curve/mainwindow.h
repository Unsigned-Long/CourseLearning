#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "handler.h"
#include <QLineEdit>
#include <QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

    void connection();
    void init();
    bool fvFromLineEdit(const QLineEdit* le, float& val);
    void setDetails(int row, const std::string& str1, const std::string& str2);
    std::string strFromPoint(const ns_geo::Point2f& pos);
    void drawerFig(const ns_cc::Handler& handler);
    void addSeries(const ns_geo::PointSet3f& ps, QLineSeries* series, const QString& name);
    void addScatter(const ns_geo::Point2f& pos, QScatterSeries* series, const QString& name);


private slots:
    void on_actionrun_triggered();

    void on_actionsave_triggered();

    void on_actionclear_triggered();

    void on_actionquit_triggered();

private:
    Ui::MainWindow* ui;
    float _start_K, _end_K, _stride;
    ns_cc::Handler _handler;
};
#endif // MAINWINDOW_H
