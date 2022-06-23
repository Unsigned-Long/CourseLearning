#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "csv/csv.h"
#include "icp.h"
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    this->connection();
    this->init();
}

MainWindow::~MainWindow() {
    delete ui;
}

Q3DScatter *MainWindow::create3DScatterGraph() {
    Q3DScatter *graph = new Q3DScatter();
    graph->activeTheme()->setType(Q3DTheme::ThemeQt);
    QFont font = graph->activeTheme()->font();
    font.setPointSize(14);
    font.setFamily("Consolas");
    graph->activeTheme()->setFont(font);
    graph->setShadowQuality(QAbstract3DGraph::ShadowQualitySoftLow);
    graph->scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetFront);
    graph->axisX()->setTitle("X");
    graph->axisY()->setTitle("Y");
    graph->axisZ()->setTitle("Z");
    return graph;
}

void MainWindow::addDataSeries(Q3DScatter *graph, const ns_geo::PointSet3d &pts, const QColor &color) {
    QScatter3DSeries *series = new QScatter3DSeries(new QScatterDataProxy);
    series->setMeshSmooth(true);
    series->setBaseColor(color);
    graph->addSeries(series);

    QScatterDataArray *dataArray = new QScatterDataArray;
    dataArray->resize(pts.size());
    QScatterDataItem *ptrToDataArray = &dataArray->first();
    for (int i = 0; i != pts.size(); ++i) {
        const auto &p = pts[i];
        ptrToDataArray->setPosition(QVector3D(p.x, p.y, p.z));
        ptrToDataArray++;
    }

    graph->seriesList().at(graph->seriesList().size() - 1)->dataProxy()->resetArray(dataArray);
}

ns_geo::PointSet3d MainWindow::readPointsLaser(const std::string &filename) {
    auto reader = ns_csv::CSVReader::create(filename);
    ns_geo::PointSet3d pts;
    double x, y, z;
    while (reader->readLine(' ', x, y, z)) {
        pts.push_back(ns_geo::Point3d(x, y, z));
    }
    return pts;
}

ns_geo::PointSet3d MainWindow::readPointsStation(const std::string &filename) {
    auto reader = ns_csv::CSVReader::create(filename);
    ns_geo::PointSet3d pts;
    double x, y, z;
    std::string id, code;
    while (reader->readLine(',', id, x, y, z, code)) {
        pts.push_back(ns_geo::Point3d(x, y, z));
    }
    return pts;
}

void MainWindow::displayPtsInfo(QTableWidget *tab, const ns_geo::PointSet3d &pts) {
    tab->setRowCount(pts.size());
    tab->setColumnCount(3);
    tab->setHorizontalHeaderLabels({"X(M)", "Y(M)", "Z(M)"});
    QTableWidgetItem *item;
    for (int i = 0; i != pts.size(); ++i) {
        item = new QTableWidgetItem(QString::number(pts[i].x, 'f', 3));
        item->setTextAlignment(Qt::AlignCenter);
        tab->setItem(i, 0, item);

        item = new QTableWidgetItem(QString::number(pts[i].y, 'f', 3));
        item->setTextAlignment(Qt::AlignCenter);
        tab->setItem(i, 1, item);

        item = new QTableWidgetItem(QString::number(pts[i].z, 'f', 3));
        item->setTextAlignment(Qt::AlignCenter);
        tab->setItem(i, 2, item);
    }
    return;
}

void MainWindow::connection() {
    connect(ui->btn_load_laser, &QPushButton::clicked, this, [=]() {
        auto filename = QFileDialog::getOpenFileName(this, "Load Laser Points", "../data");
        if (filename.isEmpty()) {
            return;
        }
        this->laserPts = this->readPointsLaser(filename.toStdString());
        this->addDataSeries(this->graphLaser, this->laserPts, QColor(0, 255, 0));
        ui->lineEdit_laser->setText(filename);
        this->displayPtsInfo(ui->tab_laser, this->laserPts);
        if (!this->stationPts.empty()) {
            this->addDataSeries(this->graphCoord, this->laserPts, QColor(0, 255, 0));
            this->addDataSeries(this->graphCoord, this->stationPts, QColor(255, 0, 0));
        }
    });
    connect(ui->btn_load_station, &QPushButton::clicked, this, [=]() {
        auto filename = QFileDialog::getOpenFileName(this, "Load Station Points", "../data");
        if (filename.isEmpty()) {
            return;
        }
        this->stationPts = this->readPointsStation(filename.toStdString());
        this->addDataSeries(this->graphStation, this->stationPts, QColor(255, 0, 0));
        this->displayPtsInfo(ui->tab_station_init, this->stationPts);
        ui->lineEdit_station->setText(filename);
        if (!this->laserPts.empty()) {
            this->addDataSeries(this->graphCoord, this->laserPts, QColor(0, 255, 0));
            this->addDataSeries(this->graphCoord, this->stationPts, QColor(255, 0, 0));
        }
    });
    connect(ui->btn_icp, &QPushButton::clicked, this, [=]() {
        // check
        if (this->laserPts.empty()) {
            return;
        }
        if (this->stationPts.empty()) {
            return;
        }

        // pre
        ns_geo::PointSet3d newLaserPts(this->laserPts.size());
        for (int i = 0; i != this->laserPts.size(); ++i) {
            newLaserPts[i].x = laserPts[i].x;
            newLaserPts[i].y = 0.0;
            newLaserPts[i].z = laserPts[i].z;
        }
        auto Tls = ns_section::ICP::solve(this->stationPts, newLaserPts);

        // visual
        this->newStationPts = ns_geo::PointSet3d(stationPts.size());
        for (int i = 0; i != stationPts.size(); ++i) {
            const auto &p1 = stationPts[i];
            auto p2 = Tls * Eigen::Vector3d(p1.x, p1.y, p1.z);
            newStationPts[i] = ns_geo::Point3d(p2(0), p2(1), p2(2));
        }
        this->graphCoord->removeSeries(this->graphCoord->seriesList().at(0));
        this->graphCoord->removeSeries(this->graphCoord->seriesList().at(0));
        this->addDataSeries(this->graphCoord, this->laserPts, QColor(0, 255, 0));
        this->addDataSeries(this->graphCoord, this->newStationPts, QColor(255, 0, 0));

        // display
        ui->label_icp->setText("After ICP");
        this->displayPtsInfo(ui->tab_station_reg, this->newStationPts);
        Eigen::Matrix4d trans = Tls.matrix();
        ui->tab_trans->setRowCount(4);
        ui->tab_trans->setColumnCount(4);
        QTableWidgetItem *item;
        for (int i = 0; i != 4; ++i) {
            for (int j = 0; j != 4; ++j) {
                item = new QTableWidgetItem(QString::number(trans(i, j), 'f', 4));
                item->setTextAlignment(Qt::AlignCenter);
                ui->tab_trans->setItem(i, j, item);
            }
        }
    });
}

void MainWindow::init() {
    {
        this->graphLaser = this->create3DScatterGraph();
        QWidget *container = QWidget::createWindowContainer(graphLaser);
        ui->layout_3dgraph_laser->addWidget(container);
    }
    {
        this->graphStation = this->create3DScatterGraph();
        QWidget *container = QWidget::createWindowContainer(graphStation);
        ui->layout_3dgraph_station->addWidget(container);
    }
    {
        this->graphCoord = this->create3DScatterGraph();
        QWidget *container = QWidget::createWindowContainer(graphCoord);
        ui->layout_3dgraph_same_coord->addWidget(container);
    }
}
