#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "csv/csv.h"
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
    series->setItemLabelFormat(QStringLiteral("@xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
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

ns_geo::PointSet3d MainWindow::readXYZPts(const std::string &filename) {
    auto reader = ns_csv::CSVReader::create(filename);
    ns_geo::PointSet3d pts;
    double x, y, z;
    while (reader->readLine(' ', x, y, z)) {
        pts.push_back(ns_geo::Point3d(x, y, z));
    }
    return pts;
}

void MainWindow::connection() {
    connect(ui->btn_load_laser, &QPushButton::clicked, this, [=]() {
        auto filename = QFileDialog::getOpenFileName(this, "Load Laser Points", "../data");
        if (filename.isEmpty()) {
            return;
        }
        this->laserPts = this->readXYZPts(filename.toStdString());
        this->addDataSeries(this->graphInit, this->laserPts, QColor(255, 0, 0));
        ui->lineEdit_laser->setText(filename);
        // remove the y axis
        for (auto &p : this->laserPts) {
            p.y = 0.0;
        }
    });
    connect(ui->btn_load_station, &QPushButton::clicked, this, [=]() {
        auto filename = QFileDialog::getOpenFileName(this, "Load Station Points", "../data");
        if (filename.isEmpty()) {
            return;
        }
        this->stationPts = this->readXYZPts(filename.toStdString());
        this->addDataSeries(this->graphInit, this->stationPts, QColor(0, 255, 0));
        ui->lineEdit_station->setText(filename);
    });
}

void MainWindow::init() {
    {
        this->graphInit = this->create3DScatterGraph();
        QWidget *container = QWidget::createWindowContainer(graphInit);
        ui->layout_3dgraph_init->addWidget(container);
    }

    {
        this->graphICP = this->create3DScatterGraph();
        QWidget *container = QWidget::createWindowContainer(graphICP);
        ui->layout_3dgraph_icp->addWidget(container);
    }
}
