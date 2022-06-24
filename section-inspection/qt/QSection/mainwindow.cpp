#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "csv/csv.h"
#include "icp.h"
#include <QDebug>
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    this->init();
    this->connection();
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

void MainWindow::addDataSeries(Q3DScatter *graph, const ns_geo::PointSet3d &pts,
                               const QColor &color, float size) {
    QScatter3DSeries *series = new QScatter3DSeries(new QScatterDataProxy);
    series->setMeshSmooth(true);
    series->setBaseColor(color);
    series->setItemSize(size);
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
        this->addDataSeries(this->graphLaser, this->laserPts, QColor(0, 255, 0), 0.09f);
        ui->lineEdit_laser->setText(filename);
        this->displayPtsInfo(ui->tab_laser, this->laserPts);
        if (!this->stationPts.empty()) {
            this->addDataSeries(this->graphCoord, this->laserPts, QColor(0, 255, 0), 0.09f);
            this->addDataSeries(this->graphCoord, this->stationPts, QColor(255, 0, 0), 0.09f);
        }
        ui->tabWidget->setCurrentIndex(0);
    });
    connect(ui->btn_load_station, &QPushButton::clicked, this, [=]() {
        auto filename = QFileDialog::getOpenFileName(this, "Load Station Points", "../data");
        if (filename.isEmpty()) {
            return;
        }
        this->stationPts = this->readPointsStation(filename.toStdString());
        this->addDataSeries(this->graphStation, this->stationPts, QColor(255, 0, 0), 0.09f);
        this->displayPtsInfo(ui->tab_station_init, this->stationPts);
        ui->lineEdit_station->setText(filename);
        if (!this->laserPts.empty()) {
            this->addDataSeries(this->graphCoord, this->laserPts, QColor(0, 255, 0), 0.09f);
            this->addDataSeries(this->graphCoord, this->stationPts, QColor(255, 0, 0), 0.09f);
        }
        ui->tabWidget->setCurrentIndex(0);
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

        // kd tree
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->resize(newLaserPts.size());
        for (int i = 0; i != newLaserPts.size(); ++i) {
            (*cloud)[i] = pcl::PointXYZ(newLaserPts[i].x, newLaserPts[i].y, newLaserPts[i].z);
        }
        kdtree.setInputCloud(cloud);

        // visual
        this->newStationPts = ns_geo::PointSet3d(stationPts.size());
        for (int i = 0; i != stationPts.size(); ++i) {
            const auto &p1 = stationPts[i];
            auto p2 = Tls * Eigen::Vector3d(p1.x, p1.y, p1.z);
            newStationPts[i] = ns_geo::Point3d(p2(0), p2(1), p2(2));
        }
        this->addDataSeries(this->graphICP, this->laserPts, QColor(0, 255, 0), 0.09f);
        this->addDataSeries(this->graphICP, this->newStationPts, QColor(255, 0, 0), 0.1f);
        connect(this->graphICP->seriesList().at(1), &QScatter3DSeries::selectedItemChanged, this, [=](int index) {
            if (index == -1) {
                return;
            }
            auto p = this->newStationPts.at(index);
            this->selectedMainPtIdx = index;
            this->selectRadiusPoints(pcl::PointXYZ(p.x, p.y, p.z));
        });
        ui->tabWidget->setCurrentIndex(2);

        // display
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
    connect(ui->btn_fit, &QPushButton::clicked, this, [=]() {
        if (selectedPts.size() < 2) {
            return;
        }
        ns_geo::PointSet2d ptsToFit(selectedPts.size());
        for (int i = 0; i != selectedPts.size(); ++i) {
            ptsToFit[i].x = selectedPts[i].x;
            ptsToFit[i].y = selectedPts[i].z;
        }
        auto line = ns_section::SLine::fit(ptsToFit);
        auto np = line.nearest({newStationPts[selectedMainPtIdx].x, newStationPts[selectedMainPtIdx].z});
        this->addDataSeries(this->graphICP, {{np.x, 0.0, np.y}}, QColor(255, 255, 0), 0.11f);
        ui->lineEdit_x->setText(QString::number(np.x, 'f', 3));
        ui->lineEdit_z->setText(QString::number(np.y, 'f', 3));
    });
    connect(ui->btn_left, &QPushButton::clicked, this, [=]() {
        ui->left_x->setText(ui->lineEdit_x->text());
        ui->left_z->setText(ui->lineEdit_z->text());
        this->selectedLeftPtIdx = this->selectedMainPtIdx;
    });
    connect(ui->btn_right, &QPushButton::clicked, this, [=]() {
        ui->right_x->setText(ui->lineEdit_x->text());
        ui->right_z->setText(ui->lineEdit_z->text());
        this->selectedRightPtIdx = this->selectedMainPtIdx;
    });
    connect(ui->btn_make_pair, &QPushButton::clicked, this, [=]() {
        // check
        if (ui->left_x->text().isEmpty() || ui->left_z->text().isEmpty()) {
            return;
        }
        if (ui->right_x->text().isEmpty() || ui->right_z->text().isEmpty()) {
            return;
        }
        ui->tab_pair->insertRow(ui->tab_pair->rowCount());
        ui->tab_pair->insertRow(ui->tab_pair->rowCount());
        ui->tab_pair->insertRow(ui->tab_pair->rowCount());
        ui->tab_pair->insertRow(ui->tab_pair->rowCount());
        int rowIdx = ui->tab_pair->rowCount() - 4;
        int pairIdx = ui->tab_pair->rowCount() / 4;
        QTableWidgetItem *item;
        SectionPair p;
        {
            // station left
            auto psl = this->newStationPts[selectedLeftPtIdx];
            p.staLeft.x = psl.x;
            p.staLeft.y = psl.z;

            item = new QTableWidgetItem("STA_L_" + QString::number(pairIdx));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tab_pair->setItem(rowIdx, 0, item);

            item = new QTableWidgetItem(QString::number(psl.x, 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tab_pair->setItem(rowIdx, 1, item);

            item = new QTableWidgetItem(QString::number(psl.z, 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tab_pair->setItem(rowIdx, 2, item);
        }
        rowIdx += 1;
        {
            // laser left
            item = new QTableWidgetItem("LAS_L_" + QString::number(pairIdx));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tab_pair->setItem(rowIdx, 0, item);

            item = new QTableWidgetItem(ui->left_x->text());
            item->setTextAlignment(Qt::AlignCenter);
            ui->tab_pair->setItem(rowIdx, 1, item);

            item = new QTableWidgetItem(ui->left_z->text());
            item->setTextAlignment(Qt::AlignCenter);
            ui->tab_pair->setItem(rowIdx, 2, item);

            p.lasLeft.x = ui->left_x->text().toDouble();
            p.lasLeft.y = ui->left_z->text().toDouble();
        }
        rowIdx += 1;
        {
            // station right
            auto psr = this->newStationPts[selectedRightPtIdx];
            p.staRight.x = psr.x;
            p.staRight.y = psr.z;
            item = new QTableWidgetItem("STA_R_" + QString::number(pairIdx));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tab_pair->setItem(rowIdx, 0, item);

            item = new QTableWidgetItem(QString::number(psr.x, 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tab_pair->setItem(rowIdx, 1, item);

            item = new QTableWidgetItem(QString::number(psr.z, 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tab_pair->setItem(rowIdx, 2, item);
        }
        rowIdx += 1;
        {
            // laser right
            item = new QTableWidgetItem("LAS_R_" + QString::number(pairIdx));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tab_pair->setItem(rowIdx, 0, item);

            item = new QTableWidgetItem(ui->right_x->text());
            item->setTextAlignment(Qt::AlignCenter);
            ui->tab_pair->setItem(rowIdx, 1, item);

            item = new QTableWidgetItem(ui->right_z->text());
            item->setTextAlignment(Qt::AlignCenter);
            ui->tab_pair->setItem(rowIdx, 2, item);

            p.lasRight.x = ui->right_x->text().toDouble();
            p.lasRight.y = ui->right_z->text().toDouble();
        }
        pairs.push_back(p);
    });
    connect(ui->btn_compute, &QPushButton::clicked, this, [=]() {
        LOG_VAR(pairs);
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
    {
        this->graphICP = this->create3DScatterGraph();
        QWidget *container = QWidget::createWindowContainer(graphICP);
        ui->layout_3dgraph_icp->addWidget(container);
    }
    {
        ui->tab_pair->setColumnCount(3);
        ui->tab_pair->setHorizontalHeaderLabels({"Desc", "X(M)", "Z(M)"});
    }
}

void MainWindow::selectRadiusPoints(const pcl::PointXYZ &p) {
    double radius = ui->doubleSpinBox->value();
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    this->kdtree.radiusSearch(p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    while (this->graphICP->seriesList().size() > 2) {
        this->graphICP->removeSeries(
            this->graphICP->seriesList().at(
                this->graphICP->seriesList().size() - 1));
    }
    selectedPts.resize(pointIdxRadiusSearch.size());
    for (int i = 0; i != pointIdxRadiusSearch.size(); ++i) {
        selectedPts[i] = laserPts.at(pointIdxRadiusSearch[i]);
    }
    this->addDataSeries(this->graphICP, selectedPts, QColor(0, 0, 255), 0.1f);
    return;
}
