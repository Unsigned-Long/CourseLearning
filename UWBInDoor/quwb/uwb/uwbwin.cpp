#include "uwbwin.h"
#include "QDialog"
#include "QFileDialog"
#include "QMessageBox"
#include "helper.h"
#include "ui_uwbwin.h"

UWBWin::UWBWin(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::UWBWin)
{
    ui->setupUi(this);
    this->connection();
    ui->stackedWidget->setCurrentIndex(0);
}

UWBWin::~UWBWin()
{
    delete ui;
}

void UWBWin::connection()
{
    connect(ui->btn_start, &QPushButton::clicked, [=]() {
        this->init();
        ui->stackedWidget->setCurrentIndex(1);
    });
    connect(ui->btn_back, &QPushButton::clicked, [=]() {
        ui->stackedWidget->setCurrentIndex(0);
    });
    connect(ui->btn_exit, &QPushButton::clicked, [=]() {
        this->close();
    });
    connect(ui->btn_open_data, &QPushButton::clicked, [=]() {
        auto filename = QFileDialog::getOpenFileName(this, "UWB Data file", this->_lastFilePath);
        if (filename.isEmpty()) {
            QMessageBox::information(this, "Attention", "The file name is invalid");
            this->statusBar()->showMessage("file open failed");
            return;
        }
        this->_lastFilePath = filename;
        ui->lineEdit_data->setText(this->_lastFilePath);

        ns_uwb::UWBContext::data().clear();
        const_cast<std::map<int, ns_point::Point3d>&>(ns_uwb::UWBContext::baseStation()).clear();
        const_cast<std::string&>(ns_uwb::UWBContext::fileHeader()).clear();

        ns_uwb::UWBContext::UWBInit(this->_lastFilePath.toStdString(), "../../configs/UWBParams.json");

        this->setHeaderInfo();
        this->setStationInfo();
        this->setDataItemInfo();
        ui->tabWidget->setCurrentIndex(0);
        this->statusBar()->showMessage("file load finished");

        ui->spinBox_range->setMaximum(ns_uwb::UWBContext::data().size());
    });
    connect(ui->radioBtn_range, &QRadioButton::clicked, [=](bool checked) {
        ui->spinBox_range->setEnabled(checked);
        ui->spinBox_step->setEnabled(checked);
        this->statusBar()->showMessage("solve mode [range step]");
    });
    connect(ui->radioBtn_all, &QRadioButton::clicked, [=](bool checked) {
        ui->spinBox_range->setEnabled(!checked);
        ui->spinBox_step->setEnabled(!checked);
        this->statusBar()->showMessage("solve mode [all]");
    });
    connect(ui->btn_run, &QPushButton::clicked, [=]() {
        if (ns_uwb::UWBContext::data().empty()) {
            QMessageBox::information(this, "Attention", "Please load data first");
            return;
        }
        auto alg = ui->comboBox_algo->currentText();
        if (alg == "Ceres")
            this->_curAlg = ns_uwb::ceresPosition;
        else if (alg == "newtonLS")
            this->_curAlg = ns_uwb::newtonLS;
        else if (alg == "taylorSeries")
            this->_curAlg = ns_uwb::taylorSeries;
        else if (alg == "linear")
            this->_curAlg = ns_uwb::linear;
        else if (alg == "sequential")
            this->_curAlg = ns_uwb::sequential;
        else
            this->_curAlg = ns_uwb::ceresPosition;
        if (ui->radioBtn_all->isChecked()) {
            this->_calMode = CAL_MODE::CAL_ALL;
            this->_curStart = ns_uwb::UWBContext::data().cbegin();
            this->_curEnd = ns_uwb::UWBContext::data().cend();
        } else {
            this->_calMode = CAL_MODE::CALL_RABGE_STEP;
            ns_uwb::UWBContext::CALCUL_RANGE = ui->spinBox_range->value();
            ns_uwb::UWBContext::CALCUL_RANGE_STEP = ui->spinBox_step->value();
            this->_curStart = ns_uwb::UWBContext::data().cbegin();
            this->_curEnd = ns_uwb::UWBContext::data().cbegin() + ns_uwb::UWBContext::CALCUL_RANGE;
        }
        ui->tabWidget->setCurrentIndex(1);
        ui->checkBox_showSta->setEnabled(false);
        // for charts
        this->_chart.removeAllSeries();
        if (ui->radioBtn_line->isChecked())
            this->_pos_range = new QLineSeries;
        else {
            this->_pos_range = new QScatterSeries;
            dynamic_cast<QScatterSeries*>(this->_pos_range)->setMarkerSize(10);
        }
        this->_pos_range->setName("POS_RANGE");
        this->_pos_range->clear();

        if (ui->checkBox_showSta->isChecked()) {
            for (const auto& elem : ns_uwb::UWBContext::baseStation()) {
                auto s = new QScatterSeries;
                s->append(elem.second.y(), elem.second.x());
                s->setName("Sta" + QString::number(elem.first));
                s->setMarkerShape(QScatterSeries::MarkerShape::MarkerShapeRectangle);
                this->_chart.addSeries(s);
            }
        }
        if (this->_calMode == CAL_MODE::CALL_RABGE_STEP)
            this->_chart.addSeries(this->_pos_range);
        this->_chart.setTitle("Real time graph");
        this->_chart.setTitleFont(QFont("Ubuntu Mono", 15, 2));
        ui->graphicsView->setChart(&this->_chart);

        this->_timer.start(1);
    });

    connect(ui->btn_save, &QPushButton::clicked, [=]() {
        if (ns_uwb::UWBContext::data().empty() || this->_pos_range->points().empty()) {
            QMessageBox::information(this, "Attention", "There is nothing to save!");
            return;
        }
        auto filename = QFileDialog::getSaveFileName(this, "Save Result");
        if (filename.isEmpty())
            return;
        std::fstream file(filename.toStdString(), std::ios::out);
        for (auto& elem : this->_pos_range->points()) {
            file << elem.rx() << ',' << elem.ry() << '\n';
        }
        file.close();
        this->statusBar()->showMessage("save finished");
    });

    connect(&this->_timer, &QTimer::timeout, [=]() {
        this->run();
    });
    connect(ui->btn_stop, &QPushButton::clicked, [=]() {
        this->_timer.stop();
        ui->checkBox_showSta->setEnabled(true);
        this->_curStart = ns_uwb::UWBContext::data().cbegin();
        this->_curEnd = ns_uwb::UWBContext::data().cbegin() + ns_uwb::UWBContext::CALCUL_RANGE;
    });
    connect(ui->btn_help, &QPushButton::clicked, [=]() {
        Helper* helper = new Helper(this);
        helper->exec();
    });
    connect(ui->btn_clear, &QPushButton::clicked, [=]() {
        ns_uwb::UWBContext::data().clear();
        const_cast<std::map<int, ns_point::Point3d>&>(ns_uwb::UWBContext::baseStation()).clear();
        const_cast<std::string&>(ns_uwb::UWBContext::fileHeader()).clear();
        ui->lineEdit_data->clear();
        ui->table_dataitem->setRowCount(0);
        ui->table_dataitem->setColumnCount(0);
        ui->table_station->setRowCount(0);
        ui->table_station->setColumnCount(0);
        ui->textEdit_header->clear();
        ui->lineEdit_x->clear();
        ui->lineEdit_y->clear();
        this->setSolveState(QColor(0, 0, 0));
        ui->graphicsView->setChart(new QChart);
        ui->tabWidget->setCurrentIndex(0);
        this->statusBar()->showMessage("clear finished");
    });
    return;
}

void UWBWin::init()
{
    //    ui->stackedWidget->setCurrentIndex(0);
    ui->radioBtn_all->setChecked(true);
    ui->comboBox_algo->addItems({ "Ceres", "newtonLS", "taylorSeries", "linear", "sequential" });
    ui->spinBox_range->setEnabled(false);
    ui->spinBox_step->setEnabled(false);
    ui->spinBox_step->setValue(5);
    ui->spinBox_range->setValue(10);
    ui->radioBtn_scatter->setChecked(true);
    this->statusBar()->showMessage("init finished");
    return;
}

void UWBWin::setHeaderInfo()
{
    ui->textEdit_header->setText(QString::fromStdString(ns_uwb::UWBContext::fileHeader()));
    return;
}

void UWBWin::setStationInfo()
{
    auto& sta = ns_uwb::UWBContext::baseStation();
    ui->table_station->verticalHeader()->hide();
    ui->table_station->setColumnCount(4);
    ui->table_station->setHorizontalHeaderLabels({ "Sta ID", "X(m)", "Y(m)", "Z(m)" });
    ui->table_station->setRowCount(sta.size());
    QTableWidgetItem* tempItem;
    int count = 0;
    for (auto& elem : sta) {
        ui->table_station->setItem(count, 0, tempItem = new QTableWidgetItem(QString::number(elem.first)));
        tempItem->setTextAlignment(Qt::AlignmentFlag::AlignCenter);

        ui->table_station->setItem(count, 1, tempItem = new QTableWidgetItem(QString::number(elem.second.x(), 'f', 3)));
        tempItem->setTextAlignment(Qt::AlignmentFlag::AlignCenter);

        ui->table_station->setItem(count, 2, tempItem = new QTableWidgetItem(QString::number(elem.second.y(), 'f', 3)));
        tempItem->setTextAlignment(Qt::AlignmentFlag::AlignCenter);

        ui->table_station->setItem(count, 3, tempItem = new QTableWidgetItem(QString::number(elem.second.z(), 'f', 3)));
        tempItem->setTextAlignment(Qt::AlignmentFlag::AlignCenter);
        ++count;
    }
    return;
}

void UWBWin::setDataItemInfo()
{
    auto& data = ns_uwb::UWBContext::data();
    ui->table_dataitem->setColumnCount(3);
    ui->table_dataitem->setHorizontalHeaderLabels({ "Time", "Ref Sta ID", "Range(m)" });
    ui->table_dataitem->setRowCount(data.size());
    int count = 0;
    QTableWidgetItem* tempItem;
    for (auto& elem : data) {
        ui->table_dataitem->setItem(count, 0, tempItem = new QTableWidgetItem(QString::fromStdString(elem._time.stringExpr())));
        tempItem->setTextAlignment(Qt::AlignmentFlag::AlignHCenter);
        ui->table_dataitem->setItem(count, 1, tempItem = new QTableWidgetItem(QString::number(elem._refStationID)));
        tempItem->setTextAlignment(Qt::AlignmentFlag::AlignHCenter);
        ui->table_dataitem->setItem(count, 2, tempItem = new QTableWidgetItem(QString::number(elem._range, 'f', 3)));
        tempItem->setTextAlignment(Qt::AlignmentFlag::AlignHCenter);
        ++count;
    }
    return;
}

void UWBWin::setSolveState(const QColor& color)
{
    auto rgb = "rgb(" + QString::number(color.red()) + ", "
        + QString::number(color.green()) + ", "
        + QString::number(color.blue()) + ");";
    ui->label_state->setStyleSheet("background-color: " + rgb + "color: rgb(255, 255, 255);");
    return;
}

QString UWBWin::outputSolveState(std::vector<ns_uwb::UWBContext::DataItem>::const_iterator start,
    std::vector<ns_uwb::UWBContext::DataItem>::const_iterator end)
{
    return QString("data from '") + QString::fromStdString(start->_time.stringExpr()) + "' to '" + QString::fromStdString((end - 1)->_time.stringExpr()) + "' dur {" + QString::fromStdString(std::to_string(ns_dt::distance(start->_time, (end - 1)->_time, ns_dt::TimeUnit::SECOND))) + " s}";
}

void UWBWin::run()
{
    if (this->_curEnd > ns_uwb::UWBContext::data().cend()) {
        this->_timer.stop();
        this->statusBar()->showMessage("finished");
        ui->checkBox_showSta->setEnabled(true);
        return;
    }
    // output to state bar
    this->statusBar()->showMessage(this->outputSolveState(this->_curStart, this->_curEnd));
    if (this->_calMode == CAL_MODE::CAL_ALL) {
        try {
            auto pos = this->_curAlg(ns_uwb::UWBContext::data().cbegin(), ns_uwb::UWBContext::data().cend());
            // set solve state
            this->setSolveState(QColor(40, 160, 40));

            ui->lineEdit_x->setText(QString::number(pos.x(), 'f', 3));
            ui->lineEdit_y->setText(QString::number(pos.y(), 'f', 3));

            // set qcharts

            auto pos_all = new QScatterSeries;
            pos_all->append(pos.y(), pos.x());
            pos_all->setName("POS_ALL");
            pos_all->setMarkerSize(10);

            this->_chart.addSeries(pos_all);

            this->_chart.createDefaultAxes();

        } catch (const std::runtime_error& e) {
            this->statusBar()->showMessage(QString::fromStdString(e.what()));
            this->setSolveState(QColor(190, 30, 30));
        }
        this->_timer.stop();
        ui->checkBox_showSta->setEnabled(true);
    } else {
        try {
            auto pos = this->_curAlg(this->_curStart, this->_curEnd);
            // set pos
            ui->lineEdit_x->setText(QString::number(pos.x(), 'f', 3));
            ui->lineEdit_y->setText(QString::number(pos.y(), 'f', 3));
            // set solve state
            this->setSolveState(QColor(40, 160, 40));
            // change range
            auto tempIter = this->_curStart;
            this->_curEnd += ns_uwb::UWBContext::CALCUL_RANGE_STEP;
            this->_curStart = this->_curEnd - ns_uwb::UWBContext::CALCUL_RANGE;
            // delay time
            auto delay = ns_dt::distance(tempIter->_time, this->_curStart->_time, ns_dt::TimeUnit::MSECOND);
            this->_timer.setInterval(delay);

            // set qcharts
            this->_pos_range->append(pos.y(), pos.x());
            this->_chart.removeSeries(this->_pos_range);
            this->_chart.addSeries(this->_pos_range);
            this->_chart.createDefaultAxes();

        } catch (const std::runtime_error& e) {
            ns_uwb::output(std::string("error happend : ") + e.what(), true, true, '+');
            auto tempIter = this->_curEnd;
            ++this->_curEnd;
            // set solve state
            this->setSolveState(QColor(190, 30, 30));
            // delay time
            auto delay = ns_dt::distance(tempIter->_time, this->_curEnd->_time, ns_dt::TimeUnit::MSECOND);
            this->_timer.setInterval(delay);
        }
    }

    return;
}

void UWBWin::on_actionopen_triggered()
{
    ui->btn_open_data->click();
}

void UWBWin::on_actionexit_triggered()
{
    ui->btn_exit->click();
}

void UWBWin::on_actionline_triggered()
{
    ui->radioBtn_line->setChecked(true);
}

void UWBWin::on_actionscatter_triggered()
{
    ui->radioBtn_scatter->setChecked(true);
}

void UWBWin::on_actionceres_triggered()
{
    ui->comboBox_algo->setCurrentIndex(0);
}

void UWBWin::on_actionnewtonLs_triggered()
{
    ui->comboBox_algo->setCurrentIndex(1);
}

void UWBWin::on_actiontaylorSeries_triggered()
{
    ui->comboBox_algo->setCurrentIndex(2);
}

void UWBWin::on_actionlinear_triggered()
{
    ui->comboBox_algo->setCurrentIndex(3);
}

void UWBWin::on_actionsequential_triggered()
{
    ui->comboBox_algo->setCurrentIndex(4);
}

void UWBWin::on_actionall_triggered()
{
    ui->radioBtn_all->setChecked(true);
}

void UWBWin::on_actionrange_step_triggered()
{
    ui->radioBtn_range->setChecked(true);
}

void UWBWin::on_actionrun_triggered()
{
    ui->btn_run->click();
}

void UWBWin::on_actionstop_triggered()
{
    ui->btn_stop->click();
}

void UWBWin::on_actionclear_triggered()
{
    ui->btn_clear->click();
}

void UWBWin::on_actionback_triggered()
{
    ui->btn_back->click();
}

void UWBWin::on_actionsave_triggered()
{
    ui->btn_save->click();
}

void UWBWin::on_actionshow_station_triggered(bool checked)
{
    ui->checkBox_showSta->setChecked(checked);
}
