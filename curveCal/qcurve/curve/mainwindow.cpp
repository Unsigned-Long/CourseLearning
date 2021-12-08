#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("Curve Calculator");
    init();
    connection();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::connection()
{
    connect(ui->btn_quit, &QPushButton::clicked, [=]() {
        this->close();
    });
    connect(ui->radioButton_rp, &QRadioButton::clicked, [=](bool check) {
        if (check)
            ui->tabWidget_mode->setCurrentIndex(1);
    });
    connect(ui->radioButton_sp, &QRadioButton::clicked, [=](bool check) {
        if (check)
            ui->tabWidget_mode->setCurrentIndex(0);
    });
    connect(ui->btn_save, &QPushButton::clicked, [=]() {
        if (ui->tableWidget_calRes->rowCount() == 0)
            QMessageBox::information(this, "ATTENTION", "There is nothing needs to be saved!");
        auto filename = QFileDialog::getSaveFileName(this, "SAVE");
        if (filename.isEmpty())
            return;
        std::fstream file(filename.toStdString(), std::ios::out);
        _handler.outputReport(_start_K, _end_K, _stride, file);
        QMessageBox::information(this, "ATTENTION", "Save Report Successfully!");
        return;
    });
    connect(ui->btn_clear, &QPushButton::clicked, [=]() {
        ui->lineEdit_K_JD->clear();
        ui->lineEdit_Ls->clear();
        ui->lineEdit_R->clear();
        ui->lineEdit_X_JD->clear();
        ui->lineEdit_Y_JD->clear();
        ui->lineEdit_corDeg_deg->clear();
        ui->lineEdit_corDeg_min->clear();
        ui->lineEdit_corDeg_sed->clear();
        ui->lineEdit_rp_ek->clear();
        ui->lineEdit_rp_sk->clear();
        ui->lineEdit_rp_stride->clear();
        ui->lineEdit_sp_k->clear();
        ui->lineEdit_sp_x->clear();
        ui->lineEdit_sp_y->clear();
        ui->lineEdit_tanDeg_deg->clear();
        ui->lineEdit_tanDeg_min->clear();
        ui->lineEdit_tanDeg_sed->clear();

        ui->tableWidget_detail->setRowCount(0);
        ui->tableWidget_detail->setColumnCount(0);
        ui->tableWidget_calRes->setRowCount(0);
        ui->tableWidget_calRes->setColumnCount(0);

        ui->graphicsView->setChart(new QChart());
    });
    connect(ui->btn_run, &QPushButton::clicked, [=]() {
        float K_JD, radius, ls, cornerDeg, cornerMin, cornerSed, tangentDeg, tangentMin, tangentSed;
        ns_geo::Point2f P_JD;
        if (!fvFromLineEdit(ui->lineEdit_K_JD, K_JD))
            return;
        if (!fvFromLineEdit(ui->lineEdit_R, radius))
            return;
        if (!fvFromLineEdit(ui->lineEdit_Ls, ls))
            return;
        if (!fvFromLineEdit(ui->lineEdit_corDeg_deg, cornerDeg))
            return;
        if (!fvFromLineEdit(ui->lineEdit_corDeg_min, cornerMin))
            return;
        if (!fvFromLineEdit(ui->lineEdit_corDeg_sed, cornerSed))
            return;
        if (!fvFromLineEdit(ui->lineEdit_tanDeg_deg, tangentDeg))
            return;
        if (!fvFromLineEdit(ui->lineEdit_tanDeg_min, tangentMin))
            return;
        if (!fvFromLineEdit(ui->lineEdit_tanDeg_sed, tangentSed))
            return;
        if (!fvFromLineEdit(ui->lineEdit_X_JD, P_JD.x()))
            return;
        if (!fvFromLineEdit(ui->lineEdit_Y_JD, P_JD.y()))
            return;
        auto corDir = ui->comboBox_Dir->currentText() == "LEFT" ? ns_cc::CornerDir::LEFT : ns_cc::CornerDir::RIGHT;
        ns_cc::Handler handler(K_JD, P_JD, ns_angle::Degree(cornerDeg + cornerMin / 60.0 + cornerSed / 3600.0),
            corDir, radius, ls, ns_angle::Degree(tangentDeg + tangentMin / 60.0 + tangentSed / 3600.0));
        _handler = handler;
        ui->tableWidget_detail->setRowCount(32);
        ui->tableWidget_detail->setColumnCount(2);
        ui->tableWidget_detail->setHorizontalHeaderLabels({ "Desc", "Value" });
        setDetails(0, "K_JD", std::to_string(handler._kjd));
        setDetails(1, "P_JD", strFromPoint(handler._pjd));
        setDetails(2, "CorA", ns_angle::toAngleStrExp(handler._corRad));
        setDetails(3, "CorD", ui->comboBox_Dir->currentText().toStdString());
        setDetails(4, "CurR", std::to_string(handler._r));
        setDetails(5, "C_Ls", std::to_string(handler._ls));
        setDetails(6, "TanA", ns_angle::toAngleStrExp(handler._tanRad));

        setDetails(7, "m", std::to_string(handler._m));
        setDetails(8, "p", std::to_string(handler._P));
        setDetails(9, "beta_0", ns_angle::toAngleStrExp(handler._beta_0));
        setDetails(10, "T_H", std::to_string(handler._T_H));
        setDetails(11, "L_H", std::to_string(handler._L_H));
        setDetails(12, "L_T", std::to_string(handler._L_T));
        setDetails(13, "E_H", std::to_string(handler._E_H));
        setDetails(14, "q", std::to_string(handler._q));
        setDetails(15, "K_ZH", std::to_string(handler._K_ZH));
        setDetails(16, "K_HY", std::to_string(handler._K_HY));
        setDetails(17, "K_QZ", std::to_string(handler._K_QZ));
        setDetails(18, "K_YH", std::to_string(handler._K_YH));
        setDetails(19, "K_HZ", std::to_string(handler._K_HZ));
        setDetails(20, "alpha ZH-JD", ns_angle::toAngleStrExp(handler._alpha_ZH_JD));
        setDetails(21, "alpha JD-ZH", ns_angle::toAngleStrExp(handler._alpha_JD_ZH));
        setDetails(22, "alpha HZ-JD", ns_angle::toAngleStrExp(handler._alpha_HZ_JD));
        setDetails(23, "alpha JD-HZ", ns_angle::toAngleStrExp(handler._alpha_JD_HZ));

        setDetails(24, "K_JD", std::to_string(handler._kjd));
        setDetails(25, "K_QZ + 0.5 * q", std::to_string(handler._K_QZ + 0.5 * handler._q));

        setDetails(26, "P_JD", strFromPoint(handler._pjd));
        setDetails(27, "P_ZH", strFromPoint(handler.P_ZH()));
        setDetails(28, "P_HY", strFromPoint(handler.P_HY()));
        setDetails(29, "P_QZ", strFromPoint(handler.P_QZ()));
        setDetails(30, "P_YH", strFromPoint(handler.P_YH()));
        setDetails(31, "P_HZ", strFromPoint(handler.P_HZ()));

        this->drawerFig(handler);

        if (ui->radioButton_sp->isChecked()) {
            float K;
            if (!fvFromLineEdit(ui->lineEdit_sp_k, K))
                return;
            auto pos = handler.calculate(K, false);
            ui->lineEdit_sp_x->setText(QString::number(pos.x(), 'f', 3));
            ui->lineEdit_sp_y->setText(QString::number(pos.y(), 'f', 3));
        } else {
            // write the range solve mode
            float start_K, end_K, stride;
            if (!this->fvFromLineEdit(ui->lineEdit_rp_sk, start_K))
                return;
            if (!this->fvFromLineEdit(ui->lineEdit_rp_ek, end_K))
                return;
            if (!this->fvFromLineEdit(ui->lineEdit_rp_stride, stride))
                return;
            _start_K = start_K, _end_K = end_K, _stride = stride;
            auto ps = handler.calculate(start_K, end_K, stride);
            ui->tableWidget_calRes->setRowCount(ps.size());
            ui->tableWidget_calRes->setColumnCount(4);
            ui->tableWidget_calRes->setHorizontalHeaderLabels({ "ID", "Mileage", "X", "Y" });
            ui->tableWidget_calRes->verticalHeader()->hide();
            for (std::size_t i = 0; i != ps.size(); ++i) {
                QTableWidgetItem* tempItem;
                ui->tableWidget_calRes->setItem(i, 0, tempItem = new QTableWidgetItem(QString::number(i)));
                tempItem->setTextAlignment(Qt::AlignCenter);
                ui->tableWidget_calRes->setItem(i, 1, tempItem = new QTableWidgetItem(QString::number(ps[i].z(), 'f', 3)));
                tempItem->setTextAlignment(Qt::AlignCenter);
                ui->tableWidget_calRes->setItem(i, 2, tempItem = new QTableWidgetItem(QString::number(ps[i].x(), 'f', 3)));
                tempItem->setTextAlignment(Qt::AlignCenter);
                ui->tableWidget_calRes->setItem(i, 3, tempItem = new QTableWidgetItem(QString::number(ps[i].y(), 'f', 3)));
                tempItem->setTextAlignment(Qt::AlignCenter);
            }
        }
        return;
    });
}

void MainWindow::init()
{
    ui->comboBox_Dir->addItems({ "LEFT", "RIGHT" });
    ui->comboBox_Dir->setCurrentIndex(1);
    ui->radioButton_sp->click();
    ui->tabWidget_mode->setCurrentIndex(0);
    return;
}

bool MainWindow::fvFromLineEdit(const QLineEdit* le, float& val)
{
    if (le->text().isEmpty()) {
        QMessageBox::information(this, "ATTENTION", "Some Necessary Value Is Empty.");
        return false;
    }
    val = std::stof(le->text().toStdString());
    return true;
}

void MainWindow::setDetails(int row, const std::string& str1, const std::string& str2)
{
    QTableWidgetItem* tempItem;
    ui->tableWidget_detail->setItem(row, 0, tempItem = new QTableWidgetItem(QString::fromStdString(str1)));
    tempItem->setTextAlignment(Qt::AlignCenter);
    ui->tableWidget_detail->setItem(row, 1, tempItem = new QTableWidgetItem(QString::fromStdString(str2)));
    tempItem->setTextAlignment(Qt::AlignCenter);
    return;
}

std::string MainWindow::strFromPoint(const ns_geo::Point2f& pos)
{
    return '[' + std::to_string(pos.x()) + ", " + std::to_string(pos.y()) + ']';
}

void MainWindow::drawerFig(const ns_cc::Handler& handler)
{
    QLineSeries* series1 = new QLineSeries();
    QLineSeries* series2 = new QLineSeries();
    QLineSeries* series3 = new QLineSeries();
    QLineSeries* series4 = new QLineSeries();
    QLineSeries* series5 = new QLineSeries();
    float stride = 5.0;
    this->addSeries(handler.calculate(handler.K_ZH() - 500.0, handler.K_ZH(), stride),
        series1, "[**]->[ZH]");
    this->addSeries(handler.calculate(handler.K_ZH(), handler.K_HY(), stride),
        series2, "[ZH]->[HY]");
    this->addSeries(handler.calculate(handler.K_HY(), handler.K_YH(), stride),
        series3, "[HY]->[YH]");
    this->addSeries(handler.calculate(handler.K_YH(), handler.K_HZ(), stride),
        series4, "[YH]->[HZ]");
    this->addSeries(handler.calculate(handler.K_HZ(), handler.K_HZ() + 500.0, stride),
        series5, "[HZ]->[**]");
    //
    QScatterSeries* ss1 = new QScatterSeries();
    QScatterSeries* ss2 = new QScatterSeries();
    QScatterSeries* ss3 = new QScatterSeries();
    QScatterSeries* ss4 = new QScatterSeries();
    QScatterSeries* ss5 = new QScatterSeries();

    this->addScatter(handler.P_ZH(), ss1, "P_ZH");
    this->addScatter(handler.P_HY(), ss2, "P_HY");
    this->addScatter(handler.P_QZ(), ss3, "P_QZ");
    this->addScatter(handler.P_YH(), ss4, "P_YH");
    this->addScatter(handler.P_HZ(), ss5, "P_HZ");
    //
    QChart* chart = new QChart;

    chart->addSeries(series1);
    chart->addSeries(series2);
    chart->addSeries(series3);
    chart->addSeries(series4);
    chart->addSeries(series5);

    chart->addSeries(ss1);
    chart->addSeries(ss2);
    chart->addSeries(ss3);
    chart->addSeries(ss4);
    chart->addSeries(ss5);

    chart->createDefaultAxes();
    chart->setAnimationOptions(QChart::AnimationOption::AllAnimations);

    ui->graphicsView->setChart(chart);
    return;
}

void MainWindow::addSeries(const ns_geo::PointSet3f& ps, QLineSeries* series, const QString& name)
{
    for (const auto& elem : ps)
        series->append(elem.y(), elem.x());
    series->setName(name);
}

void MainWindow::addScatter(const ns_geo::Point2f& pos, QScatterSeries* series, const QString& name)
{
    series->append(pos.y(), pos.x());
    series->setName(name);
    return;
}

void MainWindow::on_actionrun_triggered()
{
    ui->btn_run->click();
}


void MainWindow::on_actionsave_triggered()
{
    ui->btn_save->click();
}


void MainWindow::on_actionclear_triggered()
{
    ui->btn_clear->click();
}


void MainWindow::on_actionquit_triggered()
{
    ui->btn_quit->click();
}

