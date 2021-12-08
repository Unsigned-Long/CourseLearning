#include "helper.h"
#include "ui_helper.h"

Helper::Helper(QWidget* parent)
    : QDialog(parent)
    , ui(new Ui::Helper)
{
    ui->setupUi(this);
    connect(ui->btn_ok, &QPushButton::clicked, [=]() {
        this->close();
    });
}

Helper::~Helper()
{
    delete ui;
}
