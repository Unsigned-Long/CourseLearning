#ifndef HELPER_H
#define HELPER_H

#include <QDialog>

namespace Ui {
class Helper;
}

class Helper : public QDialog
{
    Q_OBJECT

public:
    explicit Helper(QWidget *parent = nullptr);
    ~Helper();

private:
    Ui::Helper *ui;
};

#endif // HELPER_H
