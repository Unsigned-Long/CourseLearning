#include "uwbwin.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    UWBWin w;
    w.show();
    return a.exec();
}
