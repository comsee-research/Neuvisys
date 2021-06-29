#include "neuvisysgui.h"

#include <QApplication>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "listener");

    QApplication a(argc, argv);
    NeuvisysGUI w;
    w.setFixedSize(0.9 * QDesktopWidget().availableGeometry().size());
    w.show();
    return a.exec();
}
