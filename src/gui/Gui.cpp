//
// Created by Thomas on 14/04/2021.
//

#include "Neuvisysgui.h"
#include <QApplication>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    NeuvisysGUI w(argc, argv,
                  "/home/thomas/Videos/",
                  "/home/thomas/Networks/simulation/rl/orientation_task/horizontal/learn_value/");
    w.setFixedSize(0.9 * QDesktopWidget().availableGeometry().size());
    w.show();
    return app.exec();
}
