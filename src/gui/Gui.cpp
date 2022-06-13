//
// Created by Thomas on 14/04/2021.
//

#include "Neuvisysgui.h"
#include <QApplication>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    NeuvisysGUI w(argc, argv,
                  "/home/thomas/Videos/natural/shapes.h5",
                  "/home/thomas/Networks/test/");
    w.setFixedSize(0.9 * QDesktopWidget().availableGeometry().size());
    w.show();
    return app.exec();
}
