//
// Created by Thomas on 14/04/2021.
//

#include "Neuvisysgui.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    NeuvisysGUI w(argc, argv,
                  "/home/thomas/Videos/natural/shapes/shapes.h5",
                  "/home/thomas/Networks/network/");
    w.setFixedSize(0.8 * QDesktopWidget().availableGeometry().size());
    w.show();
    return app.exec();
}
