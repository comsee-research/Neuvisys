//
// Created by Thomas on 14/04/2021.
//

#include "Neuvisysgui.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    NeuvisysGUI w(argc, argv,
                  "/home/thomas/Videos/natural/shapes/shapes_multiple_disparity.h5",
                  "/home/thomas/Networks/simulation/rl/orientation_task/simple_only/network_learning/");
    w.setFixedSize(0.9 * QDesktopWidget().availableGeometry().size());
    w.show();
    return app.exec();
}
