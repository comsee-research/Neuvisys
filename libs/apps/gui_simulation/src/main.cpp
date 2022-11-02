//
// Created by Thomas on 14/04/2021.
//

#include <gui_simulation/NeuvisysguiSimulation.h>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    NeuvisysGUISimulation w(argc, argv,
                  "",
                  "/home/thomas/Networks/simulation/rl_inhibition/tracking_task/joint/network_1/");
    WeightMatrix::setSeed(456412);
    w.setFixedSize(0.9 * QDesktopWidget().availableGeometry().size());
    w.show();
    return app.exec();
}
