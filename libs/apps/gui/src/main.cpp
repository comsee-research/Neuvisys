//
// Created by Thomas on 14/04/2021.
//

#include <gui/Neuvisysgui.h>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    NeuvisysGUI w(argc, argv,
                  "/home/comsee/Internship_Antony/neuvisys/Events/rotated_new_bars8/events/0.npz",
                  "/home/comsee/Internship_Antony/neuvisys/neuvisys-analysis/configuration/other_dataset_training/lateral_topdown/shared/vertical/");
    w.setFixedSize(0.8 * QDesktopWidget().availableGeometry().size());
    w.show();
    return app.exec();
}
