#include "src/gui/neuvisysgui.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    NeuvisysGUI w;
    w.show();
    return a.exec();
}
