#ifndef NEUVISYSGUI_H
#define NEUVISYSGUI_H

#include <QMainWindow>
#include <QFile>
#include <QFileDialog>
#include <QTextStream>
#include <QStandardItemModel>
#include <QMessageBox>
#include <QtCharts>

#include "neuvisysthread.h"

QT_BEGIN_NAMESPACE
namespace Ui { class NeuvisysGUI; }
QT_END_NAMESPACE

class NeuvisysGUI : public QMainWindow
{
    Q_OBJECT

public:
    NeuvisysGUI(QWidget *parent = nullptr);
    ~NeuvisysGUI();

public slots:
    void onDisplayInformation(const int progress, const double spike_rate, const cv::Mat eventDisplay, std::map<size_t, cv::Mat> weightDisplay, const std::vector<std::pair<double, long>> &potentialTrain, const std::map<size_t, std::vector<long>> &spikeTrain);
    void onNetworkConfiguration(const int nbCameras, const int nbSynapses, const std::string sharingType, const int width, const int height, const int depth);

signals:
    void guiInformation(const size_t index, const size_t layer, const size_t camera, const size_t synapse, const size_t framerate, const size_t trackingrate);

private slots:
    void on_button_event_file_1_clicked();
    void on_button_event_file_2_clicked();
    void on_button_network_directory_clicked();
    void on_button_launch_network_clicked();
    void on_text_network_config_textChanged();
    void on_text_simple_cell_config_textChanged();
    void on_text_complex_cell_config_textChanged();
    void on_button_selection_clicked();
    void on_spin_layer_selection_valueChanged(int arg1);  
    void on_spin_camera_selection_valueChanged(int arg1);
    void on_spin_synapse_selection_valueChanged(int arg1);
    void on_slider_frame_rate_sliderMoved(int position);
    void on_slider_tracking_rate_sliderMoved(int position);

protected:
    NeuvisysThread neuvisysThread;
    Ui::NeuvisysGUI *ui;
    QLineSeries *potentialSeries;
    QChart *potentialChart;
    QScatterSeries *spikeSeries;
    QChart *spikeChart;

    size_t index;
    size_t index2;
    size_t layer;
    size_t layer2;
    size_t camera;
    size_t synapse;
    size_t framerate;
    size_t trackingrate;

private:
    void openConfigFiles();
};
#endif // NEUVISYSGUI_H
