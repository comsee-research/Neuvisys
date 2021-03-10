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
    explicit NeuvisysGUI(QWidget *parent = nullptr);
    ~NeuvisysGUI() override;

public slots:
    void onDisplayInformation(int progress, double spike_rate, double threshold, double vreset, const cv::Mat &leftEventDisplay, const cv::Mat& rightEventDisplay, const std::map<size_t, cv::Mat>& weightDisplay, const std::vector<std::pair<double, long>> &potentialTrain, const std::map<size_t, std::vector<long>> &spikeTrain);
    void onNetworkConfiguration(size_t nbCameras, size_t nbSynapses, const std::string& sharingType, size_t width, size_t height, size_t depth, size_t widthPatchSize, size_t heightPatchSize);

signals:
    void guiInformation(size_t index, size_t layer, size_t camera, size_t synapse, size_t precisionEvent, size_t rangePotential, size_t precisionPotential, size_t rangeSpiketrain);

private slots:
    void on_button_event_file_clicked();
    void on_button_network_directory_clicked();
    void on_button_launch_network_clicked();
    void on_text_network_config_textChanged();
    void on_text_simple_cell_config_textChanged();
    void on_text_complex_cell_config_textChanged();
    void on_button_selection_clicked();
    void on_spin_layer_selection_valueChanged(int arg1);  
    void on_spin_camera_selection_valueChanged(int arg1);
    void on_spin_synapse_selection_valueChanged(int arg1);
    void on_slider_precision_event_sliderMoved(int position);
    void on_slider_range_potential_sliderMoved(int position);
    void on_slider_precision_potential_sliderMoved(int position);
    void on_slider_range_spiketrain_sliderMoved(int position);

protected:
    NeuvisysThread neuvisysThread;
    Ui::NeuvisysGUI *ui;
    QLineSeries *potentialSeries;
    QChart *potentialChart;
    QScatterSeries *spikeSeries;
    QChart *spikeChart;
    QGraphicsPixmapItem leftEvents;
    QGraphicsPixmapItem rightEvents;

    size_t idSimple;
    size_t idComplex;
    size_t layerSimple;
    size_t layerComplex;
    size_t camera;
    size_t synapse;

    size_t precisionEvent;
    size_t precisionPotential;
    size_t rangePotential;
    size_t rangeSpiketrain;

private:
    void openConfigFiles();
};
#endif // NEUVISYSGUI_H
