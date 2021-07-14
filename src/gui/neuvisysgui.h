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
    NeuvisysGUI(int argc, char** argv, QWidget *parent = nullptr);
    ~NeuvisysGUI() override;

public slots:
    void onDisplayProgress(int progress, double spike_rate, double threshold);
    void onDisplayEvents(const cv::Mat &leftEventDisplay, const cv::Mat& rightEventDisplay);
    void onDisplayWeights(const std::map<size_t, cv::Mat>& weightDisplay);
    void onDisplayPotential(double vreset, double threshold, const std::vector<std::pair<double, long>> &potentialTrain);
    void onDisplaySpike(const std::map<size_t, std::vector<long>> &spikeTrain);
    void onDisplayReward(const std::vector<double> &rewardTrain);
    void onDisplayAction(const std::vector<bool> &motorActivation);
    void onNetworkConfiguration(const std::string& sharingType, size_t width, size_t height, size_t depth, size_t widthPatchSize, size_t
    heightPatchSize);
    void onNetworkCreation(size_t nbCameras, size_t nbSynapses, size_t nbSimpleNeurons, size_t nbComplexNeurons, size_t nbMotorNeurons);
    void onFinished();

signals:
    void indexChanged(size_t index);
    void layerChanged(size_t layer);
    void cameraChanged(size_t camera);
    void synapseChanged(size_t synapse);
    void precisionEventChanged(size_t precisionEvent);
    void rangePotentialChanged(size_t rangePotential);
    void precisionPotentialChanged(size_t precisionPotential);
    void rangeSpikeTrainChanged(size_t rangeSpiketrain);
    void cellTypeChanged(size_t cellType);
    void stopNetwork();

private slots:
    void on_button_event_file_clicked();
    void on_button_network_directory_clicked();
    void on_button_launch_network_clicked();
    void on_text_network_config_textChanged();
    void on_text_simple_cell_config_textChanged();
    void on_text_complex_cell_config_textChanged();
    void on_text_motor_cell_config_textChanged();
    void on_button_selection_clicked();
    void on_spin_layer_selection_valueChanged(int arg1);  
    void on_spin_camera_selection_valueChanged(int arg1);
    void on_spin_synapse_selection_valueChanged(int arg1);
    void on_slider_precision_event_sliderMoved(int position);
    void on_slider_range_potential_sliderMoved(int position);
    void on_slider_precision_potential_sliderMoved(int position);
    void on_slider_range_spiketrain_sliderMoved(int position);
    void on_radio_button_simple_cell_clicked();
    void on_radio_button_complex_cell_clicked();
    void on_radio_button_motor_cell_clicked();

    void on_button_stop_network_clicked();

protected:
    NeuvisysThread neuvisysThread;
    Ui::NeuvisysGUI *ui;
    QLineSeries *potentialSeries;
    QChart *potentialChart;
    QScatterSeries *spikeSeries;
    QChart *spikeChart;
    QLineSeries *rewardSeries;
    QChart *rewardChart;
    QGraphicsPixmapItem leftEvents;
    QGraphicsPixmapItem rightEvents;

    size_t id;
    size_t layer;
    size_t camera;
    size_t synapse;
    size_t cellType;
    size_t precisionEvent;
    size_t precisionPotential;
    size_t rangePotential;
    size_t rangeSpiketrain;

private:
    void openConfigFiles();
};
#endif // NEUVISYSGUI_H
