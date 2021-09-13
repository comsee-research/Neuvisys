#ifndef NEUVISYSTHREAD_H
#define NEUVISYSTHREAD_H

#include <QThread>
#include <chrono>
#include <random>
#include <utility>

#include "../network/NetworkHandle.hpp"
#include "../dependencies/json.hpp"
#include "../robot-control/SimulationInterface.hpp"
#include "cnpy.h"

class NeuvisysThread : public QThread {
    Q_OBJECT

public:
    NeuvisysThread(int argc, char** argv, QObject *parent = nullptr);
    void render(QString networkPath, QString events, size_t nbPass, bool realtime);
    bool init();

public slots:
    void onIndexChanged(size_t index);
    void onZcellChanged(size_t zcell);
    void onDepthChanged(size_t depth);
    void onCameraChanged(size_t camera);
    void onSynapseChanged(size_t synapse);
    void onPrecisionEventChanged(size_t displayRate);
    void onRangePotentialChanged(size_t rangePotential);
    void onPrecisionPotentialChanged(size_t trackRate);
    void onRangeSpikeTrainChanged(size_t rangeSpiketrain);
    void onLayerChanged(size_t layer);
    void onStopNetwork();

signals:
    void displayProgress(int progress, double simTime, double event_rate, double on_off_ratio, double spike_rate, double threshold, double bias);
    void displayEvents(const cv::Mat &leftEventDisplay, const cv::Mat& rightEventDisplay);
    void displayWeights(const std::map<size_t, cv::Mat>& weightDisplay, size_t layer);
    void displayPotential(double vreset, double threshold, const std::vector<std::pair<double, long>> &potentialTrain);
    void displaySpike(const std::map<size_t, std::vector<long>> &spikeTrain);
    void displayReward(const std::vector<double> &rewardTrain, const std::vector<double> &valueTrain, const std::vector<double> &valueDotTrain, const std::vector<double> &tdTrain);
    void displayAction(const std::vector<bool> &motorActivation);
    void networkConfiguration(const std::string &sharingType, const std::vector<std::vector<size_t>> &layerPatches, const std::vector<size_t> &layerSizes, const
    std::vector<size_t> &neuronSizes);
    void networkCreation(size_t nbCameras, size_t nbSynapses, const std::vector<size_t> &networkStructure);
    void networkDestruction();

protected:
    int m_initArgc;
    char** m_initArgv;
    QString m_networkPath;
    QString m_events;
    size_t m_nbPass;
    size_t m_iterations;
    cv::Mat m_leftEventDisplay;
    cv::Mat m_rightEventDisplay;
    std::map<size_t, cv::Mat> m_weightDisplay;
    std::map<size_t, std::vector<long>> m_spikeTrain;
    std::vector<bool> m_motorDisplay;
    double m_simTime;
    double m_eventRate;
    bool m_realtime = false;
    bool m_stop = false;
    bool m_change = false;
    size_t m_on_count = 0, m_off_count = 0;

    int m_actor = -1;
    double m_value = 0;

    size_t m_id = 0;
    size_t m_zcell = 0;
    size_t m_depth = 0;
    size_t m_camera = 0;
    size_t m_synapse = 0;
    size_t m_layer = 0;

    double m_displayRate = 30000; // µs
    double m_trackRate = 10000; // µs
    double m_actionRate = 100000; // µs
    size_t m_rangePotential = 10000; // µs
    size_t m_rangeSpiketrain = 1000000; // µs

    void run() override;

private:
    void multiplePass(SpikingNetwork &spinet);
    void rosPass(SpikingNetwork &spinet);
    void display(SpikingNetwork &spinet, size_t sizeArray);
    void addEventToDisplay(const Event &event);
    void prepareWeights(SpikingNetwork &spinet);
};

#endif // NEUVISYSTHREAD_H
