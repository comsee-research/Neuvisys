#ifndef NEUVISYSTHREAD_H
#define NEUVISYSTHREAD_H

#include <QThread>
#include <chrono>
#include <random>
#include <utility>

#include "../network/NetworkHandle.hpp"
#include "../dependencies/json.hpp"
#include "../ros/SimulationInterface.hpp"
#include "cnpy.h"

class NeuvisysThread : public QThread {
    Q_OBJECT

public:
    NeuvisysThread(int argc, char** argv, QObject *parent = nullptr);
    void render(QString networkPath, QString events, size_t nbPass, bool realtime);
    bool init();

public slots:
    void onIndexChanged(size_t index);
    void onLayerChanged(size_t layer);
    void onCameraChanged(size_t camera);
    void onSynapseChanged(size_t synapse);
    void onPrecisionEventChanged(size_t precisionEvent);
    void onRangePotentialChanged(size_t rangePotential);
    void onPrecisionPotentialChanged(size_t precisionPotential);
    void onRangeSpikeTrainChanged(size_t rangeSpiketrain);
    void onCellTypeChanged(size_t cellType);
    void onStopNetwork();

signals:
    void displayProgress(int progress, double spike_rate, double threshold);
    void displayEvents(const cv::Mat &leftEventDisplay, const cv::Mat& rightEventDisplay);
    void displayWeights(const std::map<size_t, cv::Mat>& weightDisplay);
    void displayPotential(double vreset, double threshold, const std::vector<std::pair<double, long>> &potentialTrain);
    void displaySpike(const std::map<size_t, std::vector<long>> &spikeTrain);
    void displayReward(const std::vector<double> &rewardTrain);
    void displayAction(const std::vector<bool> &motorActivation);
    void networkConfiguration(std::string sharingType, size_t width, size_t height, size_t depth, size_t widthPatchSize, size_t heightPatchSize);
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
    std::chrono::time_point<std::chrono::system_clock> m_frameTime;
    std::chrono::time_point<std::chrono::system_clock> m_trackingTime;
    std::chrono::time_point<std::chrono::system_clock> m_motorTime;
    bool m_realtime = false;
    bool m_stop = false;

    size_t m_id = 0;
    size_t m_layer = 0;
    size_t m_camera = 0;
    size_t m_synapse = 0;
    size_t m_cellType = 0;

    size_t m_precisionEvent = 30000; // µs
    size_t m_rangePotential = 10000; // µs
    size_t m_precisionPotential = 10000; // µs
    size_t m_rangeSpiketrain = 1000000; // µs

    void run() override;

private:
    void multiplePass(SpikingNetwork &spinet);
    void rosPass(SpikingNetwork &spinet);
    void display(SpikingNetwork &spinet, size_t sizeArray);
    void addEventToDisplay(const Event &event);
};

#endif // NEUVISYSTHREAD_H
