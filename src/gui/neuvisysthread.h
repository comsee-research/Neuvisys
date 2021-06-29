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

class NeuvisysThread : public QThread
{
    Q_OBJECT

public:
    explicit NeuvisysThread(QObject *parent = nullptr);

    void render(QString networkPath, QString events, size_t nbPass, bool realtime);

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

signals:
    void displayInformation(int progress, double spike_rate, double threshold, double vreset, cv::Mat leftEventDisplay, cv::Mat rightEventDisplay, std::map<size_t, cv::Mat> weightDisplay, const std::vector<std::pair<double, long>> &potentialTrain, const std::map<size_t, std::vector<long>> &spikeTrain);
    void networkConfiguration(size_t nbCameras, size_t nbSynapses, std::string sharingType, size_t width, size_t height, size_t depth, size_t widthPatchSize, size_t heightPatchSize);

protected:
    QString m_networkPath;
    QString m_events;
    size_t m_nbPass;
    size_t m_iterations;
    cv::Mat m_leftEventDisplay;
    cv::Mat m_rightEventDisplay;
    std::map<size_t, cv::Mat> m_weightDisplay;
    std::map<size_t, std::vector<long>> m_spikeTrain;
    std::chrono::time_point<std::chrono::system_clock> m_frameTime;
    std::chrono::time_point<std::chrono::system_clock> m_trackingTime;
    bool m_realtime = false;

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

    void runSpikingNetwork(SpikingNetwork &spinet, const std::vector<Event> &eventPacket, double reward);
    void display(SpikingNetwork &spinet, size_t sizeArray);
};

#endif // NEUVISYSTHREAD_H
