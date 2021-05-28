#ifndef NEUVISYSTHREAD_H
#define NEUVISYSTHREAD_H

#include <QThread>
#include <chrono>
#include <random>
#include <utility>

#include "../network/SpikingNetwork.hpp"
#include "../dependencies/json.hpp"
#include "cnpy.h"

class NeuvisysThread : public QThread
{
    Q_OBJECT

public:
    explicit NeuvisysThread(QObject *parent = nullptr);

    void render(QString networkPath, QString events, size_t nbPass);

public slots:
    void onGuiInformation(size_t index, size_t layer, size_t camera, size_t synapse, size_t precisionEvent, size_t rangePotential, size_t precisionPotential, size_t rangeSpiketrain);

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

    size_t m_layerSimple = 0;
    size_t m_camera = 0;
    size_t m_layerComplex = 0;
    size_t m_synapse = 0;
    size_t m_idSimple = 0;
    size_t m_idComplex = 0;

    size_t m_precisionEvent = 30000; // µs
    size_t m_rangePotential = 10000; // µs
    size_t m_precisionPotential = 10000; // µs
    size_t m_rangeSpiketrain = 1000000; // µs

    void run() override;

private:
    void multiplePass();
    void main_loop(SpikingNetwork &spinet);
    void runSpikingNetwork(SpikingNetwork &spinet, Event &event, size_t sizeArray);
    inline void display(SpikingNetwork &spinet, size_t sizeArray);
};

#endif // NEUVISYSTHREAD_H
