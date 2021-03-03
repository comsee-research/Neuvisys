#ifndef NEUVISYSTHREAD_H
#define NEUVISYSTHREAD_H

#include <QThread>
#include <chrono>
#include <random>
#include <utility>

#include "src/network/SpikingNetwork.hpp"
#include "src/dependencies/json.hpp"
#include "cnpy.h"

class NeuvisysThread : public QThread
{
    Q_OBJECT

public:
    NeuvisysThread(QObject *parent = nullptr);

    void render(QString networkPath, QString events, int nbPass);

public slots:
    void onGuiInformation(const size_t index, const size_t layer, const size_t camera, const size_t synapse, const size_t precisionEvent, const size_t rangePotential, const size_t precisionPotential, const size_t rangeSpiketrain, const size_t precisionSpiketrain);

signals:
    void displayInformation(const int progress, const double spike_rate, const double threshold, const cv::Mat leftEventDisplay, const cv::Mat rightEventDisplay, const std::map<size_t, cv::Mat> weightDisplay, const std::vector<std::pair<double, long>> &potentialTrain, const std::map<size_t, std::vector<long>> &spikeTrain);
    void networkConfiguration(const int nbCameras, const int nbSynapses, const std::string sharingType, const int width, const int height, const int depth, const int widthPatchSize, const int heightPatchSize);

protected:
    QString m_networkPath;
    QString m_events;
    int m_nbPass;
    long m_iterations;
    cv::Mat m_leftEventDisplay;
    cv::Mat m_rightEventDisplay;
    std::map<size_t, cv::Mat> m_weightDisplay;
    std::map<size_t, std::vector<long>> m_spikeTrain;
    std::chrono::time_point<std::chrono::system_clock> m_frameTime;
    std::chrono::time_point<std::chrono::system_clock> m_trackingTime;

    size_t m_layer = 0;
    size_t m_camera = 0;
    size_t m_layer2 = 0;
    size_t m_synapse = 0;
    size_t m_idSimple = 0;
    size_t m_idComplex = 0;

    size_t m_precisionEvent = 30000; // µs
    size_t m_rangePotential = 10000; // µs
    size_t m_precisionPotential = 10000; // µs
    size_t m_rangeSpiketrain = 1000000; // µs
    size_t m_precisionSpiketrain = 30000; // µs

    void run() override;

private:
    void multiplePass();
    void main_loop(SpikingNetwork &spinet);
    void runSpikingNetwork(SpikingNetwork &spinet, Event &event, size_t sizeArray);
    inline void display(SpikingNetwork &spinet, size_t sizeArray);
};

#endif // NEUVISYSTHREAD_H
