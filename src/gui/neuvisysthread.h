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

    void render(QString networkPath, QString leftEvents, QString rightEvents, int nbPass);

public slots:
    void onGuiInformation(const size_t index, const size_t layer, const size_t camera, const size_t synapse, const size_t framerate, const size_t trackingrate);

signals:
    void displayInformation(const int progress, const double spike_rate, const cv::Mat eventDisplay, const std::map<size_t, cv::Mat> weightDisplay, const std::vector<std::pair<double, long>> &potentialTrain, const std::map<size_t, std::vector<long>> &spikeTrain);
    void networkConfiguration(const int nbCameras, const int nbSynapses, const std::string sharingType, const int width, const int height, const int depth);

protected:
    QString m_networkPath;
    QString m_leftEvents;
    QString m_rightEvents;
    int m_nbPass;
    long m_iterations;
    cv::Mat m_eventDisplay;
    std::map<size_t, cv::Mat> m_weightDisplay;
    std::map<size_t, std::vector<long>> m_spikeTrain;
    std::chrono::time_point<std::chrono::system_clock> m_frameTime;
    std::chrono::time_point<std::chrono::system_clock> m_trackingTime;

    size_t m_layer = 0;
    size_t m_camera = 0;
    size_t m_layer2 = 0;
    size_t m_synapse = 0;
    size_t m_index = 0;
    size_t m_index2 = 0;
    size_t m_framerate = 30000; // µs
    size_t m_trackingrate = 10000; // µs`

    void run() override;

private:
    void multiplePass();
    void main_loop(SpikingNetwork &spinet);
    void stereo_loop(SpikingNetwork &spinet);
    void runSpikingNetwork(SpikingNetwork &spinet, Event &event, size_t sizeArray);
    inline void display(SpikingNetwork &spinet, size_t sizeArray);
};

#endif // NEUVISYSTHREAD_H
