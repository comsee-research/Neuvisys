#ifndef NEUVISYS_DV_ORIENTEDSPIKINGNEURON_HPP
#define NEUVISYS_DV_ORIENTEDSPIKINGNEURON_HPP

#include <thread>
#include <mutex>
#include <vector>
#include <array>
#include <opencv2/opencv.hpp>
#include <xtensor/xarray.hpp>

#include "src/Event.hpp"
#include "src/Config.h"
#include "SpikingNeuron.hpp"

class OrientedSpikingNeuron : public SpikingNeuron {
    std::vector<Event> m_events;
    int m_countNormalize;
public:
    OrientedSpikingNeuron(int x, int y, xt::xarray<double> weightsOn, xt::xarray<double> weightsOff, double threshold);
    double getPotential(long time) override;
    void newEvent(long timestamp, int x, int y, bool polarity) override;
    bool update(long timestamp, int x, int y, bool polarity);
    void learnWeightsSTDP();
    bool fire() override;
};

#endif //NEUVISYS_DV_ORIENTEDSPIKINGNEURON_HPP
