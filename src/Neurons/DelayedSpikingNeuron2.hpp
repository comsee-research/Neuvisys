#ifndef NEUVISYS_DV_DELAYEDSPIKINGNEURON2_HPP
#define NEUVISYS_DV_DELAYEDSPIKINGNEURON2_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include <xtensor/xarray.hpp>

#include "src/Event.hpp"
#include "src/Config.h"
#include "SpikingNeuron.hpp"

class DelayedSpikingNeuron2 : public SpikingNeuron {
protected:
    xt::xarray<long> m_delays;
    std::vector<std::vector<Event>> m_events;
    unsigned long m_updateCount;
public:
    DelayedSpikingNeuron2(int x, int y, xt::xarray<double> weights, xt::xarray<long> delays, double threshold);
    long getDelay(int x, int y);

    bool update(long time);
    double getPotential(long time) override;
    void newEvent(long timestamp, int x, int y, bool polarity) override;
    double potentialDecay(long time) override;
    bool spike() override;
};

#endif //NEUVISYS_DV_DELAYEDSPIKINGNEURON2_HPP
