#ifndef NEUVISYS_DV_DELAYEDSPIKINGNEURON2_HPP
#define NEUVISYS_DV_DELAYEDSPIKINGNEURON2_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include <xtensor/xarray.hpp>

#include "src/Event.hpp"
#include "src/Config.hpp"
#include "SpikingNeuron.hpp"

class DelayedSpikingNeuron2 : public SpikingNeuron {
protected:
    xt::xarray<long> m_delays;
    std::vector<std::vector<Event>> m_events;
    unsigned long m_updateCount;
    void resetListEvents();
public:
    DelayedSpikingNeuron2(int x, int y, xt::xarray<double> weights, xt::xarray<long> delays, double threshold);
    long getDelay(int x, int y);

    double getPotential(long time) override;
    bool newEvent(long timestamp, int x, int y, bool polarity) override;
    bool update(long time);
    double potentialDecay(long time) override;
    void spike() override;
};

#endif //NEUVISYS_DV_DELAYEDSPIKINGNEURON2_HPP
