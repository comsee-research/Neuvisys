#ifndef NEUVISYS_DV_SPATIOTEMPORALNEURON_HPP
#define NEUVISYS_DV_SPATIOTEMPORALNEURON_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include <xtensor/xarray.hpp>

#include "src/Event.hpp"
#include "OrientedNeuron.hpp"
#include "TemporalNeuron.hpp"

class SpatioTemporalNeuron : public OrientedNeuron {
protected:
    std::vector<long> m_delays;
    std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp> m_waitingList;
public:
    SpatioTemporalNeuron(int x, int y, xt::xarray<double> weights, std::vector<long> delays, double threshold);
    void newEvent(long timestamp, int x, int y, bool polarity) override;
    bool update(long time);
    void spike(long time) override;
    void learnWeightsSTDP() override;
    void normalize() override;
    double getWeights(int p, int s, int x, int y);
private:
    using Neuron::getWeights;
};

#endif //NEUVISYS_DV_SPATIOTEMPORALNEURON_HPP
