#ifndef NEUVISYS_DV_SPATIOTEMPORALNEURON_HPP
#define NEUVISYS_DV_SPATIOTEMPORALNEURON_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include <xtensor/xarray.hpp>

#include "src/Event.hpp"
#include "src/Config.hpp"
#include "Neuron.hpp"

struct CompareEventsTimestamp {
    bool operator()(Event const &event1, Event const &event2) {
        return event1.timestamp() > event2.timestamp();
    }
};

class SpatioTemporalNeuron : public Neuron {
protected:
    std::vector<Event> m_events;
    long m_spikingTime;
    long m_lastSpikingTime;
    std::vector<long> m_delays;
    std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp> m_waitingList;
public:
    SpatioTemporalNeuron(int x, int y, xt::xarray<double> weights, std::vector<long> delays, double threshold);
    void newEvent(long timestamp, int x, int y, bool polarity) override;
    bool update(long time);
    void membraneUpdate(long timestamp, int x, int y, bool polarity, int synapse);
    void spike(long time);
    void learnWeightsSTDP();
    void normalizeWeights();
    double getWeights(int p, int s, int x, int y);
private:
    using Neuron::getWeights;
    using Neuron::spike;
};

#endif //NEUVISYS_DV_SPATIOTEMPORALNEURON_HPP
