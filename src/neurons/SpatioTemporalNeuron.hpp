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
    std::vector<long> m_delays;
    std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp> m_waitingList;
public:
    SpatioTemporalNeuron(NeuronConfig &conf, Luts &luts, int x, int y, xt::xarray<double> &weights, std::vector<long> delays);
    void newEvent(long timestamp, int x, int y, bool polarity);
    bool update(long time);
    void membraneUpdate(long timestamp, int x, int y, bool polarity, int synapse);
    void spike(long time);
    void updateSTDP();
    void normalizeWeights();
    double getWeights(int p, int s, int x, int y);
private:
    using Neuron::getWeights;
};

#endif //NEUVISYS_DV_SPATIOTEMPORALNEURON_HPP
