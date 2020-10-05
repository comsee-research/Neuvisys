#ifndef NEUVISYS_DV_SIMPLENEURON_HPP
#define NEUVISYS_DV_SIMPLENEURON_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include <xtensor/xarray.hpp>
#include <boost/circular_buffer.hpp>
#include "src/Config.hpp"
#include "Neuron.hpp"

struct CompareEventsTimestamp {
    bool operator()(Event const &event1, Event const &event2) {
        return event1.timestamp() > event2.timestamp();
    }
};

class SimpleNeuron : public Neuron {
    std::vector<long> m_delays;
    boost::circular_buffer<Event> m_events;
    std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp> m_waitingList;
public:
    SimpleNeuron(size_t index, NeuronConfig &conf, Luts &luts, Position pos, Position offset, xt::xarray<double> &weights, size_t nbSynapses);
    void newEvent(long timestamp, size_t x, size_t y, bool polarity) override;
    void update(long time) override;
    double getWeights(size_t p, size_t s, size_t x, size_t y) override;
private:
    void membraneUpdate(long timestamp, size_t x, size_t y, bool polarity, size_t synapse);
    void spike(long time);
    void updateSTDP();
    void normalizeWeights();
};

#endif //NEUVISYS_DV_SIMPLENEURON_HPP
