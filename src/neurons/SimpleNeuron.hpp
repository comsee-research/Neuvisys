#ifndef NEUVISYS_DV_SIMPLENEURON_HPP
#define NEUVISYS_DV_SIMPLENEURON_HPP

#include <vector>
#include <boost/circular_buffer.hpp>
#include <queue>
#include "Neuron.hpp"

struct CompareEventsTimestamp {
    bool operator()(Event const &event1, Event const &event2) {
        return event1.timestamp() > event2.timestamp();
    }
};

class SimpleNeuron : public Neuron {
    std::vector<long> m_delays;
    boost::circular_buffer<Event> m_events;
    Eigen::Tensor<double, 4> &m_weights;
    std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp> m_waitingList;
public:
    SimpleNeuron(size_t index, NeuronConfig &conf, Luts &luts, Position pos, Position offset, Eigen::Tensor<double, 4> &weights, size_t nbSynapses);
    void newEvent(long timestamp, long x, long y, bool polarity) override;
    void update(long time) override;
    double getWeights(long p, long s, long x, long y);
    void saveWeights(std::string &saveFile);
    void loadWeights(std::string &filePath);
private:
    void membraneUpdate(long timestamp, long x, long y, bool polarity, long synapse);
    void spike(long time);
    void updateSTDP();
    void normalizeWeights();
};

#endif //NEUVISYS_DV_SIMPLENEURON_HPP
