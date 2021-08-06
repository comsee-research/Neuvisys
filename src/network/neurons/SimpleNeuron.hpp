#ifndef NEUVISYS_DV_SIMPLENEURON_HPP
#define NEUVISYS_DV_SIMPLENEURON_HPP

#include <vector>
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
    Eigen::Tensor<double, SIMPLEDIM> &m_weights;
    std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp> m_waitingList;
public:
    SimpleNeuron(size_t index, NeuronConfig &conf, Position pos, Position offset, Eigen::Tensor<double, SIMPLEDIM> &weights, size_t nbSynapses);
    bool newEvent(Event event) override;
    bool update() override;
    double getWeights(long p, long c, long s, long x, long y);
    void saveWeights(std::string &saveFile) override;
    void loadWeights(std::string &filePath) override;
    bool checkRemainingEvents(long time) { return !m_waitingList.empty() && m_waitingList.top().timestamp() <= time; }
private:
    bool membraneUpdate(Event event);
    void spike(long time);
    void updateSTDP();
    void normalizeWeights();
};

#endif //NEUVISYS_DV_SIMPLENEURON_HPP
