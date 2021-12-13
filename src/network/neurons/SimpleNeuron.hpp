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
    boost::circular_buffer<NeuronEvent> m_inhibEvents;
    Eigen::Tensor<double, SIMPLEDIM> &m_weights;
    Eigen::Tensor<double, COMPLEXDIM> m_inhibWeights;
    std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp> m_waitingList;
public:
    SimpleNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset, Eigen::Tensor<double, SIMPLEDIM> &weights, size_t nbSynapses);
    bool newEvent(Event event) override;
    void newInhibitoryEvent(NeuronEvent event) override;
    bool update() override;
    double getWeights(long p, long c, long s, long x, long y) override { return m_weights(p, c, s, x, y); }
    double getInhibWeights(long x, long y, long z) override { return m_inhibWeights(x, y, z); }
    std::vector<long> getWeightsDimension() override;
    void saveWeights(std::string &saveFile) override;
    void loadWeights(std::string &filePath) override;
    bool checkRemainingEvents(long time) { return !m_waitingList.empty() && m_waitingList.top().timestamp() <= time; }
    void weightUpdate() override;
private:
    bool membraneUpdate(Event event);
    void spike(long time) override;
    void normalizeWeights();
};

#endif //NEUVISYS_DV_SIMPLENEURON_HPP
