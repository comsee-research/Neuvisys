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
    std::vector<size_t> m_delays;
    boost::circular_buffer<Event> m_events;
    boost::circular_buffer<NeuronEvent> m_topDownInhibitionEvents;
    boost::circular_buffer<NeuronEvent> m_lateralInhibitionEvents;
    Eigen::Tensor<double, SIMPLEDIM> &m_weights;
    std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp> m_waitingList;
public:
    SimpleNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset,
                 Eigen::Tensor<double, SIMPLEDIM> &weights, size_t nbSynapses);

    bool newEvent(Event event) override;

    void newTopDownInhibitoryEvent(NeuronEvent event) override;

    void newLateralInhibitoryEvent(NeuronEvent event) override;

    bool update() override;

    double getWeights(long p, long c, long s, long x, long y) override { return m_weights(p, c, s, x, y); }

    std::vector<long> getWeightsDimension() override;

    void saveWeights(std::string &fileName) override;

    void saveInhibWeights(std::string &fileName) override;

    void loadWeights(std::string &fileName) override;

    void loadInhibWeights(std::string &fileName) override;

    bool checkRemainingEvents(size_t time) { return !m_waitingList.empty() && m_waitingList.top().timestamp() <= time; }

    void weightUpdate() override;

private:
    bool membraneUpdate(Event event);

    void spike(size_t time) override;

    void normalizeWeights() override;
};

#endif //NEUVISYS_DV_SIMPLENEURON_HPP
