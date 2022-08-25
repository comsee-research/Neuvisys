//
// Created by Thomas on 14/04/2021.
//

#ifndef NEUVISYS_DV_SIMPLENEURON_HPP
#define NEUVISYS_DV_SIMPLENEURON_HPP

#include "Neuron.hpp"

struct CompareEventsTimestamp {
    bool operator()(Event const &event1, Event const &event2) {
        return event1.timestamp() > event2.timestamp();
    }
};

class SimpleNeuron : public Neuron {
protected:
    std::vector<size_t> m_delays;
    boost::circular_buffer<Event> m_events;
    boost::circular_buffer<NeuronEvent> m_topDownInhibitionEvents;
    boost::circular_buffer<NeuronEvent> m_lateralInhibitionEvents;
    std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp> m_waitingList;
    WeightMatrix &m_sharedWeights;

public:
    SimpleNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset, WeightMatrix &weights, size_t nbSynapses);

    bool newEvent(Event event) override;

    void newTopDownInhibitoryEvent(NeuronEvent event) override;

    void newLateralInhibitoryEvent(NeuronEvent event) override;

    bool update() override;

    WeightMatrix &getWeightsMatrix() override;

    double getWeightsMatrixNorm() override;

    std::vector<size_t> getWeightsDimension() override;

    void saveWeights(const std::string &filePath) override;

    void saveLateralInhibitionWeights(std::string &filePath) override;

    void saveTopDownInhibitionWeights(std::string &filePath) override;

    void loadWeights(cnpy::npz_t &arrayNPZ) override;

    void loadWeights(std::string &filePath) override;

    void loadLateralInhibitionWeights(cnpy::npz_t &arrayNPZ) override;

    void loadLateralInhibitionWeights(std::string &filePath) override;

    void loadTopDownInhibitionWeights(cnpy::npz_t &arrayNPZ) override;

    void loadTopDownInhibitionWeights(std::string &filePath) override;

    bool checkRemainingEvents(size_t time) { return !m_waitingList.empty() && m_waitingList.top().timestamp() <= time; }

    void weightUpdate() override;

private:
    bool membraneUpdate(Event event);

    void spike(size_t time) override;

};

#endif //NEUVISYS_DV_SIMPLENEURON_HPP
