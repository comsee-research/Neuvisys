#ifndef NEUVISYS_DV_NEURON_HPP
#define NEUVISYS_DV_NEURON_HPP

#include "../Config.hpp"
#include "../Utils.hpp"
#include "../Event.hpp"
#include <boost/circular_buffer.hpp>
#include <cmath>
#include <list>
#include <iomanip>

class Neuron {
protected:
    size_t m_index;
    NeuronConfig &conf;
    Position m_pos{};
    Position m_offset{};
    std::list<size_t> m_recentSpikes;
    std::vector<std::reference_wrapper<Neuron>> m_outConnections;
    std::vector<std::reference_wrapper<Neuron>> m_inConnections;
    std::vector<std::reference_wrapper<Neuron>> m_inhibitionConnections;
    long m_spikingTime{};
    long m_lastSpikingTime{};
    size_t m_totalSpike{};
    size_t m_countSpike{};
    double m_learningDecay;
    double m_potential{};
    double m_adaptation_potential{};
    double m_threshold;
    long m_timestampLastEvent{};
    bool m_spike;
    long m_creationTime{};
    double m_spikingRate{};

    std::vector<double> m_trackingThresholds;
    std::vector<long> m_trackingSpikeTrain;
    std::vector<std::pair<double, long>> m_trackingPotentialTrain;
public:
    Neuron(size_t index, NeuronConfig &conf, Position pos, Position offset);
    virtual size_t getIndex() { return m_index; }
    virtual Position getPos() const { return m_pos; }
    virtual Position getOffset() { return m_offset; }
    virtual double getThreshold() { return m_threshold; }
    virtual double getSpikingRate() { return m_spikingRate; }
    virtual double getLearningDecay() { return m_learningDecay; }
    virtual long getSpikingTime() const { return m_spikingTime; }
    virtual double getAdaptationPotential() { return m_adaptation_potential; }
    virtual size_t getSpikeCount() { return m_countSpike; }
    virtual void resetSpikeCounter() { m_countSpike = 0; }
    virtual std::vector<std::reference_wrapper<Neuron>> getOutConnections() const { return m_outConnections; }
    virtual std::vector<std::reference_wrapper<Neuron>> getInConnections() const { return m_inConnections; }
    virtual std::vector<std::reference_wrapper<Neuron>> getInhibitionConnections() { return m_inhibitionConnections; }

    virtual const std::vector<double> &getTrackingThresholds() { return m_trackingThresholds; }
    virtual const std::vector<long> &getTrackingSpikeTrain() { return m_trackingSpikeTrain; }
    virtual const std::vector<std::pair<double, long>> &getTrackingPotentialTrain() { return m_trackingPotentialTrain; }

    virtual bool hasSpiked();
    virtual double getPotential(long time);
    virtual double potentialDecay(long time);
    virtual double refractoryPotential(long time);
    virtual double adaptationPotentialDecay(long time);
    virtual void inhibition();
    virtual void saveState(std::string &fileName);
    virtual void loadState(std::string &fileName);
    virtual void saveWeights(std::string &fileName) {};
    virtual void loadWeights(std::string &fileName) {};
    virtual void thresholdAdaptation();
    virtual void spikeRateAdaptation();

    virtual void addOutConnection(Neuron &neuron) { m_outConnections.emplace_back(neuron); }
    virtual void addInConnection(Neuron &neuron) { m_inConnections.emplace_back(neuron); }
    virtual void addInhibitionConnection(Neuron &neuron) { m_inhibitionConnections.emplace_back(neuron); }
    virtual bool newEvent(Event event) {}
    virtual bool newEvent(NeuronEvent event) {}
    virtual bool newEvent(NeuronEvent event, double reward) {}
    virtual bool update() {}
    virtual void trackPotential(long time);
};

#endif //NEUVISYS_DV_NEURON_HPP
