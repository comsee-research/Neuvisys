#ifndef NEUVISYS_DV_NEURON_HPP
#define NEUVISYS_DV_NEURON_HPP

#include "src/Config.hpp"
#include "src/Utils.hpp"
#include <src/Event.hpp>

class Neuron {
protected:
    size_t m_index;
    NeuronConfig &conf;
    Position m_pos{};
    Position m_offset{};
    xt::xarray<double> &m_weights;
    std::list<int> m_recentSpikes;
    std::vector<std::reference_wrapper<Neuron>> m_outConnections;
    std::vector<std::reference_wrapper<Neuron>> m_inConnections;
    long m_spikingTime{};
    long m_lastSpikingTime{};
    int m_totalSpike{};
    int m_countSpike{};
    double m_learningDecay;
    double m_potential{};
    double m_adaptation_potential{};
    double m_threshold;
    long m_timestampLastEvent{};
    bool m_spike;
    long m_creationTime{};
    double m_spikingRate{};

    Luts &m_luts;

    // Tracking Variables
    std::vector<long> m_spikeTrain;
    std::vector<std::pair<double, long>> m_potentialTrain;
public:
    Neuron(size_t index, NeuronConfig &conf, Luts &luts, Position pos, Position offset, xt::xarray<double> &weights);
    virtual size_t getIndex() { return m_index; }
    virtual Position getPos() { return m_pos; }
    virtual Position getOffset() { return m_offset; }
    virtual double getThreshold() { return m_threshold; }
    virtual double getSpikingRate() { return m_spikingRate; }
    virtual double getLearningDecay() { return m_learningDecay; }
    virtual long getSpikingTime() { return m_spikingTime; }
    virtual double getAdaptationPotential() { return m_adaptation_potential; }
    virtual std::vector<std::reference_wrapper<Neuron>> getOutConnections() { return m_outConnections; }
    virtual std::vector<std::reference_wrapper<Neuron>> getInConnections() { return m_inConnections; }

    virtual bool hasSpiked();
    virtual double getPotential(long time);
    virtual double potentialDecay(long time);
    virtual double refractoryPotential(long time);
    virtual double adaptationPotentialDecay(long time);
    virtual void inhibition();
    virtual void saveState(std::string &fileName);
    virtual void loadState(std::string &fileName);
    virtual void thresholdAdaptation();
    virtual void spikeRateAdaptation();

    virtual void addOutConnection(Neuron &neuron) { m_outConnections.emplace_back(neuron); }
    virtual void addInConnection(Neuron &neuron) { m_inConnections.emplace_back(neuron); }
    virtual void newEvent(long timestamp, int x, int y, bool polarity) {};
    virtual void newEvent(long timestamp, int x, int y, int z) {};
    virtual void update(long time) {};
    virtual double getWeights(int x, int y, int z) {};
    virtual double getWeights(int p, int s, int x, int y) {};
    virtual void track(long time);
};

#endif //NEUVISYS_DV_NEURON_HPP
