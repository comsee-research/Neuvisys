#ifndef NEUVISYS_DV_NEURON_HPP
#define NEUVISYS_DV_NEURON_HPP

#include "src/Config.hpp"
#include "src/Utils.hpp"

class Neuron {
protected:
    NeuronConfig &conf;
    int m_x;
    int m_y;
    xt::xarray<double> &m_weights;
    std::list<int> m_recentSpikes;
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
public:
    Neuron(NeuronConfig &conf, Luts &luts, int x, int y, xt::xarray<double> &weights);
    virtual int getX() {return m_x;}
    virtual int getY() {return m_y;}
    virtual double getThreshold() {return m_threshold;}
    virtual double getSpikingRate() {return m_spikingRate;}
    virtual double getLearningDecay() {return m_learningDecay;}
    virtual long getSpikingTime() {return m_spikingTime;}
    virtual double getAdaptationPotential() {return m_adaptation_potential;}

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

    virtual void newEvent(long timestamp, int x, int y, bool polarity) {};
    virtual void newEvent(long timestamp, int x, int y, int z) {};
    virtual void update(long time) {};
    virtual double getWeights(int x, int y, int z) {};
    virtual double getWeights(int p, int s, int x, int y) {};
};

#endif //NEUVISYS_DV_NEURON_HPP
