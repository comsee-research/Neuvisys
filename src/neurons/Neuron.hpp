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
    virtual int getX();
    virtual int getY();
    virtual double getWeights(int p, int x, int y);
    virtual double getThreshold();
    virtual double getSpikingRate();
    virtual double getLearningDecay();
    virtual bool hasSpiked();
    virtual double getPotential(long time);
    virtual double getAdaptationPotential();
    virtual double potentialDecay(long time);
    virtual double refractoryPotential(long time);
    virtual double adaptationPotentialDecay(long time);
    virtual void inhibition();
    virtual void saveState(std::string &fileName);
    virtual void loadState(std::string &fileName);
    virtual void thresholdAdaptation();
    virtual void spikeRateAdaptation();
};

#endif //NEUVISYS_DV_NEURON_HPP
