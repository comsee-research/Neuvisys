#ifndef NEUVISYS_DV_NEURON_HPP
#define NEUVISYS_DV_NEURON_HPP

#include "src/Config.hpp"

class Neuron {
protected:
    int m_x;
    int m_y;
    xt::xarray<double> m_weights;
    std::list<int> m_recentSpikes;
    int m_totalSpike;
    int m_countSpike;
    double m_potential;
    double m_threshold;
    long m_timestampLastEvent;
    bool m_spike;
    long m_creationTime;
    double m_spikingRate;
    long m_inhibitionTime;
public:
    Neuron(int x, int y, xt::xarray<double> weights, double threshold);
    virtual int getX();
    virtual int getY();
    virtual double getWeights(int p, int x, int y);
    virtual double getThreshold();
    virtual double getSpikingRate();
    virtual bool hasSpiked();
    virtual double getPotential(long time);
    virtual double potentialDecay(long time);
    virtual bool newEvent(long timestamp, int x, int y, bool polarity);
    virtual void spike();
    virtual void setInhibitionTime(long inhibitionTime);
    virtual void saveState(std::string &fileName);
    virtual void loadState(std::string &fileName);
    virtual void thresholdAdaptation();
};

#endif //NEUVISYS_DV_NEURON_HPP
