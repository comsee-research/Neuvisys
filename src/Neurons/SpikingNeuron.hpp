#ifndef NEUVISYS_DV_SPIKINGNEURON_HPP
#define NEUVISYS_DV_SPIKINGNEURON_HPP

#include "src/Config.h"

class SpikingNeuron {
protected:
    int m_x{}, m_y{};
    xt::xarray<double> m_weights;
    double m_potential{};
    double m_threshold{};
    long m_timestampLastEvent{};
public:
    SpikingNeuron() = default;
    virtual int getX();
    virtual int getY();
    virtual double getWeights(int p, int x, int y);
    virtual double getThreshold();
    virtual double getPotential();
    inline virtual double getPotential(long time);
    virtual double potentialDecay(long time);
    virtual void newEvent(long timestamp, int x, int y, bool polarity);
    virtual bool spike();
};

#endif //NEUVISYS_DV_SPIKINGNEURON_HPP
