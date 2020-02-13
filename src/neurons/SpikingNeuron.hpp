#ifndef NEUVISYS_DV_SPIKINGNEURON_HPP
#define NEUVISYS_DV_SPIKINGNEURON_HPP

#include "src/Config.hpp"

class SpikingNeuron {
protected:
    int m_x{}, m_y{};
    xt::xarray<double> m_weights;
    double m_potential{};
    double m_threshold{};
    long m_timestampLastEvent{};
    bool m_spike{};
    long m_inhibitionTime{};
public:
    SpikingNeuron() = default;
    virtual int getX();
    virtual int getY();
    virtual double getWeights(int p, int x, int y);
    virtual double getThreshold();
    virtual double getPotential();
    virtual bool hasSpiked();
    virtual double getPotential(long time);
    virtual double potentialDecay(long time);
    virtual bool newEvent(long timestamp, int x, int y, bool polarity);
    virtual void spike();
    virtual void setInhibitionTime(long inhibitionTime);
    virtual void saveWeights(std::string fileName);
};

#endif //NEUVISYS_DV_SPIKINGNEURON_HPP
