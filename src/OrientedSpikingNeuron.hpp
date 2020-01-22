#ifndef NEUVISYS_DV_ORIENTEDSPIKINGNEURON_HPP
#define NEUVISYS_DV_ORIENTEDSPIKINGNEURON_HPP

#endif //NEUVISYS_DV_ORIENTEDSPIKINGNEURON_HPP

#include <vector>
#include <array>
#include <opencv2/opencv.hpp>
#include <xtensor/xarray.hpp>

#include "Config.h"

class OrientedSpikingNeuron {
protected:
    int m_x{}, m_y{};
    xt::xarray<double> m_weightsOn;
    xt::xarray<double> m_weightsOff;
    double m_potential{};
    double m_threshold{};
    long m_timestampLastEvent{};
public:
    OrientedSpikingNeuron();
    OrientedSpikingNeuron(int x, int y);
    OrientedSpikingNeuron(int x, int y, xt::xarray<double> weightsOn, xt::xarray<double> weightsOff, double threshold);

    int getX();
    int getY();
    double getThreshold();
    double getPotential();
    double potentialDecay(long time);

    double getPotential(long time);
    void newEvent(long timestamp, int x, int y, bool polarity);
    bool fire();
};
