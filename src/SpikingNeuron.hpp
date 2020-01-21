#ifndef NEUVISYS_DV_SPIKING_NEURON_HPP
#define NEUVISYS_DV_SPIKING_NEURON_HPP

#endif //NEUVISYS_DV_SPIKING_NEURON_HPP

#include <vector>
#include <array>
#include <opencv2/opencv.hpp>
#include <xtensor/xarray.hpp>

#include "Event.hpp"
#include "Config.h"

struct CompareEventsTimestamp {
    bool operator()(Event const &event1, Event const &event2) {
        return event1.timestamp() > event2.timestamp();
    }
};

class SpikingNeuron {
    int m_x{}, m_y{};

    xt::xarray<double> m_weightsOn;
    xt::xarray<double> m_weightsOff;
    xt::xarray<long> m_delays;

    std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp> m_events;
    double m_potential{};
    double m_threshold{};
    long m_timestampLastEvent{};
public:
    SpikingNeuron();
    SpikingNeuron(int x, int y);
    SpikingNeuron(int x, int y, xt::xarray<double> weightsOn, xt::xarray<double> weightsOff, xt::xarray<long> delays, double threshold);

    int getX();
    int getY();
    double getThreshold();
    long getDelay(int x, int y);
    double getPotential();
    double getPotential(long time);
    bool isEmpty();
    long getTimestampNextEvent();

    bool update(long time);
    void newEvent(long timestamp, int x, int y, bool polarity);
    double potentialDecay(long time);
    bool fire();
};
