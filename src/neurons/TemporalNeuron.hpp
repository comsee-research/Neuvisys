#ifndef NEUVISYS_DV_TEMPORALNEURON_HPP
#define NEUVISYS_DV_TEMPORALNEURON_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include <xtensor/xarray.hpp>

#include "src/Event.hpp"
#include "Neuron.hpp"

struct CompareEventsTimestamp {
    bool operator()(Event const &event1, Event const &event2) {
        return event1.timestamp() > event2.timestamp();
    }
};

class TemporalNeuron : public Neuron {
protected:
    xt::xarray<long> m_delays;
    std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp> m_events;
public:
    TemporalNeuron(int x, int y, xt::xarray<double> weights, xt::xarray<long> delays, double threshold);
    long getDelay(int x, int y);
    long getTimestampNextEvent();

    double getPotential(long time) override;
    void newEvent(long timestamp, int x, int y, bool polarity) override;
    bool update(long time);
    void spike() override;
};

#endif //NEUVISYS_DV_TEMPORALNEURON_HPP
