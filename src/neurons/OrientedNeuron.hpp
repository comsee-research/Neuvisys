#ifndef NEUVISYS_DV_ORIENTEDNEURON_HPP
#define NEUVISYS_DV_ORIENTEDNEURON_HPP

#include <thread>
#include <mutex>
#include <vector>
#include <array>
#include <opencv2/opencv.hpp>
#include <xtensor/xarray.hpp>

#include "src/Event.hpp"
#include "src/Config.hpp"
#include "Neuron.hpp"

class OrientedNeuron : public Neuron {
protected:
    std::vector<Event> m_events;
    long m_spikingTime;
    long m_lastSpikingTime;
public:
    OrientedNeuron(int x, int y, xt::xarray<double> weights, double threshold);
    double getPotential(long time) override;
    void newEvent(long timestamp, int x, int y, bool polarity) override;
    bool internalUpdate(long timestamp, int x, int y, bool polarity);
    virtual void learnWeightsSTDP();
    virtual void spike(long time);
    virtual void normalize();
private:
    using Neuron::spike;
};

#endif //NEUVISYS_DV_ORIENTEDNEURON_HPP
