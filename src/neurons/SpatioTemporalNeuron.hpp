#ifndef NEUVISYS_DV_SPATIOTEMPORALNEURON_HPP
#define NEUVISYS_DV_SPATIOTEMPORALNEURON_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include <xtensor/xarray.hpp>

#include "src/Event.hpp"
#include "OrientedNeuron.hpp"
#include "TemporalNeuron.hpp"

class SpatioTemporalNeuron : public OrientedNeuron {
protected:
    std::vector<long> m_delays;
    std::vector<std::vector<Event>> m_waitingList;
    unsigned int m_updateCount;
public:
    SpatioTemporalNeuron(int x, int y, xt::xarray<double> weights, std::vector<long> delays, double threshold);
    double getPotential(long time) override;
    bool newEvent(long timestamp, int x, int y, bool polarity) override;
//    bool update();
    void spike(long time) override;
    void learnWeightsSTDP() override;
    void normalize() override;
private:
    using OrientedNeuron::update;
};

#endif //NEUVISYS_DV_SPATIOTEMPORALNEURON_HPP
