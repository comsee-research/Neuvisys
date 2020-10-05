#ifndef NEUVISYS_DV_COMPLEXNEURON_HPP
#define NEUVISYS_DV_COMPLEXNEURON_HPP

#include <boost/circular_buffer.hpp>
#include <src/Event.hpp>
#include "Neuron.hpp"

class ComplexNeuron : public Neuron {
protected:
    boost::circular_buffer<NeuronEvent> m_events;
public:
    ComplexNeuron(size_t index, NeuronConfig &conf, Luts &luts, Position pos, Position offset, xt::xarray<double> &weights);
    void newEvent(long timestamp, size_t x, size_t y, size_t z) override;
    double getWeights(size_t x, size_t y, size_t z) override;
private:
    void membraneUpdate(long timestamp, size_t x, size_t y, size_t z);
    void spike(long time);
    void updateSTDP();
    void normalizeWeights();
};

#endif //NEUVISYS_DV_COMPLEXNEURON_HPP
