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
    void newEvent(long timestamp, int x, int y, int z) override;
    double getWeights(int x, int y, int z) override;
private:
    void membraneUpdate(long timestamp, int x, int y, int z);
    void spike(long time);
    void updateSTDP();
    void normalizeWeights();
};

#endif //NEUVISYS_DV_COMPLEXNEURON_HPP
