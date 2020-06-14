#ifndef NEUVISYS_DV_POOLINGNEURON_HPP
#define NEUVISYS_DV_POOLINGNEURON_HPP

#include <src/Event.hpp>
#include "Neuron.hpp"

class PoolingNeuron : public Neuron {
    std::vector<NeuronEvent> m_events;
public:
    PoolingNeuron(NeuronConfig &conf, Luts &luts, int x, int y, xt::xarray<double> &weights);
    void newEvent(long timestamp, int x, int y, int z) override;
private:
    void membraneUpdate(long timestamp, int x, int y, int layer);
    void spike(long time);
    void updateSTDP();
    void normalizeWeights();
};

#endif //NEUVISYS_DV_POOLINGNEURON_HPP
