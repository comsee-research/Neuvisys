#ifndef NEUVISYS_DV_COMPLEXNEURON_HPP
#define NEUVISYS_DV_COMPLEXNEURON_HPP

#include <boost/circular_buffer.hpp>
#include <src/Event.hpp>
#include "Neuron.hpp"

class ComplexNeuron : public Neuron {
protected:
    boost::circular_buffer<NeuronEvent> m_events;
    Eigen::Tensor<double, COMPLEXDIM> &m_weights;
public:
    ComplexNeuron(size_t index, NeuronConfig &conf, Luts &luts, Position pos, Position offset, Eigen::Tensor<double, COMPLEXDIM> &weights);
    void newEvent(NeuronEvent event) override;
    double getWeights(long x, long y, long z);
    void saveWeights(std::string &saveFile);
    void loadWeights(std::string &filePath);
private:
    void membraneUpdate(NeuronEvent event);
    void spike(long time);
    void updateSTDP();
    void normalizeWeights();
};

#endif //NEUVISYS_DV_COMPLEXNEURON_HPP
