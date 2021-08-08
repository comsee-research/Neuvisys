#ifndef NEUVISYS_DV_COMPLEXNEURON_HPP
#define NEUVISYS_DV_COMPLEXNEURON_HPP

#include "Neuron.hpp"

class ComplexNeuron : public Neuron {
protected:
    boost::circular_buffer<NeuronEvent> m_events;
    Eigen::Tensor<double, COMPLEXDIM> &m_weights;
public:
    ComplexNeuron(size_t index, NeuronConfig &conf, Position pos, Position offset, Eigen::Tensor<double, COMPLEXDIM> &weights);
    bool newEvent(NeuronEvent event, double reward) override;
    double getWeights(long x, long y, long z) override;
    std::vector<long> getWeightsDimension() override;
    void saveWeights(std::string &saveFile) override;
    void loadWeights(std::string &filePath) override;
private:
    bool membraneUpdate(NeuronEvent event);
    void spike(long time);
    void updateSTDP();
    void normalizeWeights();
};

#endif //NEUVISYS_DV_COMPLEXNEURON_HPP
