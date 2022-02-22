#ifndef NEUVISYS_DV_COMPLEXNEURON_HPP
#define NEUVISYS_DV_COMPLEXNEURON_HPP

#include "Neuron.hpp"

class ComplexNeuron : public Neuron {
protected:
    boost::circular_buffer<NeuronEvent> m_events;
    Eigen::Tensor<double, COMPLEXDIM> &m_weights;
public:
    ComplexNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset, Eigen::Tensor<double, COMPLEXDIM> &weights);
    bool newEvent(NeuronEvent event) override;
    double getWeights(long x, long y, long z) override { return m_weights(x, y, z); }
    std::vector<long> getWeightsDimension() override;
    void saveWeights(std::string &filePath) override;
    void loadWeights(std::string &filePath) override;
    void weightUpdate() override;
    cv::Mat summedWeightMatrix() override;
private:
    bool membraneUpdate(NeuronEvent event);
    void spike(size_t time) override;
    void normalizeWeights() override;
};

#endif //NEUVISYS_DV_COMPLEXNEURON_HPP
