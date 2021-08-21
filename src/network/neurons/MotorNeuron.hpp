//
// Created by alphat on 23/05/2021.
//

#ifndef NEUVISYS_DV_MOTORNEURON_HPP
#define NEUVISYS_DV_MOTORNEURON_HPP

#include "Neuron.hpp"

class MotorNeuron : public Neuron {
    double m_neuromodulator{};
    boost::circular_buffer<NeuronEvent> m_events;
    Eigen::Tensor<double, COMPLEXDIM> m_eligibilityTrace;
    Eigen::Tensor<double, COMPLEXDIM> &m_weights;

public:
    bool newEvent(NeuronEvent event) override;
    MotorNeuron(size_t index, NeuronConfig &conf, Position pos, Eigen::Tensor<double, 3> &weights);
    void spike(long time) override;
    double getWeights(long x, long y, long z) override;
    std::vector<long> getWeightsDimension() override;
    void saveWeights(std::string &saveFile) override;
    void loadWeights(std::string &filePath) override;
    void setNeuromodulator(double reward) override;
    void weightUpdate() override;
    std::pair<double, double> kernelSpikingRate() override;
private:
    bool membraneUpdate(NeuronEvent event);
    void normalizeWeights();
    static double kernel(long time);
    static double kernelDerivative(long time);
};


#endif //NEUVISYS_DV_MOTORNEURON_HPP
