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
    Eigen::Tensor<double, COMPLEXDIM> m_eligibilityTiming;
    Eigen::Tensor<double, COMPLEXDIM> &m_weights;

public:
    MotorNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Eigen::Tensor<double, 3> &weights);

    bool newEvent(NeuronEvent event) override;

    void spike(size_t time) override;

    double getWeights(long x, long y, long z) override;

    std::vector<long> getWeightsDimension() override;

    void saveWeights(std::string &filePath) override;

    void loadWeights(cnpy::npz_t &arrayNPZ) override;

    void normalizeWeights() override;

    void setNeuromodulator(double neuromodulator) override;

    void weightUpdate() override;

    cv::Mat summedWeightMatrix() override;

    double updateKernelSpikingRate(long time) override;

    double computeNormWeights() override;

    void rescaleWeights(double scale) override;

    void learningDecay(double decay) override;

private:
    bool membraneUpdate(NeuronEvent event);

    double kernel(double time);

    double kernelDerivative(double time);
};


#endif //NEUVISYS_DV_MOTORNEURON_HPP
