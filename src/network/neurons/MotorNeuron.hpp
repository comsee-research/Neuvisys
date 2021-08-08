//
// Created by alphat on 23/05/2021.
//

#ifndef NEUVISYS_DV_MOTORNEURON_HPP
#define NEUVISYS_DV_MOTORNEURON_HPP

#include "Neuron.hpp"

class MotorNeuron : public Neuron {
    double m_reward{};
    double m_bias{};
    int m_iter{};
    boost::circular_buffer<NeuronEvent> m_events;
    Eigen::Tensor<double, COMPLEXDIM> m_eligibilityTrace;
    Eigen::Tensor<double, COMPLEXDIM> &m_weights;
public:
    bool newEvent(NeuronEvent event, double reward) override;
    MotorNeuron(size_t index, NeuronConfig &conf, Position pos, Eigen::Tensor<double, 3> &weights);
    void spike(long time);
    double getWeights(long x, long y, long z) override;
    std::vector<long> getWeightsDimension() override;
    void saveWeights(std::string &saveFile) override;
    void loadWeights(std::string &filePath) override;
private:
    bool membraneUpdate(NeuronEvent event);
    void updateSTDP();
    void normalizeWeights();
};


#endif //NEUVISYS_DV_MOTORNEURON_HPP
