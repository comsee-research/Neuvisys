//
// Created by Thomas on 14/04/2021.
//

#ifndef NEUVISYS_DV_COMPLEXNEURON_HPP
#define NEUVISYS_DV_COMPLEXNEURON_HPP

#include "Neuron.hpp"

class ComplexNeuron : public Neuron {
protected:
    boost::circular_buffer<NeuronEvent> m_events;

public:
    ComplexNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset, const std::vector<size_t> &dimensions);

    bool newEvent(NeuronEvent event) override;

    void saveWeights(const std::string &filePath) override;

    void loadWeights(std::string &filePath) override;

    void loadWeights(cnpy::npz_t &arrayNPZ) override;

    void weightUpdate() override;

    cv::Mat summedWeightMatrix() override;

private:
    bool membraneUpdate(NeuronEvent event);

    void spike(size_t time) override;
};

#endif //NEUVISYS_DV_COMPLEXNEURON_HPP
