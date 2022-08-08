//
// Created by Thomas on 23/05/2021.
//

#ifndef NEUVISYS_DV_MOTORNEURON_HPP
#define NEUVISYS_DV_MOTORNEURON_HPP

#include "Neuron.hpp"

class MotorNeuron : public Neuron {
    double m_neuromodulator{};
    boost::circular_buffer<NeuronEvent> m_events;
    std::vector<WeightMap> m_multiWeights;
    std::vector<WeightMap> m_eligibilityTrace;
    std::vector<WeightMap> m_eligibilityTiming;

public:
    MotorNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, const std::vector<std::vector<size_t>> &dimensions);

    bool newEvent(NeuronEvent event) override;

    void spike(size_t time) override;

    void saveWeights(const std::string &filePath) override;

    void loadWeights(std::string &filePath) override;

    void loadWeights(cnpy::npz_t &arrayNPZ) override;

    void setNeuromodulator(double neuromodulator) override;

    void weightUpdate() override;

    cv::Mat summedWeightMatrix() override;

    double updateKernelSpikingRate(long time) override;

    WeightMap &getWeightsMap() override;

private:
    bool membraneUpdate(NeuronEvent event);

    double kernel(double time);

};


#endif //NEUVISYS_DV_MOTORNEURON_HPP
