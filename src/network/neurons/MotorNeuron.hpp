//
// Created by alphat on 23/05/2021.
//

#ifndef NEUVISYS_DV_MOTORNEURON_HPP
#define NEUVISYS_DV_MOTORNEURON_HPP


#include "Neuron.hpp"

class MotorNeuron : public Neuron {


public:
    bool newEvent(NeuronEvent event) override;
    MotorNeuron(size_t index, NeuronConfig &conf, Position pos, Position offset);
    void spike(long time);
private:
    bool membraneUpdate(NeuronEvent event);
};


#endif //NEUVISYS_DV_MOTORNEURON_HPP
