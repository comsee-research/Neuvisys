#include "SpikingNetwork.hpp"

SpikingNetwork::SpikingNetwork() {
    m_retina = std::vector<std::vector<size_t>>(WIDTH*HEIGHT, std::vector<size_t>(0));
    generateNeuronConfiguration();
    assignNeurons();
    m_firings = std::vector<bool>(m_neurons.size());
    std::cout << "Number of neurons: " << m_neurons.size() << std::endl;
}

void SpikingNetwork::addEvent(const long timestamp, const int x, const int y, const bool polarity) {
/*    for (size_t ind : m_retina[x*HEIGHT+y]) {
        m_neurons[ind].newEvent(timestamp, x - m_neurons[ind].getX(), y - m_neurons[ind].getY(), polarity);
    }*/
    for (size_t ind : m_retina[x*HEIGHT+y]) {
        m_neurons[ind].update(timestamp, x - m_neurons[ind].getX(), y - m_neurons[ind].getY(), polarity);
    }
}

/*void SpikingNetwork::updateNeurons(long time) {
    size_t count = 0;
    for (auto &neuron : m_neurons) {
        if (neuron.update(time)) {
            m_firings[count] = true;
        }
        ++count;
    }
}*/

void SpikingNetwork::updateDisplay(long time, std::vector<cv::Mat> &displays) {
    size_t count = 0;
    double potential;
    int norm_potential;
    for (auto &neuron : m_neurons) {
        potential = neuron.getPotential(time);
        if (potential > neuron.getThreshold()) {
            norm_potential = 255;
        } else {
            norm_potential = static_cast<int>((potential / neuron.getThreshold()) * 255);
        }
        //displays[count % NUMBER_DISPLAY](cv::Rect(neuron.getX(), neuron.getY(), NEURON_WIDTH, NEURON_HEIGHT)) = norm_potential;
        displays[0](cv::Rect(neuron.getX(), neuron.getY(), NEURON_WIDTH, NEURON_HEIGHT)) = norm_potential;

        for (int i = 0; i < NEURON_HEIGHT; ++i) {
            for (int j = 0; j < NEURON_WIDTH; ++j) {
                displays[1](i, j) = neuron.m_weightsOn(i, j);
            }
        }

        m_firings[count] = false;
        ++count;
    }
}

void SpikingNetwork::generateNeuronConfiguration() {
/*    for (int i = 0; i <= WIDTH - NEURON_WIDTH; i += NEURON_WIDTH) {
        for (int j = 0; j <= HEIGHT - NEURON_HEIGHT; j += NEURON_HEIGHT) {
            m_neurons.emplace_back(OrientedSpikingNeuron(i, j, NO_GABOR, NO_GABOR, THRESHOLD));
        }
    }*/
    for (int i = 150; i <= 165; i += NEURON_WIDTH) {
        for (int j = 150; j <= 165; j += NEURON_HEIGHT) {
            m_neurons.emplace_back(OrientedSpikingNeuron(i, j, UNIFORM_WEIGHTS, UNIFORM_WEIGHTS, THRESHOLD));
        }
    }
}

void SpikingNetwork::assignNeurons() {
    for (size_t ind = 0; ind < m_neurons.size(); ind++) {
        int x = m_neurons[ind].getX();
        int y = m_neurons[ind].getY();
        for (int i = x; i < x + NEURON_WIDTH; i++) {
            for (int j = y; j < y + NEURON_HEIGHT; j++) {
                m_retina[i*HEIGHT+j].push_back(ind);
            }
        }
    }
}
