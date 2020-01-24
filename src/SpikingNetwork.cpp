#include "SpikingNetwork.hpp"

SpikingNetwork::SpikingNetwork() {
    m_retina = std::vector<std::vector<size_t>>(WIDTH*HEIGHT, std::vector<size_t>(0));
    generateNeuronConfiguration();
    assignNeurons();
    std::cout << "Number of neurons: " << m_neurons.size() << std::endl;
}

void SpikingNetwork::addEvent(const long timestamp, const int x, const int y, const bool polarity) {
    for (size_t ind : m_retina[x*HEIGHT+y]) {
        m_neurons[ind].newEvent(timestamp, x - m_neurons[ind].getX(), y - m_neurons[ind].getY(), polarity);
    }
}

void SpikingNetwork::updateNeurons(long time) {
    for (auto &neuron : m_neurons) {
        neuron.update(time);
    }
}

void SpikingNetwork::updateDisplay(long time, std::vector<cv::Mat> &displays) {
    int count = 0;
    double potential;
    int norm_potential;
    for (auto &neuron : m_neurons) {
        potential = neuron.getPotential(time);
        if (potential > neuron.getThreshold()) {
            norm_potential = 255;
        } else {
            norm_potential = static_cast<int>((potential / neuron.getThreshold()) * 255);
        }
        displays[count % ADJACENT_NEURONS](cv::Rect(neuron.getX(), neuron.getY(), NEURON_WIDTH, NEURON_HEIGHT)) = norm_potential;
        ++count;
    }
}

void SpikingNetwork::generateNeuronConfiguration() {
    for (int i = 0; i <= WIDTH - NEURON_WIDTH; i += NEURON_WIDTH) {
        for (int j = 0; j <= HEIGHT - NEURON_HEIGHT; j += NEURON_HEIGHT) {
            //m_neurons.emplace_back(OrientedSpikingNeuron(i, j, NO_GABOR, NO_GABOR, 10000));
            m_neurons.emplace_back(DelayedSpikingNeuron(i, j, NO_GABOR, NO_GABOR, DELAYS_LR, 10000));
            m_neurons.emplace_back(DelayedSpikingNeuron(i, j, NO_GABOR, NO_GABOR, DELAYS_RL, 10000));
            m_neurons.emplace_back(DelayedSpikingNeuron(i, j, NO_GABOR, NO_GABOR, DELAYS_TB, 10000));
            m_neurons.emplace_back(DelayedSpikingNeuron(i, j, NO_GABOR, NO_GABOR, DELAYS_BT, 10000));
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
