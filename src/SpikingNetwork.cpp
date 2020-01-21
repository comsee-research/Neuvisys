#include "SpikingNetwork.hpp"

SpikingNetwork::SpikingNetwork() {
    generateNeuronConfiguration();
    assignNeurons();
    std::cout << "Number of neurons: " << m_neurons.size() << std::endl;
}

int SpikingNetwork::getNumberOfNeurons() {
    return m_neurons.size();
}

SpikingNeuron SpikingNetwork::getNeuron(size_t ind) {
    return m_neurons[ind];
}

void SpikingNetwork::addEvent(long timestamp, int x, int y, bool polarity) {
    for (size_t ind : m_retina[x*HEIGHT+y]) {
        m_neurons[ind].newEvent(timestamp, x - m_neurons[ind].getX(), y - m_neurons[ind].getY(), polarity);
    }
}

void SpikingNetwork::updateNeurons(long time) {
    for (SpikingNeuron& neuron : m_neurons) {
        if (!neuron.isEmpty() && neuron.getTimestampNextEvent() <= time) {
            neuron.update(time);
        }
    }
}

void SpikingNetwork::updateNeuronsInformation(long time, std::vector<cv::Mat> &displays) {
    int count = 0;
    for (SpikingNeuron& neuron : m_neurons) {
        if (!neuron.isEmpty() && neuron.getTimestampNextEvent() <= time) {
            /*neuronsFiring(cv::Rect(neuron.getX(), neuron.getY(), NEURON_WIDTH, NEURON_HEIGHT)) = 255 * */neuron.update(time);
        }

        double potential = neuron.getPotential(time);
        int norm_potential;
        if (potential > neuron.getThreshold()) {
            norm_potential = 255;
        } else {
            norm_potential = static_cast<int>((potential / neuron.getThreshold()) * 255);
        }
        displays[count % ADJACENT_NEURONS](cv::Rect(neuron.getX(), neuron.getY(), NEURON_WIDTH, NEURON_HEIGHT)) = norm_potential;
        count++;
    }
}

void SpikingNetwork::generateNeuronConfiguration() {
    for (int i = 0; i <= WIDTH - NEURON_WIDTH; i += NEURON_WIDTH) {
        for (int j = 0; j <= HEIGHT - NEURON_HEIGHT; j += NEURON_HEIGHT) {
            m_neurons.emplace_back(SpikingNeuron(i, j, GABOR_H, GABOR_H, NO_DELAYS, 10000));
/*            m_neurons.emplace_back(SpikingNeuron(i, j, GABOR_V, GABOR_V, NO_DELAYS, 10000));

            m_neurons.emplace_back(SpikingNeuron(i, j, NO_GABOR, NO_GABOR, DELAYS_LR, 10000));
            m_neurons.emplace_back(SpikingNeuron(i, j, NO_GABOR, NO_GABOR, DELAYS_RL, 10000));
            m_neurons.emplace_back(SpikingNeuron(i, j, NO_GABOR, NO_GABOR, DELAYS_TB, 10000));
            m_neurons.emplace_back(SpikingNeuron(i, j, NO_GABOR, NO_GABOR, DELAYS_BT, 10000));*/
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
