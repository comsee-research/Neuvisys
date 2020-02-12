#include "SpikingNetwork.hpp"

SpikingNetwork::SpikingNetwork() {
    m_retina = std::vector<std::vector<size_t>>(WIDTH*HEIGHT, std::vector<size_t>(0));
    generateNeuronConfiguration();
    assignNeurons();

    m_potentials = std::deque<double>(1000, 0);
    m_timestamps = std::deque<long>(1000, 0);
    gp.sendLine("set title \"neuron's potential plotted against time\"");
    gp.sendLine("set yrange [-20:20]");
    std::cout << "Number of neurons: " << m_neurons.size() << std::endl;
}

void SpikingNetwork::addEvent(const long timestamp, const int x, const int y, const bool polarity) {
    for (size_t ind : m_retina[x*HEIGHT+y]) {
        m_neurons[ind].newEvent(timestamp, x - m_neurons[ind].getX(), y - m_neurons[ind].getY(), polarity);

        m_potentials.push_back(m_neurons[LOOKING_AT].getPotential(timestamp));
        m_potentials.pop_front();
        m_timestamps.push_back(timestamp);
        m_timestamps.pop_front();
    }
}

void SpikingNetwork::updateNeurons(long time) {
    for (auto &neuron : m_neurons) {
//        neuron.update(time);
    }
}

void SpikingNetwork::updateDisplay(long time, std::vector<cv::Mat> &displays) {
    double potential;
    int norm_potential;
    double weight;

    for (auto &neuron : m_neurons) {
        /***** Show an image of the neurons potentials *****/
//        potential = neuron.getPotential(time);
//        if (potential > neuron.getThreshold()) {
//            norm_potential = 255;
//        } else {
//            norm_potential = static_cast<int>((potential / neuron.getThreshold()) * 255);
//        }
//        displays[1](cv::Rect(neuron.getX(), neuron.getY(), NEURON_WIDTH, NEURON_HEIGHT)) = norm_potential;

        /***** Show an image of the neurons spikes *****/
        displays[1](cv::Rect(neuron.getX(), neuron.getY(), NEURON_WIDTH, NEURON_HEIGHT)) = 255 * neuron.hasSpiked();
    }

    /***** Show the weights potentials *****/
    for (int x = 0; x < NEURON_WIDTH; ++x) {
        for (int y = 0; y < NEURON_HEIGHT; ++y) {
            for (int p = 0; p < 2; p++) {
                weight = m_neurons[LOOKING_AT].getWeights(p, x, y) * 15 * 255 / THRESHOLD;
                if (weight > 255) { weight = 255; }
                if (weight < 0) { weight = 0; }
                displays[0].at<cv::Vec3b>(y, x)[p+1] = static_cast<unsigned char>(weight);
            }
        }
    }

    /***** Show a plot of a neuron potential *****/
    if (!m_potentials.empty()) {
        std::string plot;
        plot += "plot '-' lc rgb 'blue' with lines";
        for (size_t i = 0; i < m_timestamps.size(); ++i) {
            plot += "\n " + std::to_string(m_timestamps[i]) + " " + std::to_string(m_potentials[i]);
        }
        gp.sendLine(plot, true);
        gp.sendEndOfData();
        plot = "";
    }
}

void SpikingNetwork::generateNeuronConfiguration() {
    for (int i = X_ANCHOR_POINT; i < X_ANCHOR_POINT + NEURON_WIDTH * NETWORK_WIDTH; i += NEURON_WIDTH) {
        for (int j = Y_ANCHOR_POINT; j < Y_ANCHOR_POINT + NEURON_HEIGHT * NETWORK_HEIGHT; j += NEURON_HEIGHT) {
            for (int k = 0; k < NETWORK_DEPTH; ++k) {
                m_neurons.emplace_back(OrientedSpikingNeuron(i, j, UNIFORM_WEIGHTS, THRESHOLD));
            }
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

void SpikingNetwork::neuronsInfos() {
    for (auto &neuron : m_neurons) {
        neuron.resetSpikeCount();
        neuron.normalize();
    }
}
