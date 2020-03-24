#include "SpikingNetwork.hpp"

SpikingNetwork::SpikingNetwork() {
    m_retina = std::vector<std::vector<size_t>>(WIDTH*HEIGHT, std::vector<size_t>(0));
    generateNeuronConfiguration();
    assignNeurons();

    m_potentials = std::deque<double>(1000, 0);
    m_timestamps = std::deque<long>(1000, 0);
    gp.sendLine("set title \"neuron's potential plotted against time\"");
    gp.sendLine("set yrange [" + std::to_string(VRESET) + ":" + std::to_string(VTHRESH) + "]");
    std::cout << "Number of neurons: " << m_neurons.size() << std::endl;
}

void SpikingNetwork::addEvent(const long timestamp, const int x, const int y, const bool polarity) {
    for (size_t ind : m_retina[x*HEIGHT+y]) {
        if (m_neurons[ind].newEvent(timestamp, x - m_neurons[ind].getX(), y - m_neurons[ind].getY(), polarity)) {
            for (size_t inhibit : m_retina[x*HEIGHT+y]) {
                if (inhibit != ind) {
                    m_neurons[inhibit].setInhibitionTime(timestamp);
                }
            }
        }

        if (ind == IND) {
            m_potentials.push_back(m_neurons[IND].getPotential(timestamp));
            m_potentials.pop_front();
            m_timestamps.push_back(timestamp);
            m_timestamps.pop_front();
        }
    }
}

void SpikingNetwork::updateNeurons(const long time) {
    for (auto &neuron : m_neurons) {
        neuron.update(time); //TODO
    }
}

void SpikingNetwork::saveWeights() {
    int count = 0;
    std::string fileName;
    for (auto &neuron : m_neurons) {
        fileName = SAVE_DATA_LOCATION + "neuron_" + std::to_string(count);
        neuron.saveState(fileName);
        ++count;
    }
}

void SpikingNetwork::loadWeights() {
    int count = 0;
    std::string fileName;
    for (auto &neuron : m_neurons) {
        fileName = SAVE_DATA_LOCATION + "neuron_" + std::to_string(count);
        neuron.loadState(fileName);
        ++count;
    }
}

void SpikingNetwork::generateNeuronConfiguration() {
    std::vector<long> delays;
    for (int synapse = 0; synapse < NEURON_SYNAPSES; synapse++) {
        delays.push_back(synapse * SYNAPSE_DELAY);
    }

    for (int i = X_ANCHOR_POINT; i < X_ANCHOR_POINT + NEURON_WIDTH * NETWORK_WIDTH; i += NEURON_WIDTH) {
        for (int j = Y_ANCHOR_POINT; j < Y_ANCHOR_POINT + NEURON_HEIGHT * NETWORK_HEIGHT; j += NEURON_HEIGHT) {
            for (int k = 0; k < NETWORK_DEPTH; ++k) {
                m_neurons.emplace_back(SpatioTemporalNeuron(i, j, uniformMatrixSynapses(NEURON_HEIGHT, NEURON_WIDTH, NEURON_SYNAPSES), delays, VTHRESH));
                //m_neurons.emplace_back(OrientedNeuron(i, j, uniformMatrix(NEURON_HEIGHT, NEURON_WIDTH), VTHRESH)); //TODO
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

}

void SpikingNetwork::updateDisplay(long time, std::vector<cv::Mat> &displays) {
    potentialDisplay();
    multiPotentialDisplay(time, displays[0]);
    spikingDisplay(displays[1]);
    weightDisplay(displays[2]);
}

void SpikingNetwork::potentialDisplay() {
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

void SpikingNetwork::weightDisplay(cv::Mat &display) {
    double weight;

    for (int x = 0; x < NEURON_WIDTH; ++x) {
        for (int y = 0; y < NEURON_HEIGHT; ++y) {
            for (int p = 0; p < 2; p++) {
                weight = m_neurons[IND].getWeights(p, SYNAPSE, x, y) * 255;
                //weight = m_neurons[IND].getWeights(p, x, y) * 15 * 255 / VTHRESH; //TODO
                if (weight > 255) { weight = 255; }
                if (weight < 0) { weight = 0; }
                display.at<cv::Vec3b>(y, x)[p+1] = static_cast<unsigned char>(weight);
            }
        }
    }
}

void SpikingNetwork::spikingDisplay(cv::Mat &display) {
    int count = 0;
    display = cv::Scalar(0, 0, 0);
    for (auto &neuron : m_neurons) {
        if ((count + NETWORK_DEPTH - LAYER) % NETWORK_DEPTH == 0) {
            display(cv::Rect(neuron.getX(), neuron.getY(), NEURON_WIDTH, NEURON_HEIGHT)) = 255 * neuron.hasSpiked();
        }
        ++count;
    }
    cv::rectangle(display, cv::Point(m_neurons[IND].getX(), m_neurons[IND].getY()), cv::Point(m_neurons[IND].getX() + NEURON_WIDTH, m_neurons[IND].getY() + NEURON_HEIGHT), cv::viz::Color::white());
}

void SpikingNetwork::multiPotentialDisplay(long time, cv::Mat &display) {
    double potential;
    int norm_potential;

    for (auto &neuron : m_neurons) {
        potential = neuron.getPotential(time);
        if (potential > neuron.getThreshold()) {
            norm_potential = 255;
        } else {
            norm_potential = static_cast<int>((potential / neuron.getThreshold()) * 255);
        }
        display(cv::Rect(neuron.getX(), neuron.getY(), NEURON_WIDTH, NEURON_HEIGHT)) = norm_potential;
    }
}
