#include "SpikingNetwork.hpp"

SpikingNetwork::SpikingNetwork() {
    m_retina = std::vector<std::vector<int>>(WIDTH*HEIGHT, std::vector<int>(0));
    generateNeuronConfiguration();
    assignNeurons();

    m_potentials = std::deque<double>(1000, 0);
    m_timestamps = std::deque<long>(1000, 0);
    m_spikes = std::vector<int>(0);
    gp.sendLine("set title \"neuron's potential plotted against time\"");
    gp.sendLine("set yrange [" + std::to_string(VRESET) + ":" + std::to_string(VTHRESH) + "]");
    std::cout << "Number of neurons: " << m_neurons.size() << std::endl;
}

void SpikingNetwork::addEvent(const long timestamp, const int x, const int y, const bool polarity) {
    for (int ind : m_retina[x*HEIGHT+y]) {
        m_neurons[ind].newEvent(timestamp, x - m_neurons[ind].getX(), y - m_neurons[ind].getY(), polarity);

        if (ind == IND) {
            m_potentials.push_back(m_neurons[IND].getPotential(timestamp));
            m_potentials.pop_front();
            m_timestamps.push_back(timestamp);
            m_timestamps.pop_front();
        }
    }
}

void SpikingNetwork::updateNeurons(const long time) {
    for (int ind = 0; ind < m_neurons.size(); ++ind) {
        m_neurons[ind].update(time);
        if (m_neurons[ind].hasSpiked()) {
            for (auto inhibit : m_retina[m_neurons[ind].getX()*HEIGHT+m_neurons[ind].getY()]) {
                if (inhibit != ind) {
                    m_neurons[inhibit].inhibition();
                }
            }
            m_spikes.push_back(ind);
        }
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

void SpikingNetwork::updateNeuronsParameters() {
    for (auto &neuron : m_neurons) {
        neuron.thresholdAdaptation();
    }
}

void SpikingNetwork::updateDisplay(long time, std::vector<cv::Mat> &displays) {
    potentialDisplay();
    multiPotentialDisplay(time, displays[1]);
    spikingDisplay(displays[2]);
    weightDisplay(displays[3]);
    displays[0](cv::Rect(m_neurons[IND].getX(), m_neurons[IND].getY(), NEURON_WIDTH, NEURON_HEIGHT)).copyTo(displays[4]);
    cv::rectangle(displays[0], cv::Point(m_neurons[IND].getX(), m_neurons[IND].getY()), cv::Point(m_neurons[IND].getX() + NEURON_WIDTH, m_neurons[IND].getY() + NEURON_HEIGHT), cv::viz::Color::white());
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
                if (weight > 255) { weight = 255; }
                if (weight < 0) { weight = 0; }
                display.at<cv::Vec3b>(y, x)[2-p] = static_cast<unsigned char>(weight);
            }
        }
    }
}

void SpikingNetwork::spikingDisplay(cv::Mat &display) {
    display = cv::Scalar(0, 0, 0);
    for (auto ind : m_spikes) {
        if ((ind + NETWORK_DEPTH - LAYER) % NETWORK_DEPTH == 0) {
            display(cv::Rect(m_neurons[ind].getX(), m_neurons[ind].getY(), NEURON_WIDTH, NEURON_HEIGHT)) = 255;
        }
    }
    m_spikes.clear();
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

OrientedNeuron SpikingNetwork::getNeuron(unsigned long index) {
    return m_neurons[index];
}
