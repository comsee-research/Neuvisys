#include "SpikingNetwork.hpp"

SpikingNetwork::SpikingNetwork(NetworkConfig &conf) : conf(conf), neuronConf(conf.CONF_FILES_LOCATION) {
    if (conf.WEIGHT_SHARING) {
        for (int patch = 0; patch < 9; ++patch) {
            for (int j = 0; j < conf.NETWORK_DEPTH; ++j) {
                m_sharedWeights.push_back(Util::uniformMatrixSynapses(conf.NEURON_HEIGHT, conf.NEURON_WIDTH, conf.NEURON_SYNAPSES));
            }
        }
    } else {
        for (int i = 0; i < conf.NETWORK_WIDTH * conf.NETWORK_HEIGHT * conf.NETWORK_DEPTH; ++i) {
            m_sharedWeights.push_back(Util::uniformMatrixSynapses(conf.NEURON_HEIGHT, conf.NEURON_WIDTH, conf.NEURON_SYNAPSES));
        }
    }

    m_retina = std::vector<std::vector<int>>(Conf::WIDTH * Conf::HEIGHT, std::vector<int>(0));
    generateNeuronConfiguration();
    assignNeurons();

    m_potentials = std::deque<double>(1000, 0);
    m_timestamps = std::deque<long>(1000, 0);
    m_spikes = std::vector<int>(0);

    std::vector<double> const& expM = Util::expLUT(neuronConf.TAU_M);
    std::cout << expM[999999] << std::endl;

//    gp.sendLine("set title \"neuron's potential plotted against time\"");
//    gp.sendLine("set yrange [" + std::to_string(VRESET) + ":" + std::to_string(VTHRESH) + "]");

    std::cout << "Number of neurons: " << m_neurons.size() << std::endl;
}

void SpikingNetwork::addEvent(const long timestamp, const int x, const int y, const bool polarity) {
    for (int ind : m_retina[x * Conf::HEIGHT + y]) {
        m_neurons[ind].newEvent(timestamp, x - m_neurons[ind].getX(), y - m_neurons[ind].getY(), polarity);

        if (ind == Selection::IND) {
            m_potentials.push_back(m_neurons[Selection::IND].getPotential(timestamp));
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
            for (auto inhibit : m_retina[m_neurons[ind].getX() * Conf::HEIGHT + m_neurons[ind].getY()]) {
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
        fileName = conf.SAVE_DATA_LOCATION + "neuron_" + std::to_string(count);
        neuron.saveState(fileName);
        ++count;
    }
}

void SpikingNetwork::loadWeights() {
    int count = 0;
    std::string fileName;
    for (auto &neuron : m_neurons) {
        fileName = conf.SAVE_DATA_LOCATION + "neuron_" + std::to_string(count);
        neuron.loadState(fileName);
        ++count;
    }
}

void SpikingNetwork::simpleConfiguration(const std::vector<long> &delays) {
    int count = 0;
    for (int i = conf.X_ANCHOR_POINT; i < conf.X_ANCHOR_POINT + conf.NEURON_WIDTH * conf.NETWORK_WIDTH; i += conf.NEURON_WIDTH) {
        for (int j = conf.Y_ANCHOR_POINT; j < conf.Y_ANCHOR_POINT + conf.NEURON_HEIGHT * conf.NETWORK_HEIGHT; j += conf.NEURON_HEIGHT) {
            for (int k = 0; k < conf.NETWORK_DEPTH; ++k) {
                m_neurons.emplace_back(SpatioTemporalNeuron(neuronConf, i, j, m_sharedWeights[count], delays));
                ++count;
            }
        }
    }
}

void SpikingNetwork::weightSharingConfiguration(const std::vector<long> &delays) {
    int patch = 0;
    std::vector<int> xs = {0, 153, 306};
    std::vector<int> ys = {0, 110, 220};
    for (int x : xs) {
        for (int y : ys) {
            for (int i = 0; i < 4 * conf.NEURON_WIDTH; i += conf.NEURON_WIDTH) {
                for (int j = 0; j < 4 * conf.NEURON_HEIGHT; j += conf.NEURON_HEIGHT) {
                    for (int k = 0; k < conf.NETWORK_DEPTH; ++k) {
                        m_neurons.emplace_back(SpatioTemporalNeuron(neuronConf, x+i, y+j, m_sharedWeights[patch * conf.NETWORK_DEPTH + k], delays));
                    }
                }
            }
            ++patch;
        }
    }
}

void SpikingNetwork::generateNeuronConfiguration() {
    std::vector<long> delays;
    for (int synapse = 0; synapse < conf.NEURON_SYNAPSES; synapse++) {
        delays.push_back(synapse * neuronConf.SYNAPSE_DELAY);
    }

    if (conf.WEIGHT_SHARING) {
        weightSharingConfiguration(delays);
    } else {
        simpleConfiguration(delays);
    }
}

void SpikingNetwork::assignNeurons() {
    for (size_t ind = 0; ind < m_neurons.size(); ind++) {
        int x = m_neurons[ind].getX();
        int y = m_neurons[ind].getY();
        for (int i = x; i < x + conf.NEURON_WIDTH; i++) {
            for (int j = y; j < y + conf.NEURON_HEIGHT; j++) {
                m_retina[i * Conf::HEIGHT + j].push_back(ind);
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
//    potentialDisplay();
    multiPotentialDisplay(time, displays[1]);
    spikingDisplay(displays[2]);
    weightDisplay(displays[3]);
    displays[0](cv::Rect(m_neurons[Selection::IND].getX(), m_neurons[Selection::IND].getY(), conf.NEURON_WIDTH, conf.NEURON_HEIGHT)).copyTo(displays[4]);
    cv::rectangle(displays[0], cv::Point(m_neurons[Selection::IND].getX(), m_neurons[Selection::IND].getY()), cv::Point(m_neurons[Selection::IND].getX() + conf.NEURON_WIDTH, m_neurons[Selection::IND].getY() + conf.NEURON_HEIGHT), cv::viz::Color::white());
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

    for (int x = 0; x < conf.NEURON_WIDTH; ++x) {
        for (int y = 0; y < conf.NEURON_HEIGHT; ++y) {
            for (int p = 0; p < 2; p++) {
                weight = m_neurons[Selection::IND].getWeights(p, Selection::SYNAPSE, x, y) * 255;
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
        if ((ind + conf.NETWORK_DEPTH - Selection::LAYER) % conf.NETWORK_DEPTH == 0) {
            display(cv::Rect(m_neurons[ind].getX(), m_neurons[ind].getY(), conf.NEURON_WIDTH, conf.NEURON_HEIGHT)) = 255;
        }
    }
    m_spikes.clear();
    cv::rectangle(display, cv::Point(m_neurons[Selection::IND].getX(), m_neurons[Selection::IND].getY()), cv::Point(m_neurons[Selection::IND].getX() + conf.NEURON_WIDTH, m_neurons[Selection::IND].getY() + conf.NEURON_HEIGHT), cv::viz::Color::white());
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
        display(cv::Rect(neuron.getX(), neuron.getY(), conf.NEURON_WIDTH, conf.NEURON_HEIGHT)) = norm_potential;
    }
}

SpatioTemporalNeuron SpikingNetwork::getNeuron(unsigned long index) {
    return m_neurons[index];
}
