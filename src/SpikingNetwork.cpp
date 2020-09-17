#include "SpikingNetwork.hpp"

SpikingNetwork::SpikingNetwork(NetworkConfig &conf) : conf(conf),
    m_neuronConf(conf.Neuron1Config, 0),
    m_poolingNeuronConf(conf.Neuron2Config, 1),
    m_retina(std::vector<std::vector<size_t>>(Conf::WIDTH * Conf::HEIGHT, std::vector<size_t>(0))),
    m_poolingRetina(std::vector<std::vector<size_t>>(static_cast<size_t>(conf.L1Width * conf.L1Height * conf.L1Depth), std::vector<size_t>(0))),
    m_potentials(std::deque<double>(1000, 0)),
    m_timestamps(std::deque<long>(1000, 0)),
    m_spikes(std::vector<size_t>(0)),
    m_luts(m_neuronConf.TAU_M, m_neuronConf.TAU_RP, m_neuronConf.TAU_SRA) {

    gp.sendLine("set title \"neuron's potential plotted against time\"");
    gp.sendLine("set yrange [" + std::to_string(m_neuronConf.VRESET) + ":" + std::to_string(m_neuronConf.VTHRESH) + "]");

    std::cout << "Network generation" << std::endl;
    generateWeightSharing();
    generateNeuronConfiguration();
    assignNeurons();
    if (conf.SaveData) {
        loadWeights();
    }

    m_nbNeurons = m_neurons.size();
    m_nbPoolingNeurons = m_poolingNeurons.size();
    std::cout << "Layer 1 neurons: " << m_nbNeurons << std::endl;
    std::cout << "Layer 2 neurons: " << m_nbPoolingNeurons << std::endl;
}

SpikingNetwork::~SpikingNetwork() {
    std::cout << "Network reset" << std::endl;
    if (conf.SaveData) {
        saveWeights();
    }
}

void SpikingNetwork::addEvent(const long timestamp, const int x, const int y, const bool polarity) {
    for (size_t ind : m_retina[static_cast<unsigned int>(x * Conf::HEIGHT + y)]) {
        m_neurons[ind].newEvent(timestamp, x - m_neurons[ind].getX(), y - m_neurons[ind].getY(), polarity);

        if (ind == Selection::INDEX2) {
//            m_potentials.push_back(m_neurons[Selection::IND].getPotential(timestamp));
            m_potentials.push_back(m_poolingNeurons[Selection::INDEX2].getPotential(timestamp));
            m_potentials.pop_front();
            m_timestamps.push_back(timestamp);
            m_timestamps.pop_front();
        }
    }
}

void SpikingNetwork::updateNeurons(const long time) {
    for (size_t ind = 0; ind < m_neurons.size(); ++ind) {
        m_neurons[ind].update(time); // update simple cell neurons (1st layer)
        if (m_neurons[ind].hasSpiked()) {
            if (m_layout1[ind].posz() == Selection::LAYER) {
                m_spikes.push_back(ind);
            }

            for (auto inhibit : m_retina[static_cast<unsigned int>(m_neurons[ind].getX() * Conf::HEIGHT + m_neurons[ind].getY())]) {
                if (inhibit != ind) {
                    m_neurons[inhibit].inhibition();
                }
            }

            for (size_t poolInd : m_poolingRetina[ind]) { // update complex cell neurons (2nd layer)
                m_poolingNeurons[poolInd].newEvent(m_neurons[ind].getSpikingTime(), m_layout1[ind].posx() - m_poolingNeurons[poolInd].getX(),
                                                   m_layout1[ind].posy() - m_poolingNeurons[poolInd].getY(), m_layout1[ind].posz());

                if (m_poolingNeurons[poolInd].hasSpiked()) {
                    m_poolingSpikes.push_back(poolInd);
                }
            }
        }
    }
}

void SpikingNetwork::generateWeightSharing() {
    if (conf.WeightSharing) {
        for (int patch = 0; patch < 9; ++patch) {
            for (int j = 0; j < conf.L1Depth; ++j) {
                m_sharedWeights.push_back(Util::uniformMatrixSynapses(conf.Neuron1Height, conf.Neuron1Width, conf.Neuron1Synapses));
            }
        }
    } else {
        for (int i = 0; i < conf.L1Width * conf.L1Height * conf.L1Depth; ++i) {
            m_sharedWeights.push_back(Util::uniformMatrixSynapses(conf.Neuron1Height, conf.Neuron1Width, conf.Neuron1Synapses));
        }
    }
    for (int i = 0; i < conf.L2Width * conf.L2Height; ++i) {
        m_sharedWeightsPooling.push_back(Util::uniformMatrixPooling(conf.Neuron2Width, conf.Neuron2Height, conf.L1Depth));
    }
}

void SpikingNetwork::generateNeuronConfiguration() {
    if (conf.WeightSharing) {
        weightSharingConfiguration();
    } else {
        simpleConfiguration();
    }
}

void SpikingNetwork::simpleConfiguration() {
    int count = 0; // create simple cell neurons
    for (int i = conf.L1XAnchor; i < conf.L1XAnchor + conf.Neuron1Width * conf.L1Width; i += conf.Neuron1Width) {
        for (int j = conf.L1YAnchor; j < conf.L1YAnchor + conf.Neuron1Height * conf.L1Height; j += conf.Neuron1Height) {
            for (int k = 0; k < conf.L1Depth; ++k) {
                m_neurons.emplace_back(SpatioTemporalNeuron(m_neuronConf, m_luts, i, j, m_sharedWeights[static_cast<unsigned int>(count)], conf.Neuron1Synapses));
                m_layout1.emplace_back(i / conf.Neuron1Width, j / conf.Neuron1Height, k);
                ++count;
            }
        }
    }

    count = 0; // create complex cell neurons
    for (int i = 0; i < conf.Neuron2Width * conf.L2Width; i += conf.Neuron2Width) {
        for (int j = 0; j < conf.Neuron2Height * conf.L2Height; j += conf.Neuron2Height) {
            m_poolingNeurons.emplace_back(PoolingNeuron(m_poolingNeuronConf, m_luts, i, j, m_sharedWeightsPooling[static_cast<unsigned int>(count)]));
            m_layout2.emplace_back(i, j, 0);
            ++count;
        }
    }
}

void SpikingNetwork::weightSharingConfiguration() {
    int patch = 0;
    std::vector<int> xs = {0, 153, 306};
    std::vector<int> ys = {0, 80, 160};
    for (int x : xs) {
        for (int y : ys) {
            for (int i = 0; i < 4 * conf.Neuron1Width; i += conf.Neuron1Width) {
                for (int j = 0; j < 4 * conf.Neuron1Height; j += conf.Neuron1Height) {
                    for (int k = 0; k < conf.L1Depth; ++k) {
                        m_neurons.emplace_back(SpatioTemporalNeuron(m_neuronConf, m_luts, x + i, y + j,
                                                                    m_sharedWeights[static_cast<unsigned int>(patch * conf.L1Depth + k)], conf.Neuron1Synapses));
                        m_layout1.emplace_back(i / conf.Neuron1Width, j / conf.Neuron1Height, k);
                    }
                }
            }
            ++patch;
        }
    }

    int count = 0; // create complex cell neurons
    for (int i = 0; i < conf.Neuron2Width * conf.L2Width; i += conf.Neuron2Width) {
        for (int j = 0; j < conf.Neuron2Height * conf.L2Height; j += conf.Neuron2Height) {
            m_poolingNeurons.emplace_back(PoolingNeuron(m_poolingNeuronConf, m_luts, i, j, m_sharedWeightsPooling[static_cast<unsigned int>(count)]));
            m_layout2.emplace_back(i, j, 0);
            ++count;
        }
    }
}

void SpikingNetwork::assignNeurons() {
    for (size_t ind = 0; ind < m_neurons.size(); ind++) {
        for (int i = m_neurons[ind].getX(); i < m_neurons[ind].getX() + conf.Neuron1Width; i++) {
            for (int j = m_neurons[ind].getY(); j < m_neurons[ind].getY() + conf.Neuron1Height; j++) {
                auto pixel = static_cast<unsigned int>(i * Conf::HEIGHT + j);
                m_retina[pixel].push_back(ind);
                m_neurons[ind].m_connections(0, 0, j - m_neurons[ind].getY(), i - m_neurons[ind].getX()) = pixel;
            }
        }
    }

    for (size_t ind = 0; ind < m_poolingNeurons.size(); ++ind) {
        for (int i = m_poolingNeurons[ind].getX(); i < m_poolingNeurons[ind].getX() + conf.Neuron2Width; i++) {
            for (int j = m_poolingNeurons[ind].getY(); j < m_poolingNeurons[ind].getY() + conf.Neuron2Height; j++) {
                for (int k = 0; k < conf.L1Depth; ++k) {
                    auto neuron = static_cast<unsigned int>(i * conf.L1Height * conf.L1Depth + j * conf.L1Depth + k);
                    m_poolingRetina[neuron].push_back(ind);
                    m_poolingNeurons[ind].m_connections(k, j - m_poolingNeurons[ind].getY(), i - m_poolingNeurons[ind].getX()) = neuron;
                }
            }
        }
    }
}

void SpikingNetwork::updateNeuronsParameters(const long time) {
    for (auto &neuron : m_neurons) {
        neuron.thresholdAdaptation();
    }
}

void SpikingNetwork::updateDisplay(long time, std::map<std::string, cv::Mat> &displays) {
//    potentialDisplay();
    multiPotentialDisplay(time, displays["potentials"]);
    spikingDisplay(displays["spikes"]);
    weightDisplay(displays["weights"]);
    displays["frames"](cv::Rect(m_neurons[Selection::INDEX].getX(), m_neurons[Selection::INDEX].getY(),
            conf.Neuron1Width, conf.Neuron1Height)).copyTo(displays["zoom"]);
    cv::rectangle(displays["frames"], cv::Point(m_neurons[Selection::INDEX].getX(), m_neurons[Selection::INDEX].getY()),
                  cv::Point(m_neurons[Selection::INDEX].getX() + conf.Neuron1Width, m_neurons[Selection::INDEX].getY() + conf.Neuron1Height),
                  cv::Scalar(255, 255, 255));
    weight2Display(displays["weights2"]);
    multiPotential2Display(time, displays["potentials2"]);
    spiking2Display(displays["spikes2"]);
}

[[maybe_unused]] void SpikingNetwork::potentialDisplay() {
    if (!m_potentials.empty()) {
        std::string plot;
        plot += "plot '-' lc rgb 'blue' with lines";
        for (size_t i = 0; i < m_timestamps.size(); ++i) {
            plot += "\n " + std::to_string(m_timestamps[i]) + " " + std::to_string(m_potentials[i]);
        }
        gp.sendLine(plot, true);
        gp.sendEndOfData();
    }
}

void SpikingNetwork::weightDisplay(cv::Mat &display) {
    cv::Mat temp = cv::Mat::zeros(conf.Neuron1Height, conf.Neuron1Width, CV_8UC3);
    if (m_nbNeurons > 0) {
        double weight;
        for (int x = 0; x < conf.Neuron1Width; ++x) {
            for (int y = 0; y < conf.Neuron1Height; ++y) {
                for (int p = 0; p < 2; p++) {
                    weight = m_neurons[Selection::INDEX].getWeights(p, static_cast<int>(Selection::SYNAPSE), x, y) * 255;
                    if (weight > 255) { weight = 255; }
                    if (weight < 0) { weight = 0; }
                    temp.at<cv::Vec3b>(y, x)[2 - p] = static_cast<unsigned char>(weight);
                }
            }
        }
    }
    cv::resize(temp, display, display.size(), 0, 0, cv::INTER_NEAREST);
}

void SpikingNetwork::weight2Display(cv::Mat &display) {
    cv::Mat temp = cv::Mat::zeros(conf.Neuron2Height, conf.Neuron2Width, CV_8UC3);
    if (m_nbPoolingNeurons > 0) {
        double weight;
        for (int x = 0; x < conf.Neuron2Width; ++x) {
            for (int y = 0; y < conf.Neuron2Height; ++y) {
                weight = m_poolingNeurons[Selection::INDEX2].getWeights(x, y, Selection::LAYER) * 255;
                if (weight > 255) { weight = 255; }
                if (weight < 0) { weight = 0; }
                temp.at<cv::Vec3b>(y, x)[0] = static_cast<unsigned char>(weight);
            }
        }
    }
    cv::resize(temp, display, display.size(), 0, 0, cv::INTER_NEAREST);
}

void SpikingNetwork::spikingDisplay(cv::Mat &display) {
    display = cv::Scalar(0, 0, 0);
    if (m_nbNeurons > 0) {
        for (auto ind : m_spikes) {
            display(cv::Rect(m_neurons[ind].getX(), m_neurons[ind].getY(), conf.Neuron1Width, conf.Neuron1Height)) = 255;
        }
        m_spikes.clear();
        cv::rectangle(display, cv::Point(m_neurons[Selection::INDEX].getX(), m_neurons[Selection::INDEX].getY()),
                      cv::Point(m_neurons[Selection::INDEX].getX() + conf.Neuron1Width, m_neurons[Selection::INDEX].getY() + conf.Neuron1Height),
                      cv::Scalar(255, 255, 255));
    }
}

void SpikingNetwork::spiking2Display(cv::Mat &display) {
    display = cv::Scalar(0, 0, 0);
    if (m_nbPoolingNeurons > 0) {
        for (auto ind : m_poolingSpikes) {
            display(cv::Rect(m_poolingNeurons[ind].getX() * conf.Neuron1Width, m_poolingNeurons[ind].getY() * conf.Neuron1Height,
                             conf.Neuron2Width * conf.Neuron1Width, conf.Neuron2Height * conf.Neuron1Height)) = 255;
        }
        m_poolingSpikes.clear();
        cv::rectangle(display, cv::Point(m_poolingNeurons[Selection::INDEX2].getX() * conf.Neuron1Width, m_poolingNeurons[Selection::INDEX2].getY() * conf.Neuron1Height),
                      cv::Point(m_poolingNeurons[Selection::INDEX2].getX() * conf.Neuron1Width + conf.Neuron2Width * conf.Neuron1Width,
                                m_poolingNeurons[Selection::INDEX2].getY() * conf.Neuron1Height + conf.Neuron2Height * conf.Neuron1Height), cv::Scalar(255, 255, 255));
    }
}

void SpikingNetwork::multiPotentialDisplay(long time, cv::Mat &display) {
    int norm_potential;
    for (size_t ind = 0; ind < m_neurons.size(); ind++) {
        if (m_layout1[ind].posz() == Selection::LAYER) {
            double potential = m_neurons[ind].getPotential(time);
            if (potential > m_neurons[ind].getThreshold()) {
                norm_potential = 255;
            } else {
                norm_potential = static_cast<int>((potential / m_neurons[ind].getThreshold()) * 255);
            }
            display(cv::Rect(m_neurons[ind].getX(), m_neurons[ind].getY(), conf.Neuron1Width, conf.Neuron1Height)) = norm_potential;
        }
    }
}

void SpikingNetwork::multiPotential2Display(long time, cv::Mat &display) {
    int norm_potential;
    for (auto &neuron : m_poolingNeurons) {
        double potential = neuron.getPotential(time);
        if (potential > neuron.getThreshold()) {
            norm_potential = 255;
        } else {
            norm_potential = static_cast<int>((potential / neuron.getThreshold()) * 255);
        }
        display(cv::Rect(neuron.getX() * conf.Neuron1Width, neuron.getY() * conf.Neuron1Height,
                conf.Neuron2Width * conf.Neuron1Width, conf.Neuron2Height * conf.Neuron1Height)) = norm_potential;
    }

}

SpatioTemporalNeuron SpikingNetwork::getNeuron(unsigned long index) {
    return m_neurons[index];
}

void SpikingNetwork::saveWeights() {
    int count = 0;
    std::string fileName;
    for (auto &neuron : m_neurons) {
        fileName = conf.SaveDataLocation + "weights/simple_cells/" + std::to_string(count);
        neuron.saveState(fileName);
        ++count;
    }
    count = 0;
    for (auto &neuron : m_poolingNeurons) {
        fileName = conf.SaveDataLocation + "weights/complex_cells/" + std::to_string(count);
        neuron.saveState(fileName);
        ++count;
    }
}

void SpikingNetwork::loadWeights() {
    int count = 0;
    std::string fileName;
    for (auto &neuron : m_neurons) {
        fileName = conf.SaveDataLocation + "weights/simple_cells/" + std::to_string(count);
        neuron.loadState(fileName);
        ++count;
    }
    count = 0;
    for (auto &neuron : m_poolingNeurons) {
        fileName = conf.SaveDataLocation + "weights/complex_cells/" + std::to_string(count);
        neuron.loadState(fileName);
        ++count;
    }
}