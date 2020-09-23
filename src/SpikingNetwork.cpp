#include "SpikingNetwork.hpp"

SpikingNetwork::SpikingNetwork(NetworkConfig &conf) : conf(conf),
                                                      m_simpleNeuronConf(conf.Neuron1Config, 0),
                                                      m_complexNeuronConf(conf.Neuron2Config, 1),
                                                      m_simpleRetina(std::vector<std::vector<size_t>>(Conf::WIDTH * Conf::HEIGHT, std::vector<size_t>(0))),
                                                      m_complexRetina(std::vector<std::vector<size_t>>(static_cast<size_t>(conf.L1Width * conf.L1Height * conf.L1Depth), std::vector<size_t>(0))),
                                                      m_potentials(std::deque<double>(1000, 0)),
                                                      m_timestamps(std::deque<long>(1000, 0)),
                                                      m_simpleSpikes(std::vector<size_t>(0)),
                                                      m_simpleluts(m_simpleNeuronConf.TAU_M, m_simpleNeuronConf.TAU_RP, m_simpleNeuronConf.TAU_SRA),
                                                      m_complexluts(m_complexNeuronConf.TAU_M, m_complexNeuronConf.TAU_RP, m_complexNeuronConf.TAU_SRA) {

    gp.sendLine("set title \"neuron's potential plotted against time\"");
    gp.sendLine("set yrange [" + std::to_string(m_simpleNeuronConf.VRESET) + ":" + std::to_string(m_simpleNeuronConf.VTHRESH) + "]");

    std::cout << "Network generation" << std::endl;
    generateWeightSharing();
    generateNeuronConfiguration();
    assignNeurons();
    if (conf.SaveData) {
        loadWeights();
    }

    m_nbSimpleNeurons = m_simpleNeurons.size();
    m_nbComplexNeurons = m_complexNeurons.size();
    std::cout << "Layer 1 neurons: " << m_nbSimpleNeurons << std::endl;
    std::cout << "Layer 2 neurons: " << m_nbComplexNeurons << std::endl;
}

SpikingNetwork::~SpikingNetwork() {
    std::cout << "Network reset" << std::endl;
    if (conf.SaveData) {
        saveWeights();
    }
}

void SpikingNetwork::addEvent(const long timestamp, const int x, const int y, const bool polarity) {
    for (size_t ind : m_simpleRetina[static_cast<unsigned int>(x * Conf::HEIGHT + y)]) {
        m_simpleNeurons[ind].newEvent(timestamp, x - m_simpleNeurons[ind].getX(), y - m_simpleNeurons[ind].getY(), polarity);

//        if (ind == Selection::INDEX2) {
////            m_potentials.push_back(m_neurons[Selection::IND].getPotential(timestamp));
//            m_potentials.push_back(m_complexNeurons[Selection::INDEX2].getPotential(timestamp));
//            m_potentials.pop_front();
//            m_timestamps.push_back(timestamp);
//            m_timestamps.pop_front();
//        }
    }
}

void SpikingNetwork::updateNeurons(const long time) {
    for (size_t simpInd = 0; simpInd < m_simpleNeurons.size(); ++simpInd) {
        m_simpleNeurons[simpInd].update(time); // update simple cell neurons (1st layer)
        if (m_simpleNeurons[simpInd].hasSpiked()) {
            if (m_layout1[simpInd].posz() == Selection::LAYER) {
                m_simpleSpikes.push_back(simpInd);
            }

            for (auto simpInhibit : m_simpleRetina[static_cast<unsigned int>(m_simpleNeurons[simpInd].m_connections(0, 0, 0, 0))]) { // list of simple cells connected to the same input pixels
                if (simpInhibit != simpInd) {
                    m_simpleNeurons[simpInhibit].inhibition();
                }
            }

            for (size_t compInd : m_complexRetina[simpInd]) { // update complex cell neurons (2nd layer)
                m_complexNeurons[compInd].newEvent(m_simpleNeurons[simpInd].getSpikingTime(), m_layout1[simpInd].posx() - m_complexNeurons[compInd].getX(),
                                                   m_layout1[simpInd].posy() - m_complexNeurons[compInd].getY(), m_layout1[simpInd].posz());

                if (m_complexNeurons[compInd].hasSpiked()) {
                    for (auto compInhibit : m_complexRetina[static_cast<unsigned int>(m_complexNeurons[compInd].m_connections(0, 0, 0))]) { // list of complex cells connected to the same simple cells
                        if (compInhibit != compInd) {
                            m_simpleNeurons[compInhibit].inhibition();
                        }
                    }

                    m_complexSpikes.push_back(compInd);
                }
            }
        }
    }
}

void SpikingNetwork::generateWeightSharing() {
    if (conf.WeightSharing) {
        for (size_t patch = 0; patch < conf.L1XAnchor.size() + conf.L1YAnchor.size(); ++patch) {
            for (int j = 0; j < conf.L1Depth; ++j) {
                m_sharedWeightsSimple.push_back(Util::uniformMatrixSynapses(conf.Neuron1Height, conf.Neuron1Width, conf.Neuron1Synapses));
            }
        }
    } else {
        for (size_t i = 0; i < conf.L1XAnchor.size() * conf.L1YAnchor.size() * static_cast<size_t>(conf.L1Width * conf.L1Height * conf.L1Depth); ++i) {
            m_sharedWeightsSimple.push_back(Util::uniformMatrixSynapses(conf.Neuron1Height, conf.Neuron1Width, conf.Neuron1Synapses));
        }
    }
    for (size_t i = 0; i < conf.L2XAnchor.size() * conf.L2YAnchor.size() * static_cast<size_t>(conf.L2Width * conf.L2Height * conf.L2Depth); ++i) {
        m_sharedWeightsComplex.push_back(Util::uniformMatrixPooling(conf.Neuron2Width, conf.Neuron2Height, conf.L1Depth));
    }
}

void SpikingNetwork::generateNeuronConfiguration() {
    int count = 0; // create simple cell neurons
    for (auto x : conf.L1XAnchor) {
        for (auto y : conf.L1YAnchor) {
            for (int i = 0; i < conf.L1Width * conf.Neuron1Width; i += conf.Neuron1Width) {
                for (int j = 0; j < conf.L1Height * conf.Neuron1Height; j += conf.Neuron1Height) {
                    for (int k = 0; k < conf.L1Depth; ++k) {
                        if (conf.WeightSharing) {
                            m_simpleNeurons.emplace_back(SimpleNeuron(m_simpleNeuronConf, m_simpleluts, x + i, y + j,
                                                                      m_sharedWeightsSimple[static_cast<unsigned int>(count * conf.L1Depth + k)], conf.Neuron1Synapses));
                        } else {
                            m_simpleNeurons.emplace_back(SimpleNeuron(m_simpleNeuronConf, m_simpleluts, x + i, y + j, m_sharedWeightsSimple[static_cast<unsigned int>(count)], conf.Neuron1Synapses));
                            ++count;
                        }
                        m_layout1.emplace_back(i / conf.Neuron1Width, j / conf.Neuron1Height, k);
                    }
                }
            }
            if (conf.WeightSharing) {
                ++count;
            }
        }
    }

    count = 0; // create complex cell neurons
    for (auto x : conf.L2XAnchor) {
        for (auto y : conf.L2YAnchor) {
            for (int i = 0; i < conf.Neuron2Width * conf.L2Width; i += conf.Neuron2Width) {
                for (int j = 0; j < conf.Neuron2Height * conf.L2Height; j += conf.Neuron2Height) {
                    for (int k = 0; k < conf.L2Depth; ++k) {
                        m_complexNeurons.emplace_back(ComplexNeuron(m_complexNeuronConf, m_complexluts, x + i, y + j,
                                                                    m_sharedWeightsComplex[static_cast<unsigned int>(count)]));
                        m_layout2.emplace_back(i, j, k);
                        ++count;
                    }
                }
            }
        }
    }
}

void SpikingNetwork::assignNeurons() {
    for (size_t ind = 0; ind < m_simpleNeurons.size(); ind++) {
        for (int i = m_simpleNeurons[ind].getX(); i < m_simpleNeurons[ind].getX() + conf.Neuron1Width; i++) {
            for (int j = m_simpleNeurons[ind].getY(); j < m_simpleNeurons[ind].getY() + conf.Neuron1Height; j++) {
                auto pixel = static_cast<unsigned int>(i * Conf::HEIGHT + j);
                m_simpleRetina[pixel].push_back(ind);
                m_simpleNeurons[ind].m_connections(0, 0, j - m_simpleNeurons[ind].getY(), i - m_simpleNeurons[ind].getX()) = pixel;
            }
        }
    }

    for (size_t ind = 0; ind < m_complexNeurons.size(); ++ind) {
        for (int i = m_complexNeurons[ind].getX(); i < m_complexNeurons[ind].getX() + conf.Neuron2Width; i++) {
            for (int j = m_complexNeurons[ind].getY(); j < m_complexNeurons[ind].getY() + conf.Neuron2Height; j++) {
                for (int k = 0; k < conf.L1Depth; ++k) {
                    auto neuron = static_cast<unsigned int>(i * conf.L1Height * conf.L1Depth + j * conf.L1Depth + k);
                    m_complexRetina[neuron].push_back(ind);
                    m_complexNeurons[ind].m_connections(k, j - m_complexNeurons[ind].getY(), i - m_complexNeurons[ind].getX()) = neuron;
                }
            }
        }
    }
}

void SpikingNetwork::updateNeuronsParameters(const long time) {
    for (auto &neuron : m_simpleNeurons) {
        neuron.thresholdAdaptation();
    }
}

void SpikingNetwork::trackNeuron(const long time) {
    m_complexNeurons[0].track(time);
}

void SpikingNetwork::updateDisplay(long time, std::map<std::string, cv::Mat> &displays) {
//    potentialDisplay();
    multiPotentialDisplay(time, displays["potentials"]);
    spikingDisplay(displays["spikes"]);
    weightDisplay(displays["weights"]);
    displays["frames"](cv::Rect(m_simpleNeurons[Selection::INDEX].getX(), m_simpleNeurons[Selection::INDEX].getY(),
                                conf.Neuron1Width, conf.Neuron1Height)).copyTo(displays["zoom"]);
    cv::rectangle(displays["frames"], cv::Point(m_simpleNeurons[Selection::INDEX].getX(), m_simpleNeurons[Selection::INDEX].getY()),
                  cv::Point(m_simpleNeurons[Selection::INDEX].getX() + conf.Neuron1Width, m_simpleNeurons[Selection::INDEX].getY() + conf.Neuron1Height),
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
    if (m_nbSimpleNeurons > 0) {
        double weight;
        for (int x = 0; x < conf.Neuron1Width; ++x) {
            for (int y = 0; y < conf.Neuron1Height; ++y) {
                for (int p = 0; p < 2; p++) {
                    weight = m_simpleNeurons[Selection::INDEX].getWeights(p, static_cast<int>(Selection::SYNAPSE), x, y) * 255;
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
    if (m_nbComplexNeurons > 0) {
        double weight;
        for (int x = 0; x < conf.Neuron2Width; ++x) {
            for (int y = 0; y < conf.Neuron2Height; ++y) {
                weight = m_complexNeurons[Selection::INDEX2].getWeights(x, y, Selection::LAYER) * 255;
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
    if (m_nbSimpleNeurons > 0) {
        for (auto ind : m_simpleSpikes) {
            display(cv::Rect(m_simpleNeurons[ind].getX(), m_simpleNeurons[ind].getY(), conf.Neuron1Width, conf.Neuron1Height)) = 255;
        }
        m_simpleSpikes.clear();
        cv::rectangle(display, cv::Point(m_simpleNeurons[Selection::INDEX].getX(), m_simpleNeurons[Selection::INDEX].getY()),
                      cv::Point(m_simpleNeurons[Selection::INDEX].getX() + conf.Neuron1Width, m_simpleNeurons[Selection::INDEX].getY() + conf.Neuron1Height),
                      cv::Scalar(255, 255, 255));
    }
}

void SpikingNetwork::spiking2Display(cv::Mat &display) {
    display = cv::Scalar(0, 0, 0);
    if (m_nbComplexNeurons > 0) {
        for (auto ind : m_complexSpikes) {
            display(cv::Rect(conf.L1XAnchor[0] + m_complexNeurons[ind].getX() * conf.Neuron1Width, conf.L1YAnchor[0] + m_complexNeurons[ind].getY() * conf.Neuron1Height,
                             conf.Neuron2Width * conf.Neuron1Width, conf.Neuron2Height * conf.Neuron1Height)) = 255;
        }
        m_complexSpikes.clear();
        cv::rectangle(display, cv::Point(conf.L1XAnchor[0] + m_complexNeurons[Selection::INDEX2].getX() * conf.Neuron1Width, conf.L1YAnchor[0] + m_complexNeurons[Selection::INDEX2].getY() * conf.Neuron1Height),
                      cv::Point(conf.L1XAnchor[0] + m_complexNeurons[Selection::INDEX2].getX() * conf.Neuron1Width + conf.Neuron2Width * conf.Neuron1Width,
                                conf.L1YAnchor[0] + m_complexNeurons[Selection::INDEX2].getY() * conf.Neuron1Height + conf.Neuron2Height * conf.Neuron1Height), cv::Scalar(255, 255, 255));
    }
}

void SpikingNetwork::multiPotentialDisplay(long time, cv::Mat &display) {
    int norm_potential;
    for (size_t ind = 0; ind < m_simpleNeurons.size(); ind++) {
        if (m_layout1[ind].posz() == Selection::LAYER) {
            double potential = m_simpleNeurons[ind].getPotential(time);
            if (potential > m_simpleNeurons[ind].getThreshold()) {
                norm_potential = 255;
            } else {
                norm_potential = static_cast<int>((potential / m_simpleNeurons[ind].getThreshold()) * 255);
            }
            display(cv::Rect(m_simpleNeurons[ind].getX(), m_simpleNeurons[ind].getY(), conf.Neuron1Width, conf.Neuron1Height)) = norm_potential;
        }
    }
}

void SpikingNetwork::multiPotential2Display(long time, cv::Mat &display) {
    int norm_potential;
    for (auto &neuron : m_complexNeurons) {
        double potential = neuron.getPotential(time);
        if (potential > neuron.getThreshold()) {
            norm_potential = 255;
        } else {
            norm_potential = static_cast<int>((potential / neuron.getThreshold()) * 255);
        }
        display(cv::Rect(conf.L1XAnchor[0] + neuron.getX() * conf.Neuron1Width, conf.L1YAnchor[0] + neuron.getY() * conf.Neuron1Height,
                conf.Neuron2Width * conf.Neuron1Width, conf.Neuron2Height * conf.Neuron1Height)) = norm_potential;
    }

}

SimpleNeuron SpikingNetwork::getNeuron(unsigned long index) {
    return m_simpleNeurons[index];
}

void SpikingNetwork::saveWeights() {
    int count = 0;
    std::string fileName;
    for (auto &neuron : m_simpleNeurons) {
        fileName = conf.SaveDataLocation + "weights/simple_cells/" + std::to_string(count);
        neuron.saveState(fileName);
        ++count;
    }
    count = 0;
    for (auto &neuron : m_complexNeurons) {
        fileName = conf.SaveDataLocation + "weights/complex_cells/" + std::to_string(count);
        neuron.saveState(fileName);
        ++count;
    }
}

void SpikingNetwork::loadWeights() {
    int count = 0;
    std::string fileName;
    for (auto &neuron : m_simpleNeurons) {
        fileName = conf.SaveDataLocation + "weights/simple_cells/" + std::to_string(count);
        neuron.loadState(fileName);
        ++count;
    }
    count = 0;
    for (auto &neuron : m_complexNeurons) {
        fileName = conf.SaveDataLocation + "weights/complex_cells/" + std::to_string(count);
        neuron.loadState(fileName);
        ++count;
    }
}