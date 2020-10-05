#include "SpikingNetwork.hpp"

SpikingNetwork::SpikingNetwork(NetworkConfig &conf) : conf(conf),
                                                      m_simpleNeuronConf(conf.Neuron1Config, 0),
                                                      m_complexNeuronConf(conf.Neuron2Config, 1),
                                                      m_retina(std::vector<std::vector<size_t>>(Conf::WIDTH * Conf::HEIGHT, std::vector<size_t>(0))),
                                                      m_potentials(std::deque<double>(1000, 0)),
                                                      m_timestamps(std::deque<long>(1000, 0)),
                                                      m_simpleSpikes(std::vector<size_t>(0)),
                                                      m_simpleluts(m_simpleNeuronConf.TAU_M, m_simpleNeuronConf.TAU_RP, m_simpleNeuronConf.TAU_SRA),
                                                      m_complexluts(m_complexNeuronConf.TAU_M, m_complexNeuronConf.TAU_RP, m_complexNeuronConf.TAU_SRA) {

    gp.sendLine("set title \"neuron's potential plotted against time\"");
    gp.sendLine("set yrange [" + std::to_string(m_simpleNeuronConf.VRESET) + ":" + std::to_string(m_simpleNeuronConf.VTHRESH) + "]");

    m_layout1 = xt::zeros<size_t>({conf.L1XAnchor.size() * conf.L1Width, conf.L1YAnchor.size() * conf.L1Height, conf.L1Depth});
    m_layout2 = xt::zeros<size_t>({conf.L2XAnchor.size() * conf.L2Width, conf.L2YAnchor.size() * conf.L2Height, conf.L2Depth});

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
    std::cout << "Network reset" << std::endl << std::endl;
    if (conf.SaveData) {
        saveWeights();
    }
}

void SpikingNetwork::addEvent(const long timestamp, const size_t x, const size_t y, const bool polarity) {
    for (size_t ind : m_retina[x * Conf::HEIGHT + y]) {
        m_simpleNeurons[ind].newEvent(timestamp, x - m_simpleNeurons[ind].getOffset().posx(), y - m_simpleNeurons[ind].getOffset().posy(), polarity);

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
    for (auto &simpleNeuron : m_simpleNeurons) {
        simpleNeuron.update(time); // update simple cell neurons (1st layer)
        if (simpleNeuron.hasSpiked()) {
            if (simpleNeuron.getPos().posz() == Selection::LAYER) {
                m_simpleSpikes.push_back(simpleNeuron.getIndex());
            }

            for (size_t i = 0; i < conf.L1Depth; ++i) { // simple cell inhibition
                if (i != simpleNeuron.getPos().posz()) {
                    m_simpleNeurons[m_layout1(simpleNeuron.getPos().posx(), simpleNeuron.getPos().posy(), i)].inhibition();
                }
            }

            for (auto &complexNeuron : simpleNeuron.getOutConnections()) { // update complex cell neurons (2nd layer)
                complexNeuron.get().newEvent(simpleNeuron.getSpikingTime(), simpleNeuron.getPos().posx() - complexNeuron.get().getOffset().posx(),
                                             simpleNeuron.getPos().posy() - complexNeuron.get().getOffset().posy(), simpleNeuron.getPos().posz());

                if (complexNeuron.get().hasSpiked()) {
                    for (size_t i = 0; i < conf.L2Depth; ++i) { // complex cell inhibition
                        if (i != complexNeuron.get().getPos().posz()) {
                            m_complexNeurons[m_layout2(complexNeuron.get().getPos().posx(), complexNeuron.get().getPos().posy(), i)].inhibition();
                        }
                    }

                    m_complexSpikes.push_back(complexNeuron.get().getIndex());
                }
            }
        }
    }
}

void SpikingNetwork::generateWeightSharing() {
    if (conf.WeightSharing) {
        for (size_t patch = 0; patch < conf.L1XAnchor.size() * conf.L1YAnchor.size(); ++patch) {
            for (size_t j = 0; j < conf.L1Depth; ++j) {
                m_sharedWeightsSimple.push_back(Util::uniformMatrixSimple(conf.Neuron1Height, conf.Neuron1Width, conf.Neuron1Synapses));
            }
        }
    } else {
        for (size_t i = 0; i < conf.L1XAnchor.size() * conf.L1YAnchor.size() * conf.L1Width * conf.L1Height * conf.L1Depth; ++i) {
            m_sharedWeightsSimple.push_back(Util::uniformMatrixSimple(conf.Neuron1Height, conf.Neuron1Width, conf.Neuron1Synapses));
        }
    }
    for (size_t i = 0; i < conf.L2XAnchor.size() * conf.L2YAnchor.size() * conf.L2Width * conf.L2Height * conf.L2Depth; ++i) {
        m_sharedWeightsComplex.push_back(Util::uniformMatrixComplex(conf.Neuron2Width, conf.Neuron2Height, conf.L1Depth));
    }
}

void SpikingNetwork::generateNeuronConfiguration() {
    size_t neuronIndex = 0;
    size_t weightIndex;
    size_t countWeightSharing = 0; // create simple cell neurons
    for (size_t x = 0; x < conf.L1XAnchor.size(); ++x) {
        for (size_t y = 0; y < conf.L1YAnchor.size(); ++y) {
            for (size_t i = 0; i < conf.L1Width; ++i) {
                for (size_t j = 0; j < conf.L1Height; ++j) {
                    for (size_t k = 0; k < conf.L1Depth; ++k) {
                        if (conf.WeightSharing) {
                            weightIndex = countWeightSharing * conf.L1Depth + k;
                        } else {
                            weightIndex = neuronIndex;
                        }
                        m_simpleNeurons.emplace_back(SimpleNeuron(neuronIndex, m_simpleNeuronConf, m_simpleluts,Position(x * conf.L1Width + i, y * conf.L1Height + j, k),
                                                                  Position(conf.L1XAnchor[x] + i * conf.Neuron1Width, conf.L1YAnchor[y] + j * conf.Neuron1Height),
                                                                  m_sharedWeightsSimple[weightIndex], conf.Neuron1Synapses));
                        m_layout1(x * conf.L1Width + i, y * conf.L1Height + j, k) = neuronIndex;
                        ++neuronIndex;
                    }
                }
            }
            ++countWeightSharing;
        }
    }

    neuronIndex = 0; // create complex cell neurons
    for (size_t x = 0; x < conf.L2XAnchor.size(); ++x) {
        for (size_t y = 0; y < conf.L2YAnchor.size(); ++y) {
            for (size_t i = 0; i < conf.L2Width; ++i) {
                for (size_t j = 0; j < conf.L2Height; ++j) {
                    for (size_t k = 0; k < conf.L2Depth; ++k) {
                        m_complexNeurons.emplace_back(ComplexNeuron(neuronIndex, m_complexNeuronConf,m_complexluts,Position(x * conf.L2Width + i, y * conf.L2Height + j, k),
                                                                    Position(conf.L2XAnchor[x] + i * conf.Neuron2Width, conf.L2YAnchor[y] + j * conf.Neuron2Height),
                                                                    m_sharedWeightsComplex[neuronIndex]));
                        m_layout2(x * conf.L2Width + i, y * conf.L2Height + j, k) = neuronIndex;
                        ++neuronIndex;
                    }
                }
            }
        }
    }
}

void SpikingNetwork::assignNeurons() {
    for (auto &simpleNeuron : m_simpleNeurons) {
        for (size_t i = simpleNeuron.getOffset().posx(); i < simpleNeuron.getOffset().posx() + conf.Neuron1Width; ++i) {
            for (size_t j = simpleNeuron.getOffset().posy(); j < simpleNeuron.getOffset().posy() + conf.Neuron1Height; ++j) {
                m_retina[i * Conf::HEIGHT + j].push_back(simpleNeuron.getIndex());
            }
        }
    }

    for (auto &complexNeuron : m_complexNeurons) {
        for (size_t i = complexNeuron.getOffset().posx(); i < complexNeuron.getOffset().posx() + conf.Neuron2Width; ++i) {
            for (size_t j = complexNeuron.getOffset().posy(); j < complexNeuron.getOffset().posy() + conf.Neuron2Height; ++j) {
                for (size_t k = 0; k < conf.L1Depth; ++k) {
                    complexNeuron.addInConnection(m_simpleNeurons[m_layout1(i, j, k)]);
                    m_simpleNeurons[m_layout1(i, j, k)].addOutConnection(complexNeuron);
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
    if (!m_complexNeurons.empty()) {
        m_complexNeurons[0].track(time);
    }
    if (!m_simpleNeurons.empty()) {
//        m_simpleNeurons[16742].track(time);
    }
}

void SpikingNetwork::updateDisplay(long time, std::map<std::string, cv::Mat> &displays) {
//    potentialDisplay();
    multiPotentialDisplay(time, displays["potentials"]);
    spikingDisplay(displays["spikes"]);
    weightDisplay(displays["weights"]);

    displays["frames"](cv::Rect(static_cast<int>(m_simpleNeurons[Selection::INDEX].getOffset().posx()),static_cast<int>(m_simpleNeurons[Selection::INDEX].getOffset().posy()),
                                static_cast<int>(conf.Neuron1Width), static_cast<int>(conf.Neuron1Height))).copyTo(displays["zoom"]);

    auto point1 = cv::Point(static_cast<int>(m_simpleNeurons[Selection::INDEX].getOffset().posx()),static_cast<int>(m_simpleNeurons[Selection::INDEX].getOffset().posy()));
    auto point2 = cv::Point(point1.x + static_cast<int>(conf.Neuron1Width), point1.y + static_cast<int>(conf.Neuron1Height));
    cv::rectangle(displays["frames"], point1, point2, cv::Scalar(255, 255, 255));

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
    cv::Mat temp = cv::Mat::zeros(static_cast<int>(conf.Neuron1Height), static_cast<int>(conf.Neuron1Width), CV_8UC3);
    if (m_nbSimpleNeurons > 0) {
        double weight;
        for (size_t x = 0; x < conf.Neuron1Width; ++x) {
            for (size_t y = 0; y < conf.Neuron1Height; ++y) {
                for (size_t p = 0; p < 2; p++) {
                    weight = m_simpleNeurons[Selection::INDEX].getWeights(p, Selection::SYNAPSE, x, y) * 255;
                    if (weight > 255) { weight = 255; }
                    if (weight < 0) { weight = 0; }
                    temp.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x))[static_cast<int>(2 - p)] = static_cast<unsigned char>(weight);
                }
            }
        }
    }
    cv::resize(temp, display, display.size(), 0, 0, cv::INTER_NEAREST);
}

void SpikingNetwork::weight2Display(cv::Mat &display) {
    cv::Mat temp = cv::Mat::zeros(static_cast<int>(conf.Neuron2Height), static_cast<int>(conf.Neuron2Width), CV_8UC3);
    if (m_nbComplexNeurons > 0) {
        double weight;
        for (size_t x = 0; x < conf.Neuron2Width; ++x) {
            for (size_t y = 0; y < conf.Neuron2Height; ++y) {
                weight = m_complexNeurons[Selection::INDEX2].getWeights(x, y, Selection::LAYER) * 255;
                if (weight > 255) { weight = 255; }
                if (weight < 0) { weight = 0; }
                temp.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x))[0] = static_cast<unsigned char>(weight);
            }
        }
    }
    cv::resize(temp, display, display.size(), 0, 0, cv::INTER_NEAREST);
}

void SpikingNetwork::spikingDisplay(cv::Mat &display) {
    display = cv::Scalar(0, 0, 0);
    if (m_nbSimpleNeurons > 0) {
        for (auto ind : m_simpleSpikes) {
            display(cv::Rect(static_cast<int>(m_simpleNeurons[ind].getOffset().posx()),static_cast<int>(m_simpleNeurons[ind].getOffset().posy()),
                             static_cast<int>(conf.Neuron1Width), static_cast<int>(conf.Neuron1Height))) = 255;
        }
        m_simpleSpikes.clear();
        auto point1 = cv::Point(static_cast<int>(m_simpleNeurons[Selection::INDEX].getOffset().posx()),static_cast<int>(m_simpleNeurons[Selection::INDEX].getOffset().posy()));
        auto point2 = cv::Point(point1.x + static_cast<int>(conf.Neuron1Width), point1.y + static_cast<int>(conf.Neuron1Height));
        cv::rectangle(display, point1, point2, cv::Scalar(255, 255, 255));
    }
}

void SpikingNetwork::spiking2Display(cv::Mat &display) {
    display = cv::Scalar(0, 0, 0);
    if (m_nbComplexNeurons > 0) {
        for (auto ind : m_complexSpikes) {
            Position pos = findPixelComplexNeuron(m_complexNeurons[ind]);
            display(cv::Rect(static_cast<int>(pos.posx()), static_cast<int>(pos.posy()),
                             static_cast<int>(conf.Neuron2Width * conf.Neuron1Width), static_cast<int>(conf.Neuron2Height * conf.Neuron1Height))) = 255;
        }
        m_complexSpikes.clear();

        /* Display white rectangle for current complex neuron on UI */
        Position pos = findPixelComplexNeuron(m_complexNeurons[Selection::INDEX2]);
        auto point1 = cv::Point(static_cast<int>(pos.posx()), static_cast<int>(pos.posy()));
        auto point2 = cv::Point(point1.x + static_cast<int>(conf.Neuron2Width * conf.Neuron1Width), point1.y + static_cast<int>(conf.Neuron2Height * conf.Neuron1Height));
        cv::rectangle(display, point1, point2, cv::Scalar(255, 255, 255));
    }
}

void SpikingNetwork::multiPotentialDisplay(long time, cv::Mat &display) {
    int norm_potential;
    for (auto &simpleNeuron : m_simpleNeurons) {
        if (simpleNeuron.getPos().posz() == Selection::LAYER) {
            double potential = simpleNeuron.getPotential(time);
            if (potential > simpleNeuron.getThreshold()) {
                norm_potential = 255;
            } else {
                norm_potential = static_cast<int>((potential / simpleNeuron.getThreshold()) * 255);
            }
            display(cv::Rect(static_cast<int>(simpleNeuron.getOffset().posx()), static_cast<int>(simpleNeuron.getOffset().posy()),
                             static_cast<int>(conf.Neuron1Width), static_cast<int>(conf.Neuron1Height))) = norm_potential;
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
        Position pos = findPixelComplexNeuron(neuron);
        display(cv::Rect(static_cast<int>(pos.posx()), static_cast<int>(pos.posy()),
                         static_cast<int>(conf.Neuron2Width * conf.Neuron1Width), static_cast<int>(conf.Neuron2Height * conf.Neuron1Height))) = norm_potential;
    }
}

Position SpikingNetwork::findPixelComplexNeuron(ComplexNeuron &neuron) {
    size_t x = neuron.getOffset().posx();
    size_t y = neuron.getOffset().posy();
    size_t z = neuron.getOffset().posz();
    return Position(m_simpleNeurons[m_layout1(x, y, z)].getOffset().posx(), m_simpleNeurons[m_layout1(x, y, z)].getOffset().posy());
}

void SpikingNetwork::saveWeights() {
    size_t count = 0;
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
    size_t count = 0;
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