#include <filesystem>
#include "SpikingNetwork.hpp"

SpikingNetwork::SpikingNetwork(NetworkConfig &conf) : conf(conf),
                                                      m_simpleNeuronConf(conf.NetworkPath + "configs/simple_cell_config.json", 0),
                                                      m_complexNeuronConf(conf.NetworkPath + "configs/complex_cell_config.json", 1),
                                                      m_retina(std::vector<std::vector<uint64_t>>(Conf::WIDTH * Conf::HEIGHT, std::vector<uint64_t>(0))),
                                                      m_potentials(std::deque<double>(1000, 0)),
                                                      m_timestamps(std::deque<long>(1000, 0)),
                                                      m_simpleSpikes(std::vector<uint64_t>(0)),
                                                      m_simpleluts(m_simpleNeuronConf.TAU_M, m_simpleNeuronConf.TAU_RP, m_simpleNeuronConf.TAU_SRA),
                                                      m_complexluts(m_complexNeuronConf.TAU_M, m_complexNeuronConf.TAU_RP, m_complexNeuronConf.TAU_SRA) {

//    gp.sendLine("set title \"neuron's potential plotted against time\"");
//    gp.sendLine("set yrange [" + std::to_string(m_simpleNeuronConf.VRESET) + ":" + std::to_string(m_simpleNeuronConf.VTHRESH) + "]");

    m_nbSimpleNeurons = conf.L1XAnchor.size() * conf.L1YAnchor.size() * conf.L1Width * conf.L1Height * conf.L1Depth;
    m_nbComplexNeurons = conf.L2XAnchor.size() * conf.L2YAnchor.size() * conf.L2Width * conf.L2Height * conf.L2Depth;

    bool simpleNeuronStored = simpleNeuronsFilesExists();
    bool complexNeuronStored = complexNeuronsFilesExists();
    std::cout << "Network generation: ";
    if (simpleNeuronStored) {
        std::cout << "found simple neurons files, ";
    }
    if (complexNeuronStored) {
        std::cout << "found complex neurons files";
    }
    std::cout << std::endl;

    generateWeightSharing(simpleNeuronStored, complexNeuronStored);
    generateNeuronConfiguration();
    assignNeurons();
    if (conf.SaveData) {
        loadWeights(simpleNeuronStored, complexNeuronStored);
    }

    assert(m_nbSimpleNeurons == m_simpleNeurons.size());
    assert(m_nbComplexNeurons == m_complexNeurons.size());
    std::cout << "Layer 1 neurons: " << m_nbSimpleNeurons << std::endl;
    std::cout << "Layer 2 neurons: " << m_nbComplexNeurons << std::endl;
}

SpikingNetwork::~SpikingNetwork() {
    std::cout << "Network reset" << std::endl << std::endl;
    if (conf.SaveData) {
        saveNeuronsStates();
    }
}

bool SpikingNetwork::simpleNeuronsFilesExists() const {
    return std::filesystem::exists(conf.NetworkPath + "weights/simple_cells/0.npy");
}

bool SpikingNetwork::complexNeuronsFilesExists() const {
    return std::filesystem::exists(conf.NetworkPath + "weights/complex_cells/0.npy");
}

void SpikingNetwork::addEvent(Event event) {
    for (size_t ind : m_retina[static_cast<uint32_t>(event.x()) * Conf::HEIGHT + static_cast<uint32_t>(event.y())]) {
        bool spike = m_simpleNeurons[ind].newEvent(Event(event.timestamp(),
                                            event.x() - static_cast<int16_t>(m_simpleNeurons[ind].getOffset().posx()),
                                            event.y() - static_cast<int16_t>(m_simpleNeurons[ind].getOffset().posy()), event.polarity(), event.camera()));
        if (spike) {
            for (auto &simpleNeuronToInhibit : m_simpleNeurons[ind].getInhibitionConnections()) {
                simpleNeuronToInhibit.get().inhibition();
            }
        }

        if (conf.Display && ind == Selection::INDEX2) {
//            m_potentials.push_back(m_neurons[Selection::IND].getPotential(timestamp));
            m_potentials.push_back(m_complexNeurons[Selection::INDEX2].getPotential(event.timestamp()));
            m_potentials.pop_front();
            m_timestamps.push_back(event.timestamp());
            m_timestamps.pop_front();
        }
    }
}

void SpikingNetwork::updateNeurons(const long time) {
    for (auto &simpleNeuron : m_simpleNeurons) {
        while (simpleNeuron.checkNewEvents(time)) {
            bool spike = simpleNeuron.update(); // update simple cell neurons (1st layer)
            if (spike) {
                for (auto &simpleNeuronToInhibit : simpleNeuron.getInhibitionConnections()) {
                    simpleNeuronToInhibit.get().inhibition();
                }
            }
        }

        if (simpleNeuron.hasSpiked()) {
            if (simpleNeuron.getPos().posz() == Selection::LAYER) {
                m_simpleSpikes.push_back(simpleNeuron.getIndex());
            }

            for (auto &complexNeuron : simpleNeuron.getOutConnections()) { // update complex cell neurons (2nd layer)
                complexNeuron.get().newEvent(NeuronEvent(simpleNeuron.getSpikingTime(),
                                             static_cast<int32_t>(simpleNeuron.getPos().posx() - complexNeuron.get().getOffset().posx()),
                                             static_cast<int32_t>(simpleNeuron.getPos().posy() - complexNeuron.get().getOffset().posy()),
                                             static_cast<int32_t>(simpleNeuron.getPos().posz() - complexNeuron.get().getOffset().posz())));

                if (complexNeuron.get().hasSpiked()) {
                    for (auto &complexNeuronToInhibit : simpleNeuron.getInhibitionConnections()) {
                        complexNeuronToInhibit.get().inhibition();
                    }
                    if (complexNeuron.get().getPos().posz() == Selection::LAYER2) {
                        m_complexSpikes.push_back(complexNeuron.get().getIndex());
                    }
                }
            }
        }
    }
}

void SpikingNetwork::generateWeightSharing(bool simpleNeuronStored, bool complexNeuronStored) {
    if (conf.WeightSharing) {
        //for (size_t patch = 0; patch < conf.L1XAnchor.size() * conf.L1YAnchor.size(); ++patch) {
            for (size_t j = 0; j < conf.L1Depth; ++j) {
                if (simpleNeuronStored) {
                    m_sharedWeightsSimple.emplace_back(NBPOLARITY, conf.NbCameras, static_cast<long>(conf.Neuron1Synapses), static_cast<long>(conf.Neuron1Width), static_cast<long>(conf.Neuron1Height));
                } else {
                    m_sharedWeightsSimple.push_back(Util::uniformMatrixSimple(NBPOLARITY,static_cast<long>(conf.NbCameras),
                                                                              static_cast<long>(conf.Neuron1Synapses), static_cast<long>(conf.Neuron1Width), static_cast<long>(conf.Neuron1Height)));
                }
            }
        //}
    } else {
        for (size_t i = 0; i < m_nbSimpleNeurons; ++i) {
            if (simpleNeuronStored) {
                m_sharedWeightsSimple.emplace_back(NBPOLARITY, conf.NbCameras, static_cast<long>(conf.Neuron1Synapses), static_cast<long>(conf.Neuron1Width), static_cast<long>(conf.Neuron1Height));
            } else {
                m_sharedWeightsSimple.push_back(Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(conf.NbCameras),
                                                                          static_cast<long>(conf.Neuron1Synapses), static_cast<long>(conf.Neuron1Width), static_cast<long>(conf.Neuron1Height)));
            }
        }
    }
    for (size_t i = 0; i < m_nbComplexNeurons; ++i) {
        if (complexNeuronStored) {
            m_sharedWeightsComplex.emplace_back(static_cast<long>(conf.Neuron2Width), static_cast<long>(conf.Neuron2Height), static_cast<long>(conf.Neuron2Depth));
        } else {
            m_sharedWeightsComplex.push_back(Util::uniformMatrixComplex(static_cast<long>(conf.Neuron2Width), static_cast<long>(conf.Neuron2Height), static_cast<long>(conf.Neuron2Depth)));
        }
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
                        m_layout1[{x * conf.L1Width + i, y * conf.L1Height + j, k}] = neuronIndex;
                        ++neuronIndex;
                    }
                }
            }
            //++countWeightSharing;
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
                        m_layout2[{x * conf.L2Width + i, y * conf.L2Height + j, k}] = neuronIndex;
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

        for (size_t z = 0; z < conf.L1Depth; ++z) { // simple cell inhibition
            if (z != simpleNeuron.getPos().posz()) {
                simpleNeuron.addInhibitionConnection(m_simpleNeurons[m_layout1[std::make_tuple(simpleNeuron.getPos().posx(), simpleNeuron.getPos().posy(), z)]]);
            }
        }
    }

    for (auto &complexNeuron : m_complexNeurons) {
        for (size_t i = complexNeuron.getOffset().posx(); i < complexNeuron.getOffset().posx() + conf.Neuron2Width; ++i) {
            for (size_t j = complexNeuron.getOffset().posy(); j < complexNeuron.getOffset().posy() + conf.Neuron2Height; ++j) {
                for (size_t k = complexNeuron.getOffset().posz(); k < complexNeuron.getOffset().posz() + conf.Neuron2Depth; ++k) {
                    complexNeuron.addInConnection(m_simpleNeurons[m_layout1[{i, j, k}]]);
                    m_simpleNeurons[m_layout1[{i, j, k}]].addOutConnection(complexNeuron);
                }
            }
        }

        for (size_t z = 0; z < conf.L2Depth; ++z) { // complex cell inhibition
            if (z != complexNeuron.getPos().posz()) {
                complexNeuron.addInhibitionConnection(m_complexNeurons[m_layout2[{complexNeuron.getPos().posx(), complexNeuron.getPos().posy(), z}]]);
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
    if (!m_simpleNeurons.empty()) {
        for (auto &neuron : m_simpleNeurons) {
            neuron.trackPotential(time);
        }
    }
    if (!m_complexNeurons.empty()) {
        for (auto &neuron : m_complexNeurons) {
            neuron.trackPotential(time);
        }
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

//[[maybe_unused]] void SpikingNetwork::potentialDisplay() {
//    if (!m_potentials.empty()) {
//        std::string plot;
//        plot += "plot '-' lc rgb 'blue' with lines";
//        for (size_t i = 0; i < m_timestamps.size(); ++i) {
//            plot += "\n " + std::to_string(m_timestamps[i]) + " " + std::to_string(m_potentials[i]);
//        }
//        gp.sendLine(plot, true);
//        gp.sendEndOfData();
//    }
//}

void SpikingNetwork::weightDisplay(cv::Mat &display) {
    cv::Mat temp = cv::Mat::zeros(static_cast<int>(conf.Neuron1Height), static_cast<int>(conf.Neuron1Width), CV_8UC3);
    if (m_nbSimpleNeurons > 0) {
        double weight;
        for (size_t x = 0; x < conf.Neuron1Width; ++x) {
            for (size_t y = 0; y < conf.Neuron1Height; ++y) {
                for (size_t p = 0; p < NBPOLARITY; p++) {
                    weight = m_simpleNeurons[Selection::INDEX].getWeights(p, Selection::CAMERA, Selection::SYNAPSE, x, y) * 255;
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
    return Position(m_simpleNeurons[m_layout1[{x, y, z}]].getOffset().posx(), m_simpleNeurons[m_layout1[{x, y, z}]].getOffset().posy());
}

void SpikingNetwork::saveNeuronsStates() {
    size_t count = 0;
    std::string fileName;
    for (auto &neuron : m_simpleNeurons) {
        fileName = conf.NetworkPath + "weights/simple_cells/" + std::to_string(count);
        neuron.saveState(fileName);
        neuron.saveWeights(fileName);
        ++count;
    }
    count = 0;
    for (auto &neuron : m_complexNeurons) {
        fileName = conf.NetworkPath + "weights/complex_cells/" + std::to_string(count);
        neuron.saveState(fileName);
        neuron.saveWeights(fileName);
        ++count;
    }
}

void SpikingNetwork::loadWeights(bool simpleNeuronStored, bool complexNeuronStored) {
    std::string fileName;
    size_t count = 0;
    if (simpleNeuronStored) {
        for (auto &neuron : m_simpleNeurons) {
            fileName = conf.NetworkPath + "weights/simple_cells/" + std::to_string(count);
            neuron.loadState(fileName);
            neuron.loadWeights(fileName);
            ++count;
        }
    }
    if (complexNeuronStored) {
        count = 0;
        for (auto &neuron : m_complexNeurons) {
            fileName = conf.NetworkPath + "weights/complex_cells/" + std::to_string(count);
            neuron.loadState(fileName);
            neuron.loadWeights(fileName);
            ++count;
        }
    }

    std::vector<size_t> data(static_cast<size_t>(conf.L1XAnchor.size() * conf.L1Width * conf.L1YAnchor.size() * conf.L1Height * conf.L1Depth));
    count = 0;
    for (size_t i = 0; i < conf.L1XAnchor.size() * conf.L1Width; ++i) {
        for (size_t j = 0; j < conf.L1YAnchor.size() * conf.L1Height; ++j) {
            for (size_t k = 0; k < conf.L1Depth; ++k) {
                data[count] = m_layout1[{i, j, k}];
                ++count;
            }
        }
    }
    cnpy::npy_save(conf.NetworkPath + "weights/layout1.npy", &data[0], {conf.L1XAnchor.size() * conf.L1Width, conf.L1YAnchor.size() * conf.L1Height, conf.L1Depth}, "w");
}