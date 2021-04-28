#include "SpikingNetwork.hpp"

#include <utility>

SpikingNetwork::SpikingNetwork(NetworkConfig &conf) : m_conf(conf),
                                                      m_simpleNeuronConf(conf.NetworkPath + "configs/simple_cell_config.json", 0),
                                                      m_complexNeuronConf(conf.NetworkPath + "configs/complex_cell_config.json", 1),
                                                      m_retina(std::vector<std::vector<uint64_t>>(Conf::WIDTH * Conf::HEIGHT, std::vector<uint64_t>(0))),
                                                      m_simpleluts(m_simpleNeuronConf.TAU_M, m_simpleNeuronConf.TAU_RP, m_simpleNeuronConf.TAU_SRA),
                                                      m_complexluts(m_complexNeuronConf.TAU_M, m_complexNeuronConf.TAU_RP, m_complexNeuronConf.TAU_SRA) {
    m_frameTime = std::chrono::high_resolution_clock::now();

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
    if (m_conf.SaveData) {
        saveNeuronsStates();
    }
}

bool SpikingNetwork::simpleNeuronsFilesExists() const {
    std::string path = m_conf.NetworkPath + "weights/simple_cells/0.npy";
    if (FILE *file = fopen(path.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

bool SpikingNetwork::complexNeuronsFilesExists() const {
    std::string path = m_conf.NetworkPath + "weights/complex_cells/0.npy";
    if (FILE *file = fopen(path.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

void SpikingNetwork::run(const std::vector<Event>& eventPacket) {
    for (Event event : eventPacket) {
        addEvent(event);

//    if (count % Conf::EVENT_FREQUENCY == 0) {
//        spinet.updateNeurons(event.timestamp());
//    }

        std::chrono::duration<double> frameElapsed = std::chrono::high_resolution_clock::now() - m_frameTime;
        if (1000000 * frameElapsed.count() > static_cast<double>(m_precisionEvent)) {
            m_frameTime = std::chrono::high_resolution_clock::now();
        }

        std::chrono::duration<double> trackingElapsed = std::chrono::high_resolution_clock::now() - m_trackingTime;
        if (1000000 * trackingElapsed.count() > static_cast<double>(m_precisionPotential)) {
            m_trackingTime = std::chrono::high_resolution_clock::now();
        }

        if (static_cast<size_t>(m_iterations) % Conf::UPDATE_PARAMETER_FREQUENCY == 0) {
            updateNeuronsParameters(event.timestamp());
            std::cout << static_cast<size_t>(100 * m_iterations) / eventPacket.size() << "%" << std::endl;
        }
        ++m_iterations;
    }
}

void SpikingNetwork::addEvent(Event event) {
    for (size_t ind : m_retina[static_cast<uint32_t>(event.x()) * Conf::HEIGHT + static_cast<uint32_t>(event.y())]) {
        if (m_simpleNeurons[ind].newEvent(Event(event.timestamp(),
                                                event.x() - static_cast<int16_t>(m_simpleNeurons[ind].getOffset().posx()),
                                                event.y() - static_cast<int16_t>(m_simpleNeurons[ind].getOffset().posy()),
                                                event.polarity(), event.camera()))) {
            for (auto &simpleNeuronToInhibit : m_simpleNeurons[ind].getInhibitionConnections()) {
                simpleNeuronToInhibit.get().inhibition();
            }
            addComplexEvent(m_simpleNeurons[ind]);
        }
    }
}

inline void SpikingNetwork::addComplexEvent(SimpleNeuron &neuron) {
    for (auto &complexNeuron : neuron.getOutConnections()) {
        if (complexNeuron.get().newEvent(NeuronEvent(neuron.getSpikingTime(),
                                                     static_cast<int32_t>(neuron.getPos().posx() - complexNeuron.get().getOffset().posx()),
                                                     static_cast<int32_t>(neuron.getPos().posy() - complexNeuron.get().getOffset().posy()),
                                                     static_cast<int32_t>(neuron.getPos().posz() - complexNeuron.get().getOffset().posz())))) {
            for (auto &complexNeuronToInhibit : complexNeuron.get().getInhibitionConnections()) {
                complexNeuronToInhibit.get().inhibition();
            }
        }
    }
}

void SpikingNetwork::updateNeurons(const long time) {
    for (auto &simpleNeuron : m_simpleNeurons) {
        while (simpleNeuron.checkRemainingEvents(time)) {
            if (simpleNeuron.update()) {
                for (auto &simpleNeuronToInhibit : simpleNeuron.getInhibitionConnections()) {
                    simpleNeuronToInhibit.get().inhibition();
                }
                addComplexEvent(simpleNeuron);
            }
        }
    }
}

void SpikingNetwork::generateWeightSharing(bool simpleNeuronStored, bool complexNeuronStored) {
    if (m_conf.SharingType == "none") {
        for (size_t i = 0; i < m_nbSimpleNeurons; ++i) {
            if (simpleNeuronStored) {
                m_sharedWeightsSimple.emplace_back(NBPOLARITY, m_conf.NbCameras, static_cast<long>(m_conf.Neuron1Synapses), static_cast<long>(m_conf.Neuron1Width), static_cast<long>(m_conf.Neuron1Height));
            } else {
                m_sharedWeightsSimple.push_back(Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(m_conf.NbCameras),
                                                                          static_cast<long>(m_conf.Neuron1Synapses), static_cast<long>(m_conf.Neuron1Width), static_cast<long>(m_conf.Neuron1Height)));
            }
        }
    } else {
        size_t patch_size;
        if (m_conf.SharingType == "full") {
            patch_size = 1;
        } else if (m_conf.SharingType == "patch") {
            patch_size = m_conf.L1XAnchor.size() * m_conf.L1YAnchor.size();
        } else {
            patch_size = 0;
            std::cout << "Wrong type of sharing" << std::endl;
        }
        for (size_t patch = 0; patch < patch_size; ++patch) {
            for (size_t j = 0; j < m_conf.L1Depth; ++j) {
                if (simpleNeuronStored) {
                    m_sharedWeightsSimple.emplace_back(NBPOLARITY, m_conf.NbCameras, static_cast<long>(m_conf.Neuron1Synapses), static_cast<long>(m_conf.Neuron1Width), static_cast<long>(m_conf.Neuron1Height));
                } else {
                    m_sharedWeightsSimple.push_back(Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(m_conf.NbCameras),
                                                                              static_cast<long>(m_conf.Neuron1Synapses), static_cast<long>(m_conf.Neuron1Width), static_cast<long>(m_conf.Neuron1Height)));
                }
            }
        }
    }
    for (size_t i = 0; i < m_nbComplexNeurons; ++i) {
        if (complexNeuronStored) {
            m_sharedWeightsComplex.emplace_back(static_cast<long>(m_conf.Neuron2Width), static_cast<long>(m_conf.Neuron2Height), static_cast<long>(m_conf.Neuron2Depth));
        } else {
            m_sharedWeightsComplex.push_back(Util::uniformMatrixComplex(static_cast<long>(m_conf.Neuron2Width), static_cast<long>(m_conf.Neuron2Height), static_cast<long>(m_conf.Neuron2Depth)));
        }
    }
}

void SpikingNetwork::generateNeuronConfiguration() {
    size_t neuronIndex = 0;
    size_t weightIndex;
    size_t countWeightSharing = 0; // create simple cell neurons
    for (size_t x = 0; x < m_conf.L1XAnchor.size(); ++x) {
        for (size_t y = 0; y < m_conf.L1YAnchor.size(); ++y) {
            for (size_t i = 0; i < m_conf.L1Width; ++i) {
                for (size_t j = 0; j < m_conf.L1Height; ++j) {
                    for (size_t k = 0; k < m_conf.L1Depth; ++k) {
                        if (m_conf.SharingType == "none") {
                            weightIndex = neuronIndex;
                        } else if (m_conf.SharingType == "full" || m_conf.SharingType == "patch") {
                            weightIndex = countWeightSharing * m_conf.L1Depth + k;
                        }
                        m_simpleNeurons.emplace_back(SimpleNeuron(neuronIndex, m_simpleNeuronConf, m_simpleluts, Position(x * m_conf.L1Width + i, y * m_conf.L1Height + j, k),
                                                                  Position(m_conf.L1XAnchor[x] + i * m_conf.Neuron1Width, m_conf.L1YAnchor[y] + j * m_conf.Neuron1Height),
                                                                  m_sharedWeightsSimple[weightIndex], m_conf.Neuron1Synapses));
                        m_layout1[{x * m_conf.L1Width + i, y * m_conf.L1Height + j, k}] = neuronIndex;
                        ++neuronIndex;
                    }
                }
            }
            if (m_conf.SharingType == "patch") {
                ++countWeightSharing;
            }
        }
    }

    neuronIndex = 0; // create complex cell neurons
    for (size_t x = 0; x < m_conf.L2XAnchor.size(); ++x) {
        for (size_t y = 0; y < m_conf.L2YAnchor.size(); ++y) {
            for (size_t i = 0; i < m_conf.L2Width; ++i) {
                for (size_t j = 0; j < m_conf.L2Height; ++j) {
                    for (size_t k = 0; k < m_conf.L2Depth; ++k) {
                        m_complexNeurons.emplace_back(ComplexNeuron(neuronIndex, m_complexNeuronConf,m_complexluts,Position(x * m_conf.L2Width + i, y * m_conf.L2Height + j, k),
                                                                    Position(m_conf.L2XAnchor[x] + i * m_conf.Neuron2Width, m_conf.L2YAnchor[y] + j * m_conf.Neuron2Height),
                                                                    m_sharedWeightsComplex[neuronIndex]));
                        m_layout2[{x * m_conf.L2Width + i, y * m_conf.L2Height + j, k}] = neuronIndex;
                        ++neuronIndex;
                    }
                }
            }
        }
    }
}

void SpikingNetwork::assignNeurons() {
    for (auto &simpleNeuron : m_simpleNeurons) {
        for (size_t i = simpleNeuron.getOffset().posx(); i < simpleNeuron.getOffset().posx() + m_conf.Neuron1Width; ++i) {
            for (size_t j = simpleNeuron.getOffset().posy(); j < simpleNeuron.getOffset().posy() + m_conf.Neuron1Height; ++j) {
                m_retina[i * Conf::HEIGHT + j].push_back(simpleNeuron.getIndex());
            }
        }

        for (size_t z = 0; z < m_conf.L1Depth; ++z) { // simple cell inhibition
            if (z != simpleNeuron.getPos().posz()) {
                simpleNeuron.addInhibitionConnection(m_simpleNeurons[m_layout1[{simpleNeuron.getPos().posx(), simpleNeuron.getPos().posy(), z}]]);
            }
        }
    }

    for (auto &complexNeuron : m_complexNeurons) {
        for (size_t i = complexNeuron.getOffset().posx(); i < complexNeuron.getOffset().posx() + m_conf.Neuron2Width; ++i) {
            for (size_t j = complexNeuron.getOffset().posy(); j < complexNeuron.getOffset().posy() + m_conf.Neuron2Height; ++j) {
                for (size_t k = complexNeuron.getOffset().posz(); k < complexNeuron.getOffset().posz() + m_conf.Neuron2Depth; ++k) {
                    complexNeuron.addInConnection(m_simpleNeurons[m_layout1[{i, j, k}]]);
                    m_simpleNeurons[m_layout1[{i, j, k}]].addOutConnection(complexNeuron);
                }
            }
        }

        for (size_t z = 0; z < m_conf.L2Depth; ++z) { // complex cell inhibition
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

void SpikingNetwork::saveNetworkLearningTrace(const size_t nbRun, const std::string& eventFileName) {
    std::string fileName;
    fileName = m_conf.NetworkPath + "learning_trace.txt";

    std::ofstream ofs(fileName, std::ios::app);
    if (ofs.is_open()) {
        ofs << eventFileName << ", " << nbRun << std::endl;
    } else {
        std::cout << "cannot save learning trace" << std::endl;
    }
    ofs.close();
}

void SpikingNetwork::saveNeuronsStates() {
    size_t count = 0;
    std::string fileName;
    for (auto &neuron : m_simpleNeurons) {
        fileName = m_conf.NetworkPath + "weights/simple_cells/" + std::to_string(count);
        neuron.saveState(fileName);
        neuron.saveWeights(fileName);
        ++count;
    }
    count = 0;
    for (auto &neuron : m_complexNeurons) {
        fileName = m_conf.NetworkPath + "weights/complex_cells/" + std::to_string(count);
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
            fileName = m_conf.NetworkPath + "weights/simple_cells/" + std::to_string(count);
            neuron.loadState(fileName);
            neuron.loadWeights(fileName);
            ++count;
        }
    }
    if (complexNeuronStored) {
        count = 0;
        for (auto &neuron : m_complexNeurons) {
            fileName = m_conf.NetworkPath + "weights/complex_cells/" + std::to_string(count);
            neuron.loadState(fileName);
            neuron.loadWeights(fileName);
            ++count;
        }
    }

    std::vector<size_t> data(static_cast<size_t>(m_conf.L1XAnchor.size() * m_conf.L1Width * m_conf.L1YAnchor.size() * m_conf.L1Height * m_conf.L1Depth));
    count = 0;
    for (size_t i = 0; i < m_conf.L1XAnchor.size() * m_conf.L1Width; ++i) {
        for (size_t j = 0; j < m_conf.L1YAnchor.size() * m_conf.L1Height; ++j) {
            for (size_t k = 0; k < m_conf.L1Depth; ++k) {
                data[count] = m_layout1[{i, j, k}];
                ++count;
            }
        }
    }
    cnpy::npy_save(m_conf.NetworkPath + "weights/layout1.npy", &data[0], {m_conf.L1XAnchor.size() * m_conf.L1Width, m_conf.L1YAnchor.size() * m_conf.L1Height, m_conf.L1Depth}, "w");
}

void SpikingNetwork::trackNeuron(const long time, const size_t simpleId, const size_t complexId) {
    if (m_simpleNeuronConf.TRACKING == "partial") {
        if (!m_simpleNeurons.empty()) {
            m_simpleNeurons[simpleId].trackPotential(time);
        }
    }
    if (m_complexNeuronConf.TRACKING == "partial") {
        if (!m_complexNeurons.empty()) {
            m_complexNeurons[complexId].trackPotential(time);
        }
    }
}

Position SpikingNetwork::findPixelComplexNeuron(ComplexNeuron &neuron) {
    size_t x = neuron.getOffset().posx();
    size_t y = neuron.getOffset().posy();
    size_t z = neuron.getOffset().posz();
    return Position(m_simpleNeurons[m_layout1[{x, y, z}]].getOffset().posx(), m_simpleNeurons[m_layout1[{x, y, z}]].getOffset().posy());
}

/***** GUI Interface *****/

cv::Mat SpikingNetwork::getWeightNeuron(size_t idNeuron, size_t camera, size_t synapse, size_t neuronType, size_t layer) {
    size_t sizeX = 0, sizeY = 0, sizePol = 0;
    if (neuronType == 0) {
        sizeX = m_conf.Neuron1Width;
        sizeY = m_conf.Neuron1Height;
        sizePol = NBPOLARITY;
    } else if(neuronType == 1) {
        sizeX = m_conf.Neuron2Width;
        sizeY = m_conf.Neuron2Height;
        sizePol = 1;
    }

    cv::Mat weightImage = cv::Mat::zeros(static_cast<int>(sizeX), static_cast<int>(sizeY), CV_8UC3);
    if ((neuronType == 0 && m_nbSimpleNeurons > 0) || (neuronType == 1 && m_nbComplexNeurons > 0)) {
        double weight;
        for (size_t x = 0; x < sizeX; ++x) {
            for (size_t y = 0; y < sizeY; ++y) {
                for (size_t p = 0; p < sizePol; p++) {
                    if (neuronType == 0) {
                        weight = m_simpleNeurons[idNeuron].getWeights(p, camera, synapse, x, y) * 255;
                    } else if (neuronType == 1) {
                        weight = m_complexNeurons[idNeuron].getWeights(x, y, layer) * 255;
                    }
                    if (weight > 255) { weight = 255; }
                    if (weight < 0) { weight = 0; }
                    if (neuronType == 0) {
                        weightImage.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x))[static_cast<int>(2 - p)] = static_cast<unsigned char>(weight);
                    } else if (neuronType == 1) {
                        weightImage.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x))[0] = static_cast<unsigned char>(weight);
                    }

                }
            }
        }
    }
    return weightImage;
}

const std::vector<long> &SpikingNetwork::getSpikingNeuron(size_t idNeuron, size_t neuronType) {
    if (neuronType == 0) {
        return m_simpleNeurons[idNeuron].getTrackingSpikeTrain();
    } else if (neuronType == 1) {
        return m_complexNeurons[idNeuron].getTrackingSpikeTrain();
    }
}

const std::vector<std::pair<double, long>> &SpikingNetwork::getPotentialNeuron(size_t idNeuron, size_t neuronType) {
    if (neuronType == 0) {
        return m_simpleNeurons[idNeuron].getTrackingPotentialTrain();
    } else if (neuronType == 1) {
        return m_complexNeurons[idNeuron].getTrackingPotentialTrain();
    }
}
