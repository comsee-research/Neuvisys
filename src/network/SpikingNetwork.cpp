#include "SpikingNetwork.hpp"

#include <utility>

SpikingNetwork::SpikingNetwork(const std::string &conf) : m_conf(NetworkConfig(conf)),
                                                          m_simpleNeuronConf(m_conf.NetworkPath + "configs/simple_cell_config.json", 0),
                                                          m_complexNeuronConf(m_conf.NetworkPath + "configs/complex_cell_config.json", 1),
                                                          m_motorNeuronConf(m_conf.NetworkPath + "configs/motor_cell_config.json", 2),
                                                          m_pixelMapping(std::vector<std::vector<uint64_t>>
                                                                                 (Conf::WIDTH * Conf::HEIGHT, std::vector<uint64_t>(0))) {

//    addLayer("SimpleCell", m_conf.SharingType, {m_conf.L1XAnchor, m_conf.L1YAnchor, {0}}, {m_conf.L1Width, m_conf.L1Height, m_conf.L1Depth}, {
//            m_conf.Neuron1Width, m_conf.Neuron1Height, 1});
//    addLayer("ComplexCell", "none", {m_conf.L2XAnchor, m_conf.L2YAnchor, {0}}, {m_conf.L2Width, m_conf.L2Height, m_conf.L2Depth}, {
//            m_conf.Neuron2Width, m_conf.Neuron2Height, m_conf.Neuron2Depth});
//    addLayer("MotorCell", "none", {{0}, {0}, {0}}, {m_conf.L3Size, 1, 1},
//             {m_conf.L2Width, m_conf.L2Height, m_conf.L2Depth});

    addLayer("SimpleCell", m_conf.SharingType, {m_conf.L1XAnchor, m_conf.L1YAnchor, {0}}, {m_conf.L1Width, m_conf.L1Height, m_conf.L1Depth}, {
        m_conf.Neuron1Width, m_conf.Neuron1Height, 1});
    addLayer("MotorCell", "none", {{0}, {0}, {0}},
             {m_conf.L3Size, 1, 1}, {m_conf.L1Width, m_conf.L1Height, m_conf.L1Depth});

    if (m_conf.SaveData) {
        loadWeights();
    }

    for (size_t i = 0; i < m_neurons.size(); ++i) {
        std::cout << "Layer " << i << " neurons: " << m_neurons[i].size() << std::endl;
    }
}

void SpikingNetwork::runEvents(const std::vector<Event> &eventPacket, const double reward) {
    setReward(reward);

    for (Event event : eventPacket) {
        addEvent(event);

        if (static_cast<size_t>(m_iterations) % Conf::UPDATE_PARAMETER_FREQUENCY == 0) {
            updateNeuronsParameters(event.timestamp());
            std::cout << static_cast<size_t>(m_iterations) << " / " << eventPacket.size() << std::endl;
        }
        ++m_iterations;
    }
}

void SpikingNetwork::runEvent(const Event &event) {
    addEvent(event);

    if (static_cast<size_t>(m_iterations) % Conf::UPDATE_PARAMETER_FREQUENCY == 0) {
        updateNeuronsParameters(event.timestamp());
    }
    ++m_iterations;
}

inline void SpikingNetwork::addEvent(const Event &event) {
    for (size_t ind : m_pixelMapping[static_cast<uint32_t>(event.x()) * Conf::HEIGHT + static_cast<uint32_t>(event.y())]) {
        if (m_neurons[0][ind].get().newEvent(Event(event.timestamp(), event.x() - static_cast<int16_t>
        (m_neurons[0][ind].get().getOffset().posx()), event.y() - static_cast<int16_t>(m_neurons[0][ind].get().getOffset()
                .posy()), event.polarity(), event.camera()))) {
            for (auto &neuronToInhibit : m_neurons[0][ind].get().getInhibitionConnections()) {
                neuronToInhibit.get().inhibition();
            }
            if (m_neurons.size() > 1) {
                addNeuronEvent(m_neurons[0][ind].get());
            }
        }
    }
}

inline void SpikingNetwork::addNeuronEvent(const Neuron &neuron) {
    for (auto &nextNeuron : neuron.getOutConnections()) {
        if (nextNeuron.get().newEvent(NeuronEvent(neuron.getSpikingTime(),
                                                  static_cast<int32_t>(neuron.getPos().posx() - nextNeuron.get().getOffset().posx()),
                                                  static_cast<int32_t>(neuron.getPos().posy() - nextNeuron.get().getOffset().posy()),
                                                  static_cast<int32_t>(neuron.getPos().posz() - nextNeuron.get().getOffset().posz())),
                                      m_reward)) {
            for (auto &neuronToInhibit : nextNeuron.get().getInhibitionConnections()) {
                neuronToInhibit.get().inhibition();
            }
            addNeuronEvent(nextNeuron.get());
        }
    }
}

std::vector<uint64_t> SpikingNetwork::resolveMotor() {
    std::vector<uint64_t> motorActivations(m_neurons[m_neurons.size() - 1].size(), 0);
    for (auto &neuron : m_neurons[m_neurons.size() - 1]) {
        motorActivations[neuron.get().getIndex()] = neuron.get().getSpikeCount();
        neuron.get().resetSpikeCounter();
    }
    return motorActivations;
}

void SpikingNetwork::updateNeurons(const long time) {
    for (auto &simpleNeuron : m_simpleNeurons) {
        while (simpleNeuron.checkRemainingEvents(time)) {
            if (simpleNeuron.update()) {
                for (auto &simpleNeuronToInhibit : simpleNeuron.getInhibitionConnections()) {
                    simpleNeuronToInhibit.get().inhibition();
                }
                addNeuronEvent(simpleNeuron);
            }
        }
    }
}

void SpikingNetwork::generateWeightSharing(const std::string &neuronType, const std::vector<size_t> &neuronSizes) {
    auto nbSimpleNeurons = m_conf.L1XAnchor.size() * m_conf.L1YAnchor.size() * m_conf.L1Width * m_conf.L1Height * m_conf.L1Depth;
    auto nbComplexNeurons = m_conf.L2XAnchor.size() * m_conf.L2YAnchor.size() * m_conf.L2Width * m_conf.L2Height * m_conf.L2Depth;
    auto nbMotorNeurons = m_conf.L3Size;

    if (neuronType == "SimpleCell") {
        if (m_conf.SharingType == "none") {
            for (size_t i = 0; i < nbSimpleNeurons; ++i) {
                m_sharedWeightsSimple.push_back(Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(m_conf.NbCameras), static_cast<long>(m_conf.Neuron1Synapses),
                                                                          static_cast<long>(neuronSizes[0]), static_cast<long>(neuronSizes[1])));
            }
        }
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
                m_sharedWeightsSimple.push_back(Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(m_conf.NbCameras), static_cast<long>(m_conf.Neuron1Synapses),
                                                                          static_cast<long>(neuronSizes[0]), static_cast<long>(neuronSizes[1])));
            }
        }
    }
    if (neuronType == "ComplexCell") {
        for (size_t i = 0; i < nbComplexNeurons; ++i) {
            m_sharedWeightsComplex.push_back(Util::uniformMatrixComplex(static_cast<long>(neuronSizes[0]), static_cast<long>(neuronSizes[1]), static_cast<long>
            (neuronSizes[2])));
        }
    }
    if (neuronType == "MotorCell") {
        for (size_t i = 0; i < nbMotorNeurons; ++i) {
            m_sharedWeightsMotor.push_back(Util::uniformMatrixComplex(static_cast<long>(neuronSizes[0]), static_cast<long>(neuronSizes[1]), static_cast<long>
            (neuronSizes[2])));
        }
    }
}

void SpikingNetwork::addLayer(const std::string &neuronType, const std::string &sharingType,
                              const std::vector<std::vector<size_t>> &layerPatches,
                              const std::vector<size_t> &layerSizes,
                              const std::vector<size_t> &neuronSizes) {
    generateWeightSharing(neuronType, neuronSizes);
    size_t neuronIndex = 0;
    size_t weightIndex;
    size_t countWeightSharing = 0;
    m_layout.emplace_back();

    for (size_t x = 0; x < layerPatches[0].size(); ++x) {
        for (size_t y = 0; y < layerPatches[1].size(); ++y) {
            for (size_t z = 0; z < layerPatches[2].size(); ++z) {
                for (size_t i = 0; i < layerSizes[0]; ++i) {
                    for (size_t j = 0; j < layerSizes[1]; ++j) {
                        for (size_t k = 0; k < layerSizes[2]; ++k) {
                            if (sharingType == "none") {
                                weightIndex = neuronIndex;
                            } else if (sharingType == "full" || sharingType == "patch") {
                                weightIndex = countWeightSharing * layerSizes[2] + k;
                            }

                            auto pos = Position(x * layerSizes[0] + i, y * layerSizes[1] + j, z * layerSizes[2] + k);
                            auto offset = Position(layerPatches[0][x] + i * neuronSizes[0], layerPatches[1][y] + j * neuronSizes[1]);
                            if (neuronType == "SimpleCell") {
                                m_simpleNeurons.emplace_back(
                                        SimpleNeuron(neuronIndex, m_simpleNeuronConf, pos, offset, m_sharedWeightsSimple[weightIndex], m_conf.Neuron1Synapses));
                            } else if (neuronType == "ComplexCell") {
                                m_complexNeurons.emplace_back(ComplexNeuron(neuronIndex, m_complexNeuronConf, pos, offset, m_sharedWeightsComplex[neuronIndex]));
                            } else if (neuronType == "MotorCell") {
                                m_motorNeurons.emplace_back(MotorNeuron(neuronIndex, m_motorNeuronConf, pos, m_sharedWeightsMotor[neuronIndex]));
                            } else {
                                std::cout << "No matching cell type" << std::endl;
                            }
                            m_layout[m_neurons.size()][{pos.posx(), pos.posy(), pos.posz()}] = neuronIndex;
                            ++neuronIndex;
                        }
                    }
                }
                if (sharingType == "patch") {
                    ++countWeightSharing;
                }
            }
        }
    }

    if (neuronType == "SimpleCell") {
        m_neurons.emplace_back(m_simpleNeurons.begin(), m_simpleNeurons.end());
    } else if (neuronType == "ComplexCell") {
        m_neurons.emplace_back(m_complexNeurons.begin(), m_complexNeurons.end());
    } else if (neuronType == "MotorCell") {
        m_neurons.emplace_back(m_motorNeurons.begin(), m_motorNeurons.end());
    } else {
        std::cout << "No matching cell type" << std::endl;
    }

    connectLayer(true, layerSizes, neuronSizes);
}

void SpikingNetwork::connectLayer(const bool inhibition, const std::vector<size_t> &layerSizes, const std::vector<size_t> &neuronSizes) {
    size_t currLayer = m_neurons.size() - 1;
    size_t prevLayer;
    if (currLayer > 0) {
        prevLayer = m_neurons.size() - 2;
    }

    for (auto &neuron : m_neurons[currLayer]) {
        for (size_t i = neuron.get().getOffset().posx(); i < neuron.get().getOffset().posx() + neuronSizes[0]; ++i) {
            for (size_t j = neuron.get().getOffset().posy(); j < neuron.get().getOffset().posy() + neuronSizes[1]; ++j) {
                for (size_t k = neuron.get().getOffset().posz(); k < neuron.get().getOffset().posz() + neuronSizes[2]; ++k) {
                    if (currLayer == 0) {
                        m_pixelMapping[i * Conf::HEIGHT + j].push_back(neuron.get().getIndex());
                    } else {
                        neuron.get().addInConnection(m_neurons[prevLayer][m_layout[prevLayer][{i, j, k}]]);
                        m_neurons[prevLayer][m_layout[prevLayer][{i, j, k}]].get().addOutConnection(neuron.get());
                    }
                }
            }
        }
        if (inhibition) {
            for (size_t z = 0; z < layerSizes[2]; ++z) { // inhibition
                if (z != neuron.get().getPos().posz()) {
                    neuron.get().addInhibitionConnection(m_neurons[currLayer][m_layout[currLayer][{neuron.get().getPos().posx(), neuron.get().getPos().posy(), z}]]);
                }
            }
        }
    }
}

void SpikingNetwork::updateNeuronsParameters(const long time) {
    for (auto &neuron : m_neurons[0]) {
        neuron.get().thresholdAdaptation();
    }
}

void SpikingNetwork::saveNetwork(size_t nbRun, const std::string &eventFileName) {
    if (m_conf.SaveData) {
        std::cout << "Saving Network..." << std::endl;
        std::string fileName;
        fileName = m_conf.NetworkPath + "networkState";

        json state;
        state["event_file_name"] = eventFileName;
        state["nb_run"] = nbRun;
        state["rewards"] = m_listReward;

        std::ofstream ofs(fileName + ".json");
        if (ofs.is_open()) {
            ofs << std::setw(4) << state << std::endl;
        } else {
            std::cout << "cannot save network state file" << std::endl;
        }
        ofs.close();

        saveNeuronsStates();
    }
}

void SpikingNetwork::saveNeuronsStates() {
    size_t count, layer = 0;
    std::string fileName;

    for (auto &neurons : m_neurons) {
        count = 0;
        for (auto &neuron : neurons) {
            fileName = m_conf.NetworkPath + "weights/" + std::to_string(layer) + "/" + std::to_string(count);
            neuron.get().saveState(fileName);
            neuron.get().saveWeights(fileName);
            ++count;
        }
        ++layer;
    }
}

void SpikingNetwork::loadWeights() {
    std::string fileName;
    size_t count, layer = 0;

    for (auto &neurons : m_neurons) {
        count = 0;
        std::string path(m_conf.NetworkPath + "weights/" + std::to_string(layer) + "/0.npy");
        if (Util::fileExist(path)) {
            for (auto &neuron : neurons) {
                fileName = m_conf.NetworkPath + "weights/" + std::to_string(layer) + "/" + std::to_string(count);
                neuron.get().loadState(fileName);
                neuron.get().loadWeights(fileName);
                ++count;
            }
            std::cout << "Layer " << layer << ": weights loaded from file" << std::endl;
        } else {
            std::cout << "Layer " << layer << ": new weights generated" << std::endl;
        }
        ++layer;
    }

    std::vector<size_t> data(static_cast<size_t>(m_conf.L1XAnchor.size() * m_conf.L1Width * m_conf.L1YAnchor.size() * m_conf.L1Height * m_conf.L1Depth));
    count = 0;
    for (size_t i = 0; i < m_conf.L1XAnchor.size() * m_conf.L1Width; ++i) {
        for (size_t j = 0; j < m_conf.L1YAnchor.size() * m_conf.L1Height; ++j) {
            for (size_t k = 0; k < m_conf.L1Depth; ++k) {
                data[count] = m_layout[0][{i, j, k}];
                ++count;
            }
        }
    }
    cnpy::npy_save(m_conf.NetworkPath + "weights/layout1.npy", &data[0],
                   {m_conf.L1XAnchor.size() * m_conf.L1Width, m_conf.L1YAnchor.size() * m_conf.L1Height, m_conf.L1Depth}, "w");
}

void SpikingNetwork::trackNeuron(const long time, const size_t id, const size_t layer) {
    if (m_simpleNeuronConf.TRACKING == "partial") {
        if (!m_neurons[layer].empty()) {
            m_neurons[layer][id].get().trackPotential(time);
        }
    }
}

Position SpikingNetwork::findPixelComplexNeuron(ComplexNeuron &neuron) {
    size_t x = neuron.getOffset().posx();
    size_t y = neuron.getOffset().posy();
    size_t z = neuron.getOffset().posz();
    return {m_simpleNeurons[m_layout[0][{x, y, z}]].getOffset().posx(), m_simpleNeurons[m_layout[0][{x, y, z}]]
            .getOffset().posy()};
}

/***** GUI Interface *****/

cv::Mat SpikingNetwork::getWeightNeuron(size_t idNeuron, size_t camera, size_t synapse) {
    size_t sizeX = 0, sizeY = 0, sizePol = 0;
    sizeX = m_conf.Neuron1Width;
    sizeY = m_conf.Neuron1Height;
    sizePol = NBPOLARITY;

    cv::Mat weightImage = cv::Mat::zeros(static_cast<int>(sizeX), static_cast<int>(sizeY), CV_8UC3);
    if (!m_neurons[0].empty()) {
        double weight;
        for (size_t x = 0; x < sizeX; ++x) {
            for (size_t y = 0; y < sizeY; ++y) {
                for (size_t p = 0; p < sizePol; p++) {
                    weight = m_simpleNeurons[idNeuron].getWeights(p, camera, synapse, x, y) * 255;
                    if (weight > 255) { weight = 255; }
                    if (weight < 0) { weight = 0; }
                    weightImage.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x))[static_cast<int>(2 - p)] = static_cast<unsigned char>(weight);
                }
            }
        }
    }
    return weightImage;
}

Neuron &SpikingNetwork::getNeuron(size_t index, size_t layer) {
    if (layer < m_neurons.size()) {
        if (index < m_neurons[layer].size()) {
            return m_neurons[layer][index];
        }
    }
    throw std::runtime_error("Wrong layer or index for neuron selection");
}

void SpikingNetwork::setReward(double reward) {
    m_reward = reward;
    m_listReward.push_back(reward);
}

std::vector<size_t> SpikingNetwork::getNetworkStructure() {
    std::vector<size_t> structure;
    for (auto &m_neuron : m_neurons) {
        structure.push_back(m_neuron.size());
    }
    return structure;
}
