#include "SpikingNetwork.hpp"

#include <utility>

SpikingNetwork::SpikingNetwork(const std::string &conf) : m_conf(NetworkConfig(conf)),
                                                          m_simpleNeuronConf(m_conf.getNetworkPath() + "configs/simple_cell_config.json", 0),
                                                          m_complexNeuronConf(m_conf.getNetworkPath() + "configs/complex_cell_config.json", 1),
                                                          m_motorNeuronConf(m_conf.getNetworkPath() + "configs/motor_cell_config.json", 2),
                                                          m_pixelMapping(std::vector<std::vector<uint64_t>>
                                                                                 (Conf::WIDTH * Conf::HEIGHT, std::vector<uint64_t>(0))) {
    motorMapping.emplace_back(std::make_pair(0, -0.15)); // left horizontal -> left movement
//    motorMapping.emplace_back(std::make_pair(0, 0)); // left horizontal -> no movement
    motorMapping.emplace_back(std::make_pair(0, 0.15)); // left horizontal  -> right movement
}

void SpikingNetwork::runEvents(const std::vector<Event> &eventPacket, const double reward) {
    transmitReward(reward);

    for (Event event : eventPacket) {
        addEvent(event);

        if (static_cast<size_t>(m_iterations) % 10000 == 0) {
            updateNeuronsParameters(event.timestamp());
        }
        ++m_iterations;
    }
}

void SpikingNetwork::runEvent(const Event &event) {
    addEvent(event);

    if (static_cast<size_t>(m_iterations) % 10000 == 0) {
        updateNeuronsParameters(event.timestamp());
    }
    ++m_iterations;
}

inline void SpikingNetwork::addEvent(const Event &event) {
    for (size_t ind : m_pixelMapping[static_cast<uint32_t>(event.x()) * Conf::HEIGHT + static_cast<uint32_t>(event.y())]) {
        if (m_neurons[0][ind].get().newEvent(Event(event.timestamp(), event.x() - static_cast<int16_t>
        (m_neurons[0][ind].get().getOffset().x()), event.y() - static_cast<int16_t>(m_neurons[0][ind].get().getOffset()
                .y()), event.polarity(), event.camera()))) {
            m_neurons[0][ind].get().weightUpdate();
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
                                                  static_cast<int32_t>(neuron.getPos().x() -
                                                                       nextNeuron.get().getOffset().x()),
                                                  static_cast<int32_t>(neuron.getPos().y() -
                                                                       nextNeuron.get().getOffset().y()),
                                                  static_cast<int32_t>(neuron.getPos().z() -
                                                                       nextNeuron.get().getOffset().z())))) {
            if (nextNeuron.get().getLayer() > 1) {
                nextNeuron.get().setNeuromodulator(updateTDError());
            }
            if (nextNeuron.get().getLayer() != 3) {
                nextNeuron.get().weightUpdate();
            }

            for (auto &neuronToInhibit : nextNeuron.get().getInhibitionConnections()) {
                neuronToInhibit.get().inhibition();
            }
            addNeuronEvent(nextNeuron.get());
        }
    }
}

double SpikingNetwork::updateTDError() {
    double value = 0;
    double valueDerivative = 0;
    for (const auto &critic : m_neurons[2]) {
        auto values = critic.get().updateKernelSpikingRate();
        value += values.first;
        valueDerivative += values.second;
    }
    auto td_error = (m_conf.getNU() / static_cast<double>(m_neurons[2].size())) * (valueDerivative - (value / m_conf.getTAU_R())) - (m_conf.getV0() / m_conf.getTAU_R()) + m_reward;
    return td_error;
}

double SpikingNetwork::pushTDError(double time) {
    double value = 0;
    double valueDerivative = 0;
    for (const auto &critic : m_neurons[2]) {
        auto values = critic.get().updateKernelSpikingRate();
        value += values.first;
        valueDerivative += values.second;
    }
    auto td_error = (m_conf.getNU() / static_cast<double>(m_neurons[2].size())) * (valueDerivative - (value / m_conf.getTAU_R())) - (m_conf.getV0() / m_conf.getTAU_R()) + m_reward;

    m_listTD.push_back({ time, m_reward, m_conf.getNU() * value / static_cast<double>(m_neurons[2].size()) + m_conf.getV0(), m_conf.getNU() * valueDerivative / static_cast<double>(m_neurons[2].size()), td_error });
    return td_error;
}

//int SpikingNetwork::critic(size_t index) {
//    int reward;
//    auto action = motorMapping[index];
//    if (action.second < 0) {
//        if (m_reward < 0) {
//            reward = 1;
//        } else {
//            reward = -1;
//        }
//    } else if (action.second > 0) {
//        if (m_reward > 0) {
//            reward = 1;
//        } else {
//            reward = -1;
//        }
//    }
//    return reward;
//}

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

void SpikingNetwork::generateWeightSharing(const std::string &neuronType, const std::vector<size_t> &neuronSizes, const size_t nbNeurons) {
    if (neuronType == "SimpleCell") {
        if (m_conf.getSharingType() == "none") {
            for (size_t i = 0; i < nbNeurons; ++i) {
                m_sharedWeightsSimple.push_back(Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(m_conf.getNbCameras()), static_cast<long>(m_conf.getNeuron1Synapses()),
                                                                          static_cast<long>(neuronSizes[0]), static_cast<long>(neuronSizes[1])));
            }
        }
        size_t patch_size;
        if (m_conf.getSharingType() == "full") {
            patch_size = 1;
        } else if (m_conf.getSharingType() == "patch") {
            patch_size = m_conf.getLayerPatches()[0][0].size() * m_conf.getLayerPatches()[0][1].size();
        } else {
            patch_size = 0;
            std::cout << "Wrong type of sharing" << std::endl;
        }
        for (size_t patch = 0; patch < patch_size; ++patch) {
            for (size_t j = 0; j < m_conf.getLayerSizes()[0][2]; ++j) {
                m_sharedWeightsSimple.push_back(Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(m_conf.getNbCameras()), static_cast<long>(m_conf.getNeuron1Synapses()),
                                                                          static_cast<long>(neuronSizes[0]), static_cast<long>(neuronSizes[1])));
            }
        }
    }
    if (neuronType == "ComplexCell") {
        for (size_t i = 0; i < nbNeurons; ++i) {
            m_sharedWeightsComplex.push_back(Util::uniformMatrixComplex(static_cast<long>(neuronSizes[0]), static_cast<long>(neuronSizes[1]), static_cast<long>
            (neuronSizes[2])));
        }
    }
    if (neuronType == "CriticCell") {
        for (size_t i = 0; i < nbNeurons; ++i) {
            m_sharedWeightsCritic.push_back(Util::uniformMatrixComplex(static_cast<long>(neuronSizes[0]), static_cast<long>(neuronSizes[1]), static_cast<long>
            (neuronSizes[2])));
        }
    }
    if (neuronType == "ActorCell") {
        for (size_t i = 0; i < nbNeurons; ++i) {
            m_sharedWeightsActor.push_back(Util::uniformMatrixComplex(static_cast<long>(neuronSizes[0]), static_cast<long>(neuronSizes[1]), static_cast<long>
            (neuronSizes[2])));
        }
    }
}

void SpikingNetwork::addLayer(const std::string &neuronType, const std::string &sharingType, const bool inhibition,
                              const std::vector<std::vector<size_t>> &layerPatches,
                              const std::vector<size_t> &layerSizes,
                              const std::vector<size_t> &neuronSizes,
                              const size_t layerToConnect) {
    m_conf.getLayerPatches().push_back(layerPatches);
    m_conf.getLayerSizes().push_back(layerSizes);
    m_conf.getNeuronSizes().push_back(neuronSizes);

    auto nbNeurons = layerPatches[0].size() * layerSizes[0] * layerPatches[1].size() * layerSizes[1] * layerPatches[2].size() * layerSizes[2];
    generateWeightSharing(neuronType, neuronSizes, nbNeurons);

    size_t neuronIndex = 0;
    size_t weightIndex;
    size_t countWeightSharing = 0;
    m_layout.emplace_back();
    auto layer = m_neurons.size();

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
                                m_simpleNeurons.emplace_back(SimpleNeuron(neuronIndex, layer, m_simpleNeuronConf, pos, offset, m_sharedWeightsSimple[weightIndex],
                                                                          m_conf.getNeuron1Synapses()));
                            } else if (neuronType == "ComplexCell") {
                                m_complexNeurons.emplace_back(ComplexNeuron(neuronIndex, layer, m_complexNeuronConf, pos, offset, m_sharedWeightsComplex[neuronIndex]));
                            } else if (neuronType == "CriticCell") {
                                m_criticNeurons.emplace_back(MotorNeuron(neuronIndex, layer, m_motorNeuronConf, pos, m_sharedWeightsCritic[neuronIndex]));
                            } else if (neuronType == "ActorCell") {
                                m_actorNeurons.emplace_back(MotorNeuron(neuronIndex, layer, m_motorNeuronConf, pos, m_sharedWeightsActor[neuronIndex]));
                            } else {
                                std::cout << "No matching cell type" << std::endl;
                            }
                            m_layout[layer][{ pos.x(), pos.y(), pos.z() }] = neuronIndex;
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
    } else if (neuronType == "CriticCell") {
        m_neurons.emplace_back(m_criticNeurons.begin(), m_criticNeurons.end());
    } else if (neuronType == "ActorCell") {
        m_neurons.emplace_back(m_actorNeurons.begin(), m_actorNeurons.end());
    } else {
        std::cout << "No matching cell type" << std::endl;
    }

    connectLayer(inhibition, layerToConnect, layerSizes, neuronSizes);
}

void SpikingNetwork::connectLayer(const bool inhibition, const size_t layerToConnect, const std::vector<size_t> &layerSizes, const std::vector<size_t> &neuronSizes) {
    auto currLayer = m_neurons.size() - 1;
    for (auto &neuron : m_neurons[currLayer]) {
        for (size_t i = neuron.get().getOffset().x(); i < neuron.get().getOffset().x() + neuronSizes[0]; ++i) {
            for (size_t j = neuron.get().getOffset().y(); j < neuron.get().getOffset().y() + neuronSizes[1]; ++j) {
                for (size_t k = neuron.get().getOffset().z(); k < neuron.get().getOffset().z() + neuronSizes[2]; ++k) {
                    if (currLayer == 0) {
                        m_pixelMapping[i * Conf::HEIGHT + j].push_back(neuron.get().getIndex());
                    } else {
                        neuron.get().addInConnection(m_neurons[layerToConnect][m_layout[layerToConnect][{i, j, k}]]);
                        m_neurons[layerToConnect][m_layout[layerToConnect][{i, j, k}]].get().addOutConnection(neuron.get());
                    }
                }
            }
        }
        if (inhibition) {
            for (size_t z = 0; z < layerSizes[2]; ++z) { // inhibition
                if (z != neuron.get().getPos().z()) {
                    neuron.get().addInhibitionConnection(m_neurons[currLayer][m_layout[currLayer][{
                            neuron.get().getPos().x(), neuron.get().getPos().y(), z}]]);
                }
            }
        }
    }
}

void SpikingNetwork::updateNeuronsParameters(const long time) {
    size_t layer;
    for (auto &neurons : m_neurons) {
        for (auto &neuron : neurons) {
            if (layer == 0) {
//                neuron.get().thresholdAdaptation();
            }
            neuron.get().updateState(time);
        }
    }
}

void SpikingNetwork::saveNetwork(size_t nbRun, const std::string &eventFileName) {
    if (m_conf.getSaveData()) {
        std::cout << "Saving Network..." << std::endl;
        std::string fileName;
        fileName = m_conf.getNetworkPath() + "networkState";

        json state;
        state["event_file_name"] = eventFileName;
        state["nb_run"] = nbRun;
        state["rewards"] = m_listReward;
        state["bias"] = m_bias;
        state["reward_iter"] = m_rewardIter;

        state["td"] = m_listTD;

        std::ofstream ofs(fileName + ".json");
        if (ofs.is_open()) {
            ofs << std::setw(4) << state << std::endl;
        } else {
            std::cout << "cannot save network state file" << std::endl;
        }
        ofs.close();

        saveNeuronsStates();
    }

    size_t count;
    for (size_t layer = 0; layer < m_neurons.size(); ++layer) {
        std::vector<size_t> data(static_cast<size_t>(m_conf.getLayerPatches()[layer][0].size() * m_conf.getLayerSizes()[layer][0] * m_conf.getLayerPatches()[layer][1].size() * m_conf.getLayerSizes()[layer][1] * m_conf.getLayerPatches()[layer][2].size() * m_conf.getLayerSizes()[layer][2]));
        count = 0;
        for (size_t i = 0; i < m_conf.getLayerPatches()[layer][0].size() * m_conf.getLayerSizes()[layer][0]; ++i) {
            for (size_t j = 0; j < m_conf.getLayerPatches()[layer][1].size() * m_conf.getLayerSizes()[layer][1]; ++j) {
                for (size_t k = 0; k < m_conf.getLayerPatches()[layer][2].size() * m_conf.getLayerSizes()[layer][2]; ++k) {
                    data[count] = m_layout[layer][{i, j, k}];
                    ++count;
                }
            }
        }
        cnpy::npy_save(m_conf.getNetworkPath() + "weights/layout_" + std::to_string(layer) + ".npy", &data[0],
                       {m_conf.getLayerPatches()[layer][0].size() * m_conf.getLayerSizes()[layer][0], m_conf.getLayerPatches()[layer][1].size() * m_conf.getLayerSizes()[layer][1], m_conf.getLayerPatches()[layer][2].size() * m_conf.getLayerSizes()[layer][2]}, "w");
    }
}

void SpikingNetwork::loadNetwork() {
    std::string fileName;
    fileName = m_conf.getNetworkPath() + "networkState";

    json state;
    std::ifstream ifs(fileName + ".json");
    if (ifs.is_open()) {
        try {
            ifs >> state;
        } catch (const std::exception& e) {
            std::cerr << "In Network state file: " << fileName + ".json" << std::endl;
            throw;
        }
        m_bias = state["bias"];
        m_rewardIter = state["reward_iter"];
        for (auto &reward : state["rewards"]) {
            m_listReward.push_back(reward);
        }
    } else {
        std::cout << "Cannot open network state file" << std::endl;
    }
    ifs.close();

    loadWeights();
}

void SpikingNetwork::saveNeuronsStates() {
    size_t count, layer = 0;
    std::string fileName;

    for (auto &neurons : m_neurons) {
        count = 0;
        for (auto &neuron : neurons) {
            fileName = m_conf.getNetworkPath() + "weights/" + std::to_string(layer) + "/" + std::to_string(count);
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
        std::string path(m_conf.getNetworkPath() + "weights/" + std::to_string(layer) + "/0.npy");
        if (Util::fileExist(path)) {
            for (auto &neuron : neurons) {
                fileName = m_conf.getNetworkPath() + "weights/" + std::to_string(layer) + "/" + std::to_string(count);
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
}

void SpikingNetwork::trackNeuron(const long time, const size_t id, const size_t layer) {
    if (m_simpleNeuronConf.TRACKING == "partial") {
        if (!m_neurons[layer].empty()) {
            m_neurons[layer][id].get().trackPotential(time);
        }
    }
}

/***** GUI Interface *****/

cv::Mat SpikingNetwork::getWeightNeuron(size_t idNeuron, size_t layer, size_t camera, size_t synapse, size_t z) {
    if (!m_neurons[layer].empty()) {
        auto dim = m_neurons[layer][0].get().getWeightsDimension();
        cv::Mat weightImage = cv::Mat::zeros(static_cast<int>(dim[1]), static_cast<int>(dim[0]), CV_8UC3);

        double weight;
        for (size_t x = 0; x < dim[0]; ++x) {
            for (size_t y = 0; y < dim[1]; ++y) {
                if (layer == 0) {
                    for (size_t p = 0; p < NBPOLARITY; p++) {
                        weight = m_neurons[layer][idNeuron].get().getWeights(p, camera, synapse, x, y) * 255;
                        if (weight > 255) { weight = 255; }
                        if (weight < 0) { weight = 0; }
                        weightImage.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x))[static_cast<int>(2 - p)] = static_cast<unsigned char>(weight);
                    }
                } else {
                    weight = m_neurons[layer][idNeuron].get().getWeights(x, y, z) * 255;
                    if (weight > 255) { weight = 255; }
                    if (weight < 0) { weight = 0; }
                    weightImage.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x))[0] = static_cast<unsigned char>(weight);
                }
            }
        }
        return weightImage;
    }
    return cv::Mat::zeros(0, 0, CV_8UC3);
}

Neuron &SpikingNetwork::getNeuron(size_t index, size_t layer) {
    if (layer < m_neurons.size()) {
        if (index < m_neurons[layer].size()) {
            return m_neurons[layer][index];
        }
    }
    throw std::runtime_error("Wrong layer or index for neuron selection");
}

void SpikingNetwork::transmitReward(double reward) {
    m_reward = reward;
    ++m_rewardIter;
    m_bias += ((reward - m_bias) / static_cast<double>(m_rewardIter)); // Average reward
    m_listReward.push_back(reward);
}

std::vector<size_t> SpikingNetwork::getNetworkStructure() {
    std::vector<size_t> structure;
    for (auto &m_neuron : m_neurons) {
        structure.push_back(m_neuron.size());
    }
    return structure;
}
