#include "SpikingNetwork.hpp"

SpikingNetwork::SpikingNetwork() = default;

/* Creates a spiking neural network.
 * It loads the configuration files from the specified networkPath.
 * Constructs the neuron connections and initializes them with random weights.
 */
SpikingNetwork::SpikingNetwork(const std::string &networkPath) : m_networkConf(NetworkConfig(networkPath)),
                                                                 m_simpleNeuronConf(m_networkConf.getNetworkPath() +
                                                                                    "configs/simple_cell_config.json",
                                                                                    0),
                                                                 m_complexNeuronConf(m_networkConf.getNetworkPath() +
                                                                                     "configs/complex_cell_config.json",
                                                                                     1),
                                                                 m_criticNeuronConf(m_networkConf.getNetworkPath() +
                                                                                    "configs/critic_cell_config.json",
                                                                                    2),
                                                                 m_actorNeuronConf(m_networkConf.getNetworkPath() +
                                                                                   "configs/actor_cell_config.json", 3),
                                                                 m_pixelMapping(std::vector<std::vector<uint64_t>>(
                                                                         Conf::WIDTH * Conf::HEIGHT,
                                                                         std::vector<uint64_t>(0))) {
    for (size_t i = 0; i < m_networkConf.getLayerCellTypes().size(); ++i) {
        addLayer(m_networkConf.getLayerCellTypes()[i], m_networkConf.getSharingType(),
                 m_networkConf.getLayerInhibitions()[i],
                 m_networkConf.getLayerPatches()[i], m_networkConf.getLayerSizes()[i],
                 m_networkConf.getNeuronSizes()[i],
                 m_networkConf.getNeuronOverlap()[i], m_networkConf.getInterLayerConnections()[i]);
    }
}

/* Iterate the network on the event, updating every 1st layer neuron connected to the subsequent pixel.
 * Determines which neuron to update depending on a mapping between pixels and neurons.
 * If a neuron exceeds a threshold, it spikes and transmit another event towards deeper neurons.
 * A neuron spikes activates inhibition connections to adjacent neurons.
 */
void SpikingNetwork::addEvent(const Event &event) {
    for (size_t ind: m_pixelMapping[static_cast<uint32_t>(event.x()) * Conf::HEIGHT +
                                    static_cast<uint32_t>(event.y())]) {
        auto eventPos = Position(event.x() - static_cast<int16_t>(m_neurons[0][ind].get().getOffset().x()),
                                 event.y() - static_cast<int16_t>(m_neurons[0][ind].get().getOffset().y()));
        if (m_neurons[0][ind].get().newEvent(
                Event(event.timestamp(), eventPos.x(), eventPos.y(), event.polarity(), event.camera()))) {
            m_neurons[0][ind].get().weightUpdate();
            lateralStaticInhibition(m_neurons[0][ind].get());
            lateralDynamicInhibition(m_neurons[0][ind].get());
            if (m_neurons.size() > 1) {
                addNeuronEvent(m_neurons[0][ind].get());
            }
        }
    }
}

/* Recursive function that updates neurons deeper than the 1st layer. Similarly to the addEvent function, if a neuron spikes,
 * it transmits a new event forward.
 * the neuron to which the event is transmitted are determined from vector of reference called outConnections.
 * A neuron spikes activates inhibition connections to adjacent neurons.
 */
inline void SpikingNetwork::addNeuronEvent(const Neuron &neuron) {
//    auto stack = std::stack<std::vector<std::reference_wrapper<Neuron>>>();
//    stack.push(neuron.getOutConnections());
//    while (!stack.empty()) {
//        auto connections = stack.top();
//        stack.pop();
//        for (auto &forwardNeuron: connections) {
//            auto neuronPos = Position(neuron.getPos().x() - forwardNeuron.get().getOffset().x(),
//                                      neuron.getPos().y() - forwardNeuron.get().getOffset().y(),
//                                      neuron.getPos().z() - forwardNeuron.get().getOffset().z());
//            if (forwardNeuron.get().newEvent(
//                    NeuronEvent(neuron.getSpikingTime(), neuronPos.x(), neuronPos.y(), neuronPos.z()))) {
//                if (forwardNeuron.get().getLayer() != 3) { // weight change
//                    forwardNeuron.get().weightUpdate();
//                }
//                lateralStaticInhibition(forwardNeuron.get());
//                topDownDynamicInhibition(forwardNeuron.get());
//                neuromodulation(forwardNeuron.get());
//                if (!forwardNeuron.get().getOutConnections().empty()) {
//                    stack.push(forwardNeuron.get().getOutConnections());
//                }
//            }
//        }
//    }

    for (auto &forwardNeuron: neuron.getOutConnections()) {
        auto neuronPos = Position(neuron.getPos().x() - forwardNeuron.get().getOffset().x(),
                                  neuron.getPos().y() - forwardNeuron.get().getOffset().y(),
                                  neuron.getPos().z() - forwardNeuron.get().getOffset().z());
        if (forwardNeuron.get().newEvent(NeuronEvent(neuron.getSpikingTime(), neuronPos.x(), neuronPos.y(), neuronPos.z()))) {
            if (forwardNeuron.get().getLayer() != 3) { // weight change
                forwardNeuron.get().weightUpdate();
            }
            lateralStaticInhibition(forwardNeuron.get());
            topDownDynamicInhibition(forwardNeuron.get());
            neuromodulation(forwardNeuron.get());

            addNeuronEvent(forwardNeuron.get());
        }
    }
}

void SpikingNetwork::topDownDynamicInhibition(Neuron &neuron) {
    if (neuron.getLayer() == 1) {
        for (auto &backwardNeuron: neuron.getInConnections()) {
            auto event = NeuronEvent(neuron.getSpikingTime(), neuron.getIndex());
            backwardNeuron.get().newTopDownInhibitoryEvent(event);
        }
    }
}

void SpikingNetwork::lateralDynamicInhibition(Neuron &neuron) {
    for (auto &lateralNeuron: neuron.getlateralDynamicInhibitionConnections()) {
        auto event = NeuronEvent(neuron.getSpikingTime(), neuron.getIndex());
        lateralNeuron.get().newLateralInhibitoryEvent(event);
    }
}

void SpikingNetwork::lateralStaticInhibition(Neuron &neuron) {
    for (auto &lateralNeuron: neuron.getlateralStaticInhibitionConnections()) {
        lateralNeuron.get().inhibition();
    }
}

void SpikingNetwork::neuromodulation(Neuron &neuron) {
    if (neuron.getLayer() == 2) {
        neuron.setNeuromodulator(computeNeuromodulator(static_cast<double>(neuron.getSpikingTime())));
    }
}

void SpikingNetwork::transmitNeuromodulator(double neuromodulator) {
    m_neuromodulator = neuromodulator;
}

inline double SpikingNetwork::computeNeuromodulator(double time) {
    double value = 0;
    for (const auto &critic: m_neurons[2]) {
        value += critic.get().updateKernelSpikingRate(time);
    }
    auto V = m_networkConf.getNu() * value / static_cast<double>(m_neurons[2].size()) + m_networkConf.getV0();
    return -V / m_networkConf.getTauR() + m_neuromodulator;
}

void SpikingNetwork::updateNeurons(const long time) {
    for (auto &simpleNeuron: m_simpleNeurons) {
        while (simpleNeuron.checkRemainingEvents(time)) {
            if (simpleNeuron.update()) {
                for (auto &simpleNeuronToInhibit: simpleNeuron.getlateralStaticInhibitionConnections()) {
                    simpleNeuronToInhibit.get().inhibition();
                }
                addNeuronEvent(simpleNeuron);
            }
        }
    }
}

void SpikingNetwork::generateWeightSharing(const std::string &neuronType, const std::vector<size_t> &neuronSizes,
                                           const size_t nbNeurons) {
    long x = static_cast<long>(neuronSizes[0]);
    long y = static_cast<long>(neuronSizes[1]);
    long z = static_cast<long>(neuronSizes[2]);
    if (neuronType == "SimpleCell") {
        if (m_networkConf.getSharingType() == "none") {
            for (size_t i = 0; i < nbNeurons; ++i) {
                m_sharedWeightsSimple.push_back(
                        Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(m_networkConf.getNbCameras()),
                                                  static_cast<long>(m_networkConf.getNeuron1Synapses()), x, y));
            }
        } else if (m_networkConf.getSharingType() == "patch") {
            size_t patch_size =
                    m_networkConf.getLayerPatches()[0][0].size() * m_networkConf.getLayerPatches()[0][1].size();
            for (size_t patch = 0; patch < patch_size; ++patch) {
                for (size_t i = 0; i < m_networkConf.getLayerSizes()[0][2]; ++i) {
                    m_sharedWeightsSimple.push_back(
                            Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(m_networkConf.getNbCameras()),
                                                      static_cast<long>(m_networkConf.getNeuron1Synapses()), x, y));
                }
            }
        } else {
            std::cout << "Wrong type of sharing" << std::endl;
        }
    }
    if (neuronType == "ComplexCell") {
        for (size_t i = 0; i < nbNeurons; ++i) {
            m_sharedWeightsComplex.push_back(Util::uniformMatrixComplex(x, y, z));
        }
    }
    if (neuronType == "CriticCell") {
        for (size_t i = 0; i < nbNeurons; ++i) {
            m_sharedWeightsCritic.push_back(Util::uniformMatrixComplex(x, y, z));
        }
    }
    if (neuronType == "ActorCell") {
        for (size_t i = 0; i < nbNeurons; ++i) {
            m_sharedWeightsActor.push_back(Util::uniformMatrixComplex(x, y, z));
        }
    }
}

void SpikingNetwork::addLayer(const std::string &neuronType, const std::string &sharingType, const bool inhibition,
                              const std::vector<std::vector<size_t>> &layerPatches,
                              const std::vector<size_t> &layerSizes,
                              const std::vector<size_t> &neuronSizes,
                              const std::vector<size_t> &neuronOverlap,
                              const size_t layerToConnect) {
    auto nbNeurons =
            layerPatches[0].size() * layerSizes[0] * layerPatches[1].size() * layerSizes[1] * layerPatches[2].size() *
            layerSizes[2];
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
                            } else if (sharingType == "patch") {
                                weightIndex = countWeightSharing * layerSizes[2] + k;
                            }

                            // Position of the neuron in the neuronal layer space (x, y, z)
                            auto pos = Position(x * layerSizes[0] + i, y * layerSizes[1] + j, z * layerSizes[2] + k);

                            // Receptive field starting position (x, y) compared to the previous layer
                            auto offset = Position(layerPatches[0][x] + i * (neuronSizes[0] - neuronOverlap[0]),
                                                   layerPatches[1][y] + j * (neuronSizes[1] - neuronOverlap[1]));

                            if (neuronType == "SimpleCell") {
                                m_simpleNeurons.emplace_back(
                                        SimpleNeuron(neuronIndex, layer, m_simpleNeuronConf, pos, offset,
                                                     m_sharedWeightsSimple[weightIndex],
                                                     m_networkConf.getNeuron1Synapses()));
                            } else if (neuronType == "ComplexCell") {
                                m_complexNeurons.emplace_back(
                                        ComplexNeuron(neuronIndex, layer, m_complexNeuronConf, pos, offset,
                                                      m_sharedWeightsComplex[neuronIndex]));
                            } else if (neuronType == "CriticCell") {
                                m_criticNeurons.emplace_back(
                                        MotorNeuron(neuronIndex, layer, m_criticNeuronConf, pos,
                                                    m_sharedWeightsCritic[neuronIndex]));
                            } else if (neuronType == "ActorCell") {
                                m_actorNeurons.emplace_back(
                                        MotorNeuron(neuronIndex, layer, m_actorNeuronConf, pos,
                                                    m_sharedWeightsActor[neuronIndex]));
                            } else {
                                std::cout << "No matching cell type" << std::endl;
                            }
                            m_layout[layer][{pos.x(), pos.y(), pos.z()}] = neuronIndex;
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
    m_structure.push_back(m_neurons[layer].size());
    connectLayer(inhibition, layerToConnect, layerPatches, layerSizes, neuronSizes);
}

void SpikingNetwork::connectLayer(const bool inhibition, const size_t layerToConnect,
                                  const std::vector<std::vector<size_t>> &layerPatches,
                                  const std::vector<size_t> &layerSizes, const std::vector<size_t> &neuronSizes) {
    auto currLayer = m_neurons.size() - 1;
    for (auto &neuron: m_neurons[currLayer]) {
        topDownConnection(neuron.get(), currLayer, layerToConnect, neuronSizes);
        if (inhibition) {
            lateralStaticInhibitionConnection(neuron.get(), currLayer, layerSizes);
            lateralDynamicInhibitionConnection(neuron.get(), currLayer, layerPatches, layerSizes);
        }
    }
}

void SpikingNetwork::topDownConnection(Neuron &neuron, const size_t currLayer, const size_t layerToConnect,
                                       const std::vector<size_t> &neuronSizes) {
    for (size_t i = neuron.getOffset().x(); i < neuron.getOffset().x() + neuronSizes[0]; ++i) {
        for (size_t j = neuron.getOffset().y(); j < neuron.getOffset().y() + neuronSizes[1]; ++j) {
            for (size_t k = neuron.getOffset().z(); k < neuron.getOffset().z() + neuronSizes[2]; ++k) {
                if (currLayer == 0) {
                    m_pixelMapping[i * Conf::HEIGHT + j].push_back(neuron.getIndex());
                } else {
                    neuron.addInConnection(m_neurons[layerToConnect][m_layout[layerToConnect][{i, j, k}]]);
                    m_neurons[layerToConnect][m_layout[layerToConnect][{i, j, k}]].get().addOutConnection(neuron);
                }
            }
        }
    }
}

void SpikingNetwork::lateralDynamicInhibitionConnection(Neuron &neuron, const size_t currLayer,
                                                        const std::vector<std::vector<size_t>> &layerPatches,
                                                        const std::vector<size_t> &layerSizes) {
    for (int x = static_cast<int>(neuron.getPos().x()) - 1; x < static_cast<int>(neuron.getPos().x()) + 2; ++x) {
        for (int y = static_cast<int>(neuron.getPos().y()) - 1; y < static_cast<int>(neuron.getPos().y()) + 2; ++y) {
            for (size_t z = 0; z < layerSizes[2]; ++z) {
                if ((x != neuron.getPos().x() || y != neuron.getPos().y()) && x >= 0 && y >= 0 &&
                    x < layerPatches[0].size() * layerSizes[0] &&
                    y < layerPatches[1].size() * layerSizes[1]) {
                    neuron.addLateralDynamicInhibitionConnections(
                            m_neurons[currLayer][m_layout[currLayer][{x, y, z}]]);
                }
            }
        }
    }
}

void SpikingNetwork::lateralStaticInhibitionConnection(Neuron &neuron, const size_t currLayer,
                                                       const std::vector<size_t> &layerSizes) {
    for (size_t z = 0; z < layerSizes[2]; ++z) {
        if (z != neuron.getPos().z()) {
            neuron.addLateralStaticInhibitionConnections(m_neurons[currLayer][m_layout[currLayer][{
                    neuron.getPos().x(), neuron.getPos().y(), z}]]);
        }
    }
}

/* Updates some neurons internal state.
 * Computes each neurons average spiking rate.
 * Adapts the threshold of the 1st layer's neurons.
 */
void SpikingNetwork::updateNeuronsStates(long timeInterval, size_t nbEvents) {
    size_t layer;
    m_averageActivity = 0;
    for (auto &neurons: m_neurons) {
        for (auto &neuron: neurons) {
            neuron.get().updateState(timeInterval, 0.5);

            neuron.get().learningDecay(static_cast<double>(nbEvents) / E6);
            if (layer == 0) {
                neuron.get().thresholdAdaptation();
                m_averageActivity += neuron.get().getSpikingRate();
            }
        }
    }
    m_averageActivity /= static_cast<double>(m_neurons[0].size());
    std::cout << m_averageActivity << std::endl;
}

void SpikingNetwork::normalizeActions() {
    auto layer = m_neurons.size() - 1;
    auto norms = std::vector<double>(getNetworkStructure().back(), 0);

    double normMax = 0;
    size_t count = 0;
    for (auto &neuron: m_neurons[layer]) {
        norms[count] = neuron.get().computeNormWeights();
        if (norms[count] > normMax) {
            normMax = norms[count];
        }
        ++count;
    }

    count = 0;
    for (auto &neuron: m_neurons[layer]) {
        if (normMax != norms[count]) {
            neuron.get().rescaleWeights(normMax / norms[count]);
        }
        ++count;
    }
}

void SpikingNetwork::saveNetwork() {
    saveNeuronsStates();

    size_t count;
    for (size_t layer = 0; layer < m_neurons.size(); ++layer) {
        std::vector<size_t> data(
                static_cast<size_t>(m_networkConf.getLayerPatches()[layer][0].size() *
                                    m_networkConf.getLayerSizes()[layer][0] *
                                    m_networkConf.getLayerPatches()[layer][1].size() *
                                    m_networkConf.getLayerSizes()[layer][1] *
                                    m_networkConf.getLayerPatches()[layer][2].size() *
                                    m_networkConf.getLayerSizes()[layer][2]));
        count = 0;
        for (size_t i = 0;
             i < m_networkConf.getLayerPatches()[layer][0].size() * m_networkConf.getLayerSizes()[layer][0]; ++i) {
            for (size_t j = 0;
                 j < m_networkConf.getLayerPatches()[layer][1].size() * m_networkConf.getLayerSizes()[layer][1]; ++j) {
                for (size_t k = 0;
                     k <
                     m_networkConf.getLayerPatches()[layer][2].size() * m_networkConf.getLayerSizes()[layer][2]; ++k) {
                    data[count] = m_layout[layer][{i, j, k}];
                    ++count;
                }
            }
        }
        cnpy::npy_save(m_networkConf.getNetworkPath() + "weights/layout_" + std::to_string(layer) + ".npy", &data[0],
                       {m_networkConf.getLayerPatches()[layer][0].size() * m_networkConf.getLayerSizes()[layer][0],
                        m_networkConf.getLayerPatches()[layer][1].size() * m_networkConf.getLayerSizes()[layer][1],
                        m_networkConf.getLayerPatches()[layer][2].size() * m_networkConf.getLayerSizes()[layer][2]},
                       "w");
    }
}

void SpikingNetwork::intermediateSave(size_t saveCount) {
    std::string fileName;
    std::filesystem::create_directory(
            m_networkConf.getNetworkPath() + "weights/intermediate_" + std::to_string(saveCount) + "/");
    std::filesystem::create_directory(
            m_networkConf.getNetworkPath() + "weights/intermediate_" + std::to_string(saveCount) + "/0/");
    std::filesystem::create_directory(
            m_networkConf.getNetworkPath() + "weights/intermediate_" + std::to_string(saveCount) + "/1/");

    size_t layer = 0;
    for (auto &neurons: m_neurons) {
        for (size_t i = 0; i < m_networkConf.getLayerSizes()[layer][2]; ++i) {
            fileName = m_networkConf.getNetworkPath() + "weights/intermediate_" + std::to_string(saveCount) + "/" +
                       std::to_string(layer) + "/";
            neurons[i].get().saveWeights(fileName);
        }
        ++layer;
    }
}

void SpikingNetwork::saveNeuronsStates() {
    size_t layer = 0;
    std::string fileName;

    for (auto &neurons: m_neurons) {
        if (layer == 0 && m_networkConf.getSharingType() == "patch") {
            size_t step = m_networkConf.getLayerSizes()[0][0] * m_networkConf.getLayerSizes()[0][1] *
                          m_networkConf.getLayerSizes()[0][2];
            size_t patch_size =
                    m_networkConf.getLayerPatches()[0][0].size() * m_networkConf.getLayerPatches()[0][1].size();
            for (size_t patch = 0; patch < patch_size; ++patch) {
                for (size_t i = 0; i < m_networkConf.getLayerSizes()[0][2]; ++i) {
                    fileName = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
                    neurons[patch * step + i].get().saveWeights(fileName);
                }
            }
        }
        for (auto &neuron: neurons) {
            fileName = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
            neuron.get().saveState(fileName);
            neuron.get().saveInhibWeights(fileName);
            if (layer != 0 || m_networkConf.getSharingType() != "patch") {
                neuron.get().saveWeights(fileName);
            }
        }
        ++layer;
    }
}

void SpikingNetwork::loadWeights() {
    std::string fileName;
    size_t layer = 0;

    for (auto &neurons: m_neurons) {
        std::string path(m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/0.npy");
        if (Util::fileExist(path)) {
            if (layer == 0 && m_networkConf.getSharingType() == "patch") {
                size_t step = m_networkConf.getLayerSizes()[0][0] * m_networkConf.getLayerSizes()[0][1] *
                              m_networkConf.getLayerSizes()[0][2];
                size_t patch_size =
                        m_networkConf.getLayerPatches()[0][0].size() * m_networkConf.getLayerPatches()[0][1].size();
                for (size_t patch = 0; patch < patch_size; ++patch) {
                    for (size_t i = 0; i < m_networkConf.getLayerSizes()[0][2]; ++i) {
                        fileName = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
                        neurons[patch * step + i].get().loadWeights(fileName);
                    }
                }
            }
            for (auto &neuron: neurons) {
                fileName = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
                neuron.get().loadState(fileName);
                neuron.get().loadInhibWeights(fileName);
                if (layer != 0 || m_networkConf.getSharingType() != "patch") {
                    neuron.get().loadWeights(fileName);
                }
            }
            std::cout << "Layer " << layer << ": weights loaded from file" << std::endl;
        } else {
            std::cout << "Layer " << layer << ": new weights generated" << std::endl;
        }
        ++layer;
    }
}

std::reference_wrapper<Neuron> &SpikingNetwork::getNeuron(const size_t index, const size_t layer) {
    if (layer < m_neurons.size()) {
        if (index < m_neurons[layer].size()) {
            return m_neurons[layer][index];
        }
    }
    throw std::runtime_error("Wrong layer or index for neuron selection");
}
