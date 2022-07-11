//
// Created by Thomas on 14/04/2021.
//

#include "SpikingNetwork.hpp"

SpikingNetwork::SpikingNetwork() = default;

/**
 * @brief Creates a spiking neural network.
 * It loads the configuration files from the specified networkPath.
 * Constructs the neuron connections and initializes them with random weights.
 * @param networkPath - Path to the network folder.
 */
SpikingNetwork::SpikingNetwork(const std::string &networkPath) : m_networkConf(NetworkConfig(networkPath + "configs/network_config.json")),
                                                                 m_simpleNeuronConf(networkPath + "configs/simple_cell_config.json", 0),
                                                                 m_complexNeuronConf(networkPath + "configs/complex_cell_config.json", 1),
                                                                 m_criticNeuronConf(networkPath + "configs/critic_cell_config.json", 2),
                                                                 m_actorNeuronConf(networkPath + "configs/actor_cell_config.json", 3),
                                                                 m_pixelMapping(std::vector<std::vector<uint64_t>>(
                                                                         m_networkConf.getVfWidth() * m_networkConf.getVfHeight(),
                                                                         std::vector<uint64_t>(0))) {
    for (size_t i = 0; i < m_networkConf.getLayerConnectivity().size(); ++i) {
        addLayer(m_networkConf.getSharingType(), m_networkConf.getLayerConnectivity()[i]);
    }
}

/**
 * @brief Iterate the network on the event, updating every 1st layer neuron connected to the subsequent pixel.
 * Determines which neuron to update depending on a mapping between pixels and neurons.
 * If a neuron exceeds a threshold, it spikes and transmit another event towards deeper neurons.
 * A neuron spikes activates newStaticInhibitoryEvent connections to adjacent neurons.
 * @param event - The event coming from the pixel array.
 */
void SpikingNetwork::addEvent(const Event &event) {
    for (size_t ind: m_pixelMapping[static_cast<uint32_t>(event.x()) * 260 + static_cast<uint32_t>(event.y())]) { // TODO: optim
        auto eventPos = Position(event.x() - static_cast<int16_t>(m_neurons[0][ind].get().getOffset().x()),
                                 event.y() - static_cast<int16_t>(m_neurons[0][ind].get().getOffset().y()));
        if (m_networkConf.getNeuron1Synapses() == 1) {
            bool spiked = m_neurons[0][ind].get().newEvent(Event(event.timestamp(), eventPos.x(), eventPos.y(), event.polarity(), event.camera()));
//            double wi = m_neurons[0][ind].get().getWeights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y());
//            neuronsStatistics(event.timestamp(), 0, m_neurons[0][ind].get().getPos(), m_neurons[0][ind].get(), wi);
            if (spiked) {
                m_neurons[0][ind].get().weightUpdate();
                lateralStaticInhibition(m_neurons[0][ind].get());
                lateralDynamicInhibition(m_neurons[0][ind].get());
                if (m_neurons.size() > 1) {
                    addNeuronEvent(m_neurons[0][ind].get());
                }
            }
        } else if (m_networkConf.getNeuron1Synapses() > 1) {
            m_neurons[0][ind].get().newEvent(Event(event.timestamp(), eventPos.x(), eventPos.y(), event.polarity(), event.camera()));
            updateMultiSynapticNeurons(event.timestamp());
        }
    }
}

/**
 * @brief Recursive function that updates neurons deeper than the 1st layer. Similarly to the addEvent function, if a neuron spikes,
 * it transmits a new event forward.
 * the neuron to which the event is transmitted are determined from vector of reference called outConnections.
 * A neuron spikes activates newStaticInhibitoryEvent connections to adjacent neurons.
 * @param neuron - The neuron that just spiked.
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
        bool spiked = forwardNeuron.get().newEvent(NeuronEvent(neuron.getSpikingTime(),
                                                               neuronPos.x(), neuronPos.y(), neuronPos.z(), neuron.getLayer()));
//        double wi = forwardNeuron.get().getWeights(neuronPos.x(), neuronPos.y(), neuronPos.z());
//        neuronsStatistics(neuron.getSpikingTime(), 0, forwardNeuron.get().getPos(), forwardNeuron.get(), wi);
        if (spiked) {
//            if (forwardNeuron.get().getLayer() == getNetworkStructure().size() - 2) { // critic neuromodulation
//                forwardNeuron.get().setNeuromodulator(m_neuromodulator);
//            }
            forwardNeuron.get().weightUpdate();
            lateralStaticInhibition(forwardNeuron.get());
            topDownDynamicInhibition(forwardNeuron.get());

            addNeuronEvent(forwardNeuron.get());
        }
    }
}

/**
 * @brief Propagation of an inhibitory event to neurons in the previous layer.
 * @param neuron - Neuron that triggered the event.
 */
void SpikingNetwork::topDownDynamicInhibition(Neuron &neuron) {
    for (auto &previousNeuron: neuron.getTopDownDynamicInhibitionConnections()) {
        auto event = NeuronEvent(neuron.getSpikingTime(), neuron.getIndex());
        previousNeuron.get().newTopDownInhibitoryEvent(event);
//        neuronsStatistics(event.timestamp(), 3, neuron.getPos(), previousNeuron.get(), previousNeuron.get().getTopDownInhibitionWeights(event.id()));
    }
}

/**
 * @brief Propagation of an inhibitory event to lateral neurons in the same layer.
 * @param neuron - Neuron that triggered the event.
 */
void SpikingNetwork::lateralDynamicInhibition(Neuron &neuron) {
    for (auto &lateralNeuron: neuron.getLateralDynamicInhibitionConnections()) {
        auto event = NeuronEvent(neuron.getSpikingTime(), neuron.getIndex());
        lateralNeuron.get().newLateralInhibitoryEvent(event);
//        neuronsStatistics(event.timestamp(), 2, neuron.getPos(), lateralNeuron.get(), lateralNeuron.get().getlateralInhibitionWeights(event.id()));
    }
}

/**
 * @brief Propagation of an inhibitory event to neurons with similar visual fields and in the same layer.
 * This event is not stored because there is no plasticity.
 * @param neuron - Neuron that triggered the event.
 */
void SpikingNetwork::lateralStaticInhibition(Neuron &neuron) {
    for (auto &lateralNeuron: neuron.getLateralStaticInhibitionConnections()) {
        auto event = NeuronEvent(neuron.getSpikingTime(), neuron.getIndex());
        lateralNeuron.get().newStaticInhibitoryEvent(event);
//        neuronsStatistics(neuron.getSpikingTime(), 1, neuron.getPos(), lateralNeuron.get(), lateralNeuron.get().getConf().ETA_INH);
    }
}

/**
 * @brief Stores a neuromodulator for subsequent computation.
 * @param neuromodulator - The neuromodulator to be stored.
 */
void SpikingNetwork::transmitNeuromodulator(double neuromodulator) {
    m_neuromodulator = neuromodulator;
}

/**
 * @brief Update function used with synaptic delays.
 * @param time
 */
void SpikingNetwork::updateMultiSynapticNeurons(const long time) {
    for (auto &simpleNeuron: m_simpleNeurons) {
        while (simpleNeuron.checkRemainingEvents(time)) {
            if (simpleNeuron.update()) {
                simpleNeuron.weightUpdate();
                for (auto &simpleNeuronToInhibit: simpleNeuron.getLateralStaticInhibitionConnections()) {
                    auto event = NeuronEvent(simpleNeuron.getSpikingTime(), simpleNeuron.getIndex());
                    simpleNeuronToInhibit.get().newStaticInhibitoryEvent(event);
                }
                lateralDynamicInhibition(simpleNeuron);
                addNeuronEvent(simpleNeuron);
            }
        }
    }
}

/**
 * @brief Generate all the weight matrices that will be used by neurons in the network.
 * The number of weight matrices depends on the type of weight sharing used (only for simple cells, 1st layer at the moment).
 * "none": no weight sharing, every neuron has its own weight matrix.
 * "patch": weight matrices are shared among neurons of similar depth (z axis) and of the same patch.
 * "full": weight matrices are shared among neurons of similar depth (z axis), and for all patches.
 * @param connections
 * @param nbNeurons
 */
void SpikingNetwork::generateWeightSharing(const LayerConnectivity &connections, const size_t nbNeurons) {
    size_t x = static_cast<long>(connections.neuronSizes[0][0]);
    size_t y = static_cast<long>(connections.neuronSizes[0][1]);
    size_t z = static_cast<long>(connections.neuronSizes[0][2]);
    if (connections.neuronType == "SimpleCell") {
        auto dimensions = std::vector<size_t>({NBPOLARITY, static_cast<size_t>(m_networkConf.getNbCameras()),
                                               static_cast<size_t>(m_networkConf.getNeuron1Synapses()), x, y});

        if (m_networkConf.getSharingType() == "none") {
            for (size_t i = 0; i < nbNeurons; ++i) {
                m_sharedWeightsSimple.emplace_back(dimensions, true, 0, m_simpleNeuronConf.NORM_FACTOR);
            }
        } else if (m_networkConf.getSharingType() == "patch") {
            size_t patch_size = connections.patches[0].size() * connections.patches[1].size();
            for (size_t patch = 0; patch < patch_size; ++patch) {
                for (size_t i = 0; i < connections.size[2]; ++i) {
                    m_sharedWeightsSimple.emplace_back(dimensions, true, 0, m_simpleNeuronConf.NORM_FACTOR);
                }
            }
        } else if (m_networkConf.getSharingType() == "full") {
            for (size_t i = 0; i < connections.size[2]; ++i) {
                m_sharedWeightsSimple.emplace_back(dimensions, true, 0, m_simpleNeuronConf.NORM_FACTOR);
            }
        } else {
            std::cout << "Wrong type of sharing" << std::endl;
        }
    }
}

/**
 *
 * @param sgaringType
 * @param connections
 */
void SpikingNetwork::addLayer(const std::string &sharingType, const LayerConnectivity &connections) {
    auto nbNeurons = connections.patches[0].size() * connections.size[0] * connections.patches[1].size() * connections.size[1] * connections.patches[2].size() *
                     connections.size[2];
    generateWeightSharing(connections, nbNeurons);

    size_t neuronIndex = 0;
    size_t weightIndex;
    size_t countWeightSharing = 0;
    m_layout.emplace_back();
    auto layer = m_neurons.size();

    for (size_t x = 0; x < connections.patches[0].size(); ++x) {
        for (size_t y = 0; y < connections.patches[1].size(); ++y) {
            for (size_t z = 0; z < connections.patches[2].size(); ++z) {
                for (size_t i = 0; i < connections.size[0]; ++i) {
                    for (size_t j = 0; j < connections.size[1]; ++j) {
                        for (size_t k = 0; k < connections.size[2]; ++k) {
                            if (sharingType == "none") {
                                weightIndex = neuronIndex;
                            } else if (sharingType == "patch") {
                                weightIndex = countWeightSharing * connections.size[2] + k;
                            } else if (sharingType == "full") {
                                weightIndex = k;
                            }

                            // Position of the neuron in the neuronal layer space (x, y, z)
                            auto pos = Position(x * connections.size[0] + i, y * connections.size[1] + j, z * connections.size[2] + k);

                            // Receptive field starting position (x, y) compared to the previous layer
                            auto offset = Position(connections.patches[0][x] + i * (connections.neuronSizes[0][0] - connections.neuronOverlap[0]), // TODO: more than
                                    // 1 neuronSizes
                                                   connections.patches[1][y] + j * (connections.neuronSizes[0][1] - connections.neuronOverlap[1]));

                            if (connections.neuronType == "SimpleCell") {
                                m_simpleNeurons.emplace_back(SimpleNeuron(neuronIndex, layer, m_simpleNeuronConf, pos, offset,
                                                                          m_sharedWeightsSimple[weightIndex], m_networkConf.getNeuron1Synapses()));
                            } else if (connections.neuronType == "ComplexCell") {
                                m_complexNeurons.emplace_back(ComplexNeuron(neuronIndex, layer, m_complexNeuronConf, pos, offset, connections.neuronSizes[0]));
                            } else if (connections.neuronType == "CriticCell") {
                                m_criticNeurons.emplace_back(MotorNeuron(neuronIndex, layer, m_criticNeuronConf, pos, connections.neuronSizes));
                            } else if (connections.neuronType == "ActorCell") {
                                m_actorNeurons.emplace_back(MotorNeuron(neuronIndex, layer, m_actorNeuronConf, pos, connections.neuronSizes));
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

    if (connections.neuronType == "SimpleCell") {
        m_neurons.emplace_back(m_simpleNeurons.begin(), m_simpleNeurons.end());
    } else if (connections.neuronType == "ComplexCell") {
        m_neurons.emplace_back(m_complexNeurons.begin(), m_complexNeurons.end());
    } else if (connections.neuronType == "CriticCell") {
        m_neurons.emplace_back(m_criticNeurons.begin(), m_criticNeurons.end());
    } else if (connections.neuronType == "ActorCell") {
        m_neurons.emplace_back(m_actorNeurons.begin(), m_actorNeurons.end());
    } else {
        std::cout << "No matching cell type" << std::endl;
    }
    m_structure.push_back(m_neurons[layer].size());
    connectLayer(connections);
}

/**
 *
 * @param connections
 */
void SpikingNetwork::connectLayer(const LayerConnectivity &connections) {
    auto currLayer = m_neurons.size() - 1;
    for (auto &neuron: m_neurons[currLayer]) {
        topDownConnection(neuron.get(), connections.interConnections, currLayer, connections.neuronSizes, connections.inhibitions);
        if (std::find(connections.inhibitions.begin(), connections.inhibitions.end(), "static") != connections.inhibitions.end()) {
            lateralStaticInhibitionConnection(neuron.get(), currLayer, connections.size);
        }
        if (std::find(connections.inhibitions.begin(), connections.inhibitions.end(), "lateral") != connections.inhibitions.end()) {
            lateralDynamicInhibitionConnection(neuron.get(), currLayer, connections.patches, connections.size);
        }
    }
}

/**
 *
 * @param neuron
 * @param currLayer
 * @param inhibition
 * @param interConnections
 * @param neuronSizes
 */
void SpikingNetwork::topDownConnection(Neuron &neuron, const std::vector<int> &interConnections, const size_t currLayer, const std::vector<std::vector<size_t>>
        &neuronSizes, const std::vector<std::string> &inhibition) {
    if (interConnections.size() > 1) { // If there is a skip connection, all to all connection is forced.
        for (const auto layerToConnect: interConnections) {
            for (const auto &prevNeuron: m_neurons[layerToConnect]) {
                neuron.addInConnection(prevNeuron);
                prevNeuron.get().addOutConnection(neuron);
            }
        }
    } else {
        const auto layerToConnect = interConnections[0];
        const auto &neuronSize = neuronSizes[0];
        for (size_t i = neuron.getOffset().x(); i < neuron.getOffset().x() + neuronSize[0]; ++i) {
            for (size_t j = neuron.getOffset().y(); j < neuron.getOffset().y() + neuronSize[1]; ++j) {
                for (size_t k = neuron.getOffset().z(); k < neuron.getOffset().z() + neuronSize[2]; ++k) {
                    if (currLayer == 0) {
                        m_pixelMapping[i * m_networkConf.getVfHeight() + j].push_back(neuron.getIndex());
                    } else {
                        neuron.addInConnection(m_neurons[layerToConnect][m_layout[layerToConnect][{i, j, k}]]);
                        m_neurons[layerToConnect][m_layout[layerToConnect][{i, j, k}]].get().addOutConnection(neuron);
                        if ((std::find(inhibition.begin(), inhibition.end(), "topdown") != inhibition.end())) {
                            neuron.addTopDownDynamicInhibitionConnection(m_neurons[layerToConnect][m_layout[layerToConnect][{i, j, k}]]);
                            m_neurons[layerToConnect][m_layout[layerToConnect][{i, j, k}]].get().initializeTopDownDynamicInhibitionWeights(neuron);
                            m_neurons[layerToConnect][m_layout[layerToConnect][{i, j, k}]].get().addTopDownDynamicInhibitionConnection(neuron);
                        }
                    }
                }
            }
        }
    }
}

/**
 *
 * @param neuron
 * @param currLayer
 * @param layerPatches
 * @param layerSizes
 */
void SpikingNetwork::lateralDynamicInhibitionConnection(Neuron &neuron, const size_t currLayer,
                                                        const std::vector<std::vector<size_t>> &layerPatches,
                                                        const std::vector<size_t> &layerSizes) { // TODO: optim -> very slow
    for (int x = static_cast<int>(neuron.getPos().x()) - 1; x < static_cast<int>(neuron.getPos().x()) + 2; ++x) {
        for (int y = static_cast<int>(neuron.getPos().y()) - 1; y < static_cast<int>(neuron.getPos().y()) + 2; ++y) {
            for (size_t z = 0; z < layerSizes[2]; ++z) {
                if ((x != neuron.getPos().x() || y != neuron.getPos().y()) && x >= 0 && y >= 0 &&
                    x < layerPatches[0].size() * layerSizes[0] &&
                    y < layerPatches[1].size() * layerSizes[1]) {
                    neuron.addLateralDynamicInhibitionConnections(m_neurons[currLayer][m_layout[currLayer][{x, y, z}]]);
                }
            }
        }
    }
}

/**
 *
 * @param neuron
 * @param currLayer
 * @param layerSizes
 */
void SpikingNetwork::lateralStaticInhibitionConnection(Neuron &neuron, const size_t currLayer,
                                                       const std::vector<size_t> &layerSizes) {
    for (size_t z = 0; z < layerSizes[2]; ++z) {
        if (z != neuron.getPos().z()) {
            neuron.addLateralStaticInhibitionConnections(m_neurons[currLayer][m_layout[currLayer][{neuron.getPos().x(), neuron.getPos().y(), z}]]);
        }
    }
}

/**
 * Updates some neurons internal state.
 * Computes each neurons average spiking rate.
 * Adapts the threshold of the 1st layer's neurons.
 * @param timeInterval
 */
void SpikingNetwork::updateNeuronsStates(long timeInterval) {
    size_t layer;
    double averageActivity = 0;
    for (auto &neurons: m_neurons) {
        for (auto &neuron: neurons) {
            neuron.get().updateState(timeInterval);
            if (layer == 0) {
                neuron.get().thresholdAdaptation();
                averageActivity += neuron.get().getSpikingRate();
            }
        }
    }
    averageActivity /= static_cast<double>(m_neurons[0].size());
    auto alpha = 0.6;
    m_averageActivity = (alpha * averageActivity) + (1.0 - alpha) * m_averageActivity;
}

/**
 *
 */
//void SpikingNetwork::normalizeActions() {
//    auto layer = m_neurons.size() - 1;
//    auto norms = std::vector<double>(getNetworkStructure().back(), 0);
//
//    double normMax = 0;
//    size_t count = 0;
//    for (auto &neuron: m_neurons[layer]) {
//        norms[count] = neuron.get().computeNormWeights();
//        if (norms[count] > normMax) {
//            normMax = norms[count];
//        }
//        ++count;
//    }
//
//    count = 0;
//    for (auto &neuron: m_neurons[layer]) {
//        if (normMax != norms[count]) {
//            neuron.get().rescaleWeights(normMax / norms[count]);
//        }
//        ++count;
//    }
//}

/**
 *
 */
void SpikingNetwork::saveNetwork() {
    saveNeuronsStates();
    saveNetworkLayout();
}

/**
 *
 */
void SpikingNetwork::saveNetworkLayout() {
    size_t count;
    for (size_t layer = 0; layer < m_neurons.size(); ++layer) {
        std::vector<size_t> data(static_cast<size_t>(getNetworkStructure()[layer]));
        count = 0;
        auto size_x = m_networkConf.getLayerConnectivity()[layer].patches[0].size() * m_networkConf.getLayerConnectivity()[layer].size[0];
        auto size_y = m_networkConf.getLayerConnectivity()[layer].patches[1].size() * m_networkConf.getLayerConnectivity()[layer].size[1];
        auto size_z = m_networkConf.getLayerConnectivity()[layer].patches[2].size() * m_networkConf.getLayerConnectivity()[layer].size[2];
        for (size_t i = 0; i < size_x; ++i) {
            for (size_t j = 0; j < size_y; ++j) {
                for (size_t k = 0; k < size_z; ++k) {
                    data[count] = m_layout[layer][{i, j, k}];
                    ++count;
                }
            }
        }
        cnpy::npy_save(m_networkConf.getNetworkPath() + "weights/layout_" + std::to_string(layer) + ".npy", &data[0],
                       {size_x, size_y, size_z}, "w");
    }
}

/**
 *
 * @param saveCount
 */
void SpikingNetwork::intermediateSave(size_t saveCount) {
    std::string fileName;
    std::filesystem::create_directory(m_networkConf.getNetworkPath() + "weights/intermediate_" + std::to_string(saveCount) + "/");
    std::filesystem::create_directory(m_networkConf.getNetworkPath() + "weights/intermediate_" + std::to_string(saveCount) + "/0/");
    std::filesystem::create_directory(m_networkConf.getNetworkPath() + "weights/intermediate_" + std::to_string(saveCount) + "/1/");

    size_t layer = 0;
    for (auto &neurons: m_neurons) {
        for (size_t i = 0; i < m_networkConf.getLayerConnectivity()[layer].size[2]; ++i) {
            fileName = m_networkConf.getNetworkPath() + "weights/intermediate_" + std::to_string(saveCount) + "/" + std::to_string(layer) + "/";
            neurons[i].get().saveWeights(fileName);
        }
        ++layer;
    }
}

/**
 *
 */
void SpikingNetwork::saveNeuronsStates() {
    size_t layer = 0;
    std::string fileName;
    bool tdi, li;

    for (auto &neurons: m_neurons) {
        if (std::find(m_networkConf.getLayerConnectivity()[layer].inhibitions.begin(),
                      m_networkConf.getLayerConnectivity()[layer].inhibitions.end(), "lateral") != m_networkConf.getLayerConnectivity()[layer].inhibitions.end()) {
            li = true;
        } else {
            li = false;
        }
        if (std::find(m_networkConf.getLayerConnectivity()[layer].inhibitions.begin(),
                      m_networkConf.getLayerConnectivity()[layer].inhibitions.end(), "topdown") != m_networkConf.getLayerConnectivity()[layer].inhibitions.end()) {
            tdi = true;
        } else {
            tdi = false;
        }

        if (layer == 0 && m_networkConf.getSharingType() == "patch") {
            size_t step = m_networkConf.getLayerConnectivity()[layer].size[0] * m_networkConf.getLayerConnectivity()[layer].size[1] * m_networkConf.getLayerConnectivity()[layer].size[2];
            size_t patch_size = m_networkConf.getLayerConnectivity()[layer].patches[0].size() * m_networkConf.getLayerConnectivity()[layer].patches[1].size();
            for (size_t patch = 0; patch < patch_size; ++patch) {
                for (size_t i = 0; i < m_networkConf.getLayerConnectivity()[layer].size[2]; ++i) {
                    fileName = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
                    neurons[patch * step + i].get().saveWeights(fileName);
                }
            }
        } else if (layer == 0 && m_networkConf.getSharingType() == "full") {
            for (size_t i = 0; i < m_networkConf.getLayerConnectivity()[layer].size[2]; ++i) {
                fileName = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
                neurons[i].get().saveWeights(fileName);
            }
        }
        for (auto &neuron: neurons) {
            fileName = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
            neuron.get().saveState(fileName);
            if (li) {
                neuron.get().saveLateralInhibitionWeights(fileName);
            }
            if (tdi) {
                neuron.get().saveTopDownInhibitionWeights(fileName);
            }
            if (layer != 0 || m_networkConf.getSharingType() == "none") {
                neuron.get().saveWeights(fileName);
            }
        }
        ++layer;
    }

//    size_t layer = 0;
//    std::string statePath, filePath, tdiFilePath, liFilePath;
//    bool tdi = false, li = false;
//
//    for (auto &neurons: m_neurons) {
//        filePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/weights.npz";
//        liFilePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/weightsLI.npz";
//        tdiFilePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/weightsTDI.npz";
//        statePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
//
//        std::vector<double> emptyArray(0);
//        cnpy::npz_save(filePath, "", &emptyArray[0], {0}, "w");
//
//        if (std::find(m_networkConf.getLayerConnectivity()[layer].inhibitions.begin(),
//                      m_networkConf.getLayerConnectivity()[layer].inhibitions.end(), "lateral") != m_networkConf.getLayerConnectivity()[layer].inhibitions.end()) {
//            cnpy::npz_save(liFilePath, "", &emptyArray[0], {0}, "w");
//            li = true;
//        }
//        if (std::find(m_networkConf.getLayerConnectivity()[layer].inhibitions.begin(),
//                      m_networkConf.getLayerConnectivity()[layer].inhibitions.end(), "topdown") != m_networkConf.getLayerConnectivity()[layer].inhibitions.end()) {
//            cnpy::npz_save(tdiFilePath, "", &emptyArray[0], {0}, "w");
//            tdi = true;
//        }
//
//        if (layer == 0 && m_networkConf.getSharingType() == "patch") {
//            size_t step = m_networkConf.getLayerConnectivity()[layer].size[0] * m_networkConf.getLayerConnectivity()[layer].size[1] * m_networkConf.getLayerConnectivity()[layer].size[2];
//            size_t patch_size = m_networkConf.getLayerPatches()[0][0].size() * m_networkConf.getLayerPatches()[0][1].size();
//            for (size_t patch = 0; patch < patch_size; ++patch) {
//                for (size_t i = 0; i < m_networkConf.getLayerConnectivity()[layer].size[2]; ++i) {
//                    neurons[patch * step + i].get().saveWeights(filePath);
//                }
//            }
//        }
//
//        for (auto &neuron: neurons) {
//            neuron.get().saveState(statePath);
//
//            if (li) {
//                neuron.get().saveLateralInhibitionWeights(liFilePath);
//            }
//            if (tdi) {
//                neuron.get().saveTopDownInhibitionWeights(tdiFilePath);
//            }
//            if (layer != 0 || m_networkConf.getSharingType() != "patch") {
//                neuron.get().saveWeights(filePath);
//            }
//        }
//
//        ++layer;
//    }
}

/**
 *
 */
void SpikingNetwork::loadWeights() {
    std::string statePath, filePath, tdiFilePath, liFilePath;
    size_t layer = 0;
    bool tdi, li;
    cnpy::npz_t liArrayNPZ, tdiArrayNPZ;

    for (auto &neurons: m_neurons) {
        std::string path(m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/0.npy");
        liFilePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/0li.npy";
        tdiFilePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/0tdi.npy";
        if (std::find(m_networkConf.getLayerConnectivity()[layer].inhibitions.begin(), m_networkConf.getLayerConnectivity()[layer].inhibitions.end(), "lateral") !=
            m_networkConf.getLayerConnectivity()[layer].inhibitions.end() && Util::fileExist(liFilePath)) {
            li = true;
        } else {
            li = false;
        }
        if (std::find(m_networkConf.getLayerConnectivity()[layer].inhibitions.begin(), m_networkConf.getLayerConnectivity()[layer].inhibitions.end(), "topdown") !=
            m_networkConf.getLayerConnectivity()[layer].inhibitions.end() && Util::fileExist(tdiFilePath)) {
            tdi = true;
        } else {
            tdi = false;
        }
        if (Util::fileExist(path)) {
            if (layer == 0 && m_networkConf.getSharingType() == "patch") {
                size_t step = m_networkConf.getLayerConnectivity()[layer].size[0] * m_networkConf.getLayerConnectivity()[layer].size[1] * m_networkConf.getLayerConnectivity()[layer].size[2];
                size_t patch_size = m_networkConf.getLayerConnectivity()[layer].patches[0].size() * m_networkConf.getLayerConnectivity()[layer].patches[1].size();
                for (size_t patch = 0; patch < patch_size; ++patch) {
                    for (size_t i = 0; i < m_networkConf.getLayerConnectivity()[layer].size[2]; ++i) {
                        filePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
                        neurons[patch * step + i].get().loadWeights(filePath);
                    }
                }
            } else if (layer == 0 && m_networkConf.getSharingType() == "full") {
                for (size_t i = 0; i < m_networkConf.getLayerConnectivity()[layer].size[2]; ++i) {
                    filePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
                    neurons[i].get().loadWeights(filePath);
                }
            }
            for (auto &neuron: neurons) {
                filePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
                neuron.get().loadState(filePath);
                if (li) {
                    neuron.get().loadLateralInhibitionWeights(filePath);
                }
                if (tdi) {
                    neuron.get().loadTopDownInhibitionWeights(filePath);
                }
                if (layer != 0 || m_networkConf.getSharingType() == "none") {
                    neuron.get().loadWeights(filePath);
                }
            }
            std::cout << "Layer " << layer << ": weights loaded from file" << std::endl;
        } else {
            std::cout << "Layer " << layer << ": new weights generated" << std::endl;
        }
        ++layer;
    }

//    for (auto &neurons: m_neurons) {
//        filePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/weights.npz";
//        liFilePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/weightsLI.npz";
//        tdiFilePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/weightsTDI.npz";
//        statePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
//
//        if (Util::fileExist(liFilePath)) {
//            liArrayNPZ = cnpy::npz_load(liFilePath);
//            li = true;
//        } else {
//            li = false;
//        }
//        if (Util::fileExist(tdiFilePath)) {
//            tdiArrayNPZ = cnpy::npz_load(liFilePath);
//            tdi = true;
//        } else {
//            tdi = false;
//        }
//
//        if (Util::fileExist(filePath)) {
//            auto arrayNPZ = cnpy::npz_load(filePath);
//
//            if (layer == 0 && m_networkConf.getSharingType() == "patch") {
//                size_t step = m_networkConf.getLayerConnectivity()[layer].size[0] * m_networkConf.getLayerConnectivity()[layer].size[1] * m_networkConf.getLayerConnectivity()[layer].size[2];
//                size_t patch_size = m_networkConf.getLayerPatches()[0][0].size() * m_networkConf.getLayerPatches()[0][1].size();
//                for (size_t patch = 0; patch < patch_size; ++patch) {
//                    for (size_t i = 0; i < m_networkConf.getLayerConnectivity()[layer].size[2]; ++i) {
//                        neurons[patch * step + i].get().loadWeights(arrayNPZ);
//                    }
//                }
//            }
//
//            for (auto &neuron: neurons) {
//                neuron.get().loadState(statePath);
//
//                if (li) {
//                    neuron.get().loadLateralInhibitionWeights(liArrayNPZ);
//                }
//                if (tdi) {
//                    neuron.get().loadTopDownInhibitionWeights(tdiArrayNPZ);
//                }
//                if (layer != 0 || m_networkConf.getSharingType() != "patch") {
//                    neuron.get().loadWeights(arrayNPZ);
//                }
//            }
//            std::cout << "Layer " << layer << ": weights loaded from file" << std::endl;
//        } else {
//            std::cout << "Layer " << layer << ": new weights generated" << std::endl;
//        }
//        ++layer;
//    }
}

/**
 *
 * @param index
 * @param layer
 * @return
 */
std::reference_wrapper<Neuron> &SpikingNetwork::getNeuron(const size_t index, const size_t layer) {
    if (layer < m_neurons.size()) {
        if (index < m_neurons[layer].size()) {
            return m_neurons[layer][index];
        }
    }
    throw std::runtime_error("Wrong layer or index for neuron selection");
}


void SpikingNetwork::neuronsStatistics(uint64_t time, int type_, Position pos, Neuron &neuron, double wi) {
    if (neuron.getConf().POTENTIAL_TRACK[0] == neuron.getPos().x() && neuron.getConf().POTENTIAL_TRACK[1] == neuron.getPos().y()) {
        neuron.assignToAmountOfEvents(type_);
        neuron.assignToPotentialThreshold();
        // std::pair<double, uint64_t> pot = std::make_pair(neuron.getPotential(time),time);
        neuron.assignToPotentialTrain(std::make_pair(neuron.getPotential(time), time));
        if (type_ != 0) {
            type_ -= 1;
            if (type_ != 2) {
                if (pos.x() > neuron.getPos().x() + 1 || pos.x() < neuron.getPos().x() - 1) {
                    std::cout << "pos x = " << pos.x() << " ; pos y = " << pos.y() << std::endl;
                    std::cout << "type = " << type_ << std::endl;
                    std::cout << "neuron to track x = " << neuron.getConf().POTENTIAL_TRACK[0] << " ; y = " << neuron.getConf().POTENTIAL_TRACK[1]
                              << std::endl << std::endl;
                }
                neuron.assignToSumInhibWeights(type_, pos, wi);
            }
            std::tuple<double, double, uint64_t> var = {neuron.getPotential(time) - wi, neuron.getPotential(time), time};
//                std::cout << "value of potential = " << std::get<0>(var) << " ; value of potential afterwards = " << std::get<1>(var) << " ; value of time = " << std::get<2>(var) << std::endl << std::endl;
//                std::cout << "real value of potential now = " << neuron.getPotential(time) << std::endl;
            neuron.assignToTimingOfInhibition(type_, var);
        }
    }
}

void SpikingNetwork::saveStatistics(int sequence) {
    std::string fileName;
    size_t layer = 0;
    std::filesystem::file_status s = std::filesystem::file_status{};
    for (auto &neurons: m_neurons) {
        std::string path(m_networkConf.getNetworkPath() + "statistics/" + std::to_string(layer));
        if (std::filesystem::status_known(s) ? std::filesystem::exists(s) : std::filesystem::exists(path)) {
            std::filesystem::create_directory(
                    m_networkConf.getNetworkPath() + "statistics/" + std::to_string(layer) + "/" + std::to_string(sequence));
            for (auto &neuron: neurons) {
                if (neuron.get().getConf().POTENTIAL_TRACK[0] == neuron.get().getPos().x() &&
                    neuron.get().getConf().POTENTIAL_TRACK[1] == neuron.get().getPos().y()) {
                    fileName = m_networkConf.getNetworkPath() + "statistics/" + std::to_string(layer) + "/" + std::to_string(sequence) + "/";
                    saveStatesStatistics(fileName, neuron.get());
                }
            }
            ++layer;
        }
    }
}

void SpikingNetwork::writeJsonNeuronsStatistics(nlohmann::json &state, Neuron &neuron) {
    state["amount_of_events"] = neuron.getAmountOfEvents();
    state["potentials_thresholds"] = neuron.getPotentialThreshold();
    state["potential_train"] = neuron.getPotentialTrain();
    state["sum_inhib_weights"] = neuron.getSumInhibWeights();
    state["timing_of_inhibition"] = neuron.getTimingOfInhibition();
}

void SpikingNetwork::saveStatesStatistics(std::string &fileName, Neuron &neuron) {
    nlohmann::json state;
    writeJsonNeuronsStatistics(state, neuron);
    std::ofstream ofs(fileName + std::to_string(neuron.getIndex()) + ".json");
    if (ofs.is_open()) {
        ofs << std::setw(4) << state << std::endl;
    } else {
        std::cout << "cannot save neuron state statistics file" << std::endl;
    }
    ofs.close();
}
