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

void SpikingNetwork::runEvent(const Event &event) {
    addEvent(event);
    ++m_iterations;
}

/* Iterate the network on the event, updating every 1st layer neuron connected to the subsequent pixel.
 * Determines wich neuron to update depending on a mapping between pixels and neurons.
 * If a neuron exceeds a threshold, it spikes and transmit another event towards deeper neurons.
 * A neuron spikes activates inhibition connections to adjacent neurons.
 */
inline void SpikingNetwork::addEvent(const Event &event) {
    for (size_t ind: m_pixelMapping[static_cast<uint32_t>(event.x()) * Conf::HEIGHT +
                                    static_cast<uint32_t>(event.y())]) {
        if (m_neurons[0][ind].get().newEvent(Event(event.timestamp(), event.x() - static_cast<int16_t>
        (m_neurons[0][ind].get().getOffset().x()), event.y() - static_cast<int16_t>(m_neurons[0][ind].get().getOffset()
                .y()), event.polarity(), event.camera()))) {
            m_neurons[0][ind].get().weightUpdate();
            for (auto &neuronToInhibit: m_neurons[0][ind].get().getInhibitionConnections()) {
                neuronToInhibit.get().inhibition();
            }
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
    for (auto &nextNeuron: neuron.getOutConnections()) {
        if (nextNeuron.get().newEvent(NeuronEvent(neuron.getSpikingTime(),
                                                  static_cast<int32_t>(neuron.getPos().x() -
                                                                       nextNeuron.get().getOffset().x()),
                                                  static_cast<int32_t>(neuron.getPos().y() -
                                                                       nextNeuron.get().getOffset().y()),
                                                  static_cast<int32_t>(neuron.getPos().z() -
                                                                       nextNeuron.get().getOffset().z())))) {
            if (nextNeuron.get().getLayer() == 2) {
                nextNeuron.get().setNeuromodulator(computeNeuromodulator(static_cast<double>(neuron.getSpikingTime())));
            }
            if (nextNeuron.get().getLayer() != 3) {
                nextNeuron.get().weightUpdate();
            }

            for (auto &neuronToInhibit: nextNeuron.get().getInhibitionConnections()) {
                neuronToInhibit.get().inhibition();
            }
            addNeuronEvent(nextNeuron.get());
        }
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
                for (auto &simpleNeuronToInhibit: simpleNeuron.getInhibitionConnections()) {
                    simpleNeuronToInhibit.get().inhibition();
                }
                addNeuronEvent(simpleNeuron);
            }
        }
    }
}

void SpikingNetwork::generateWeightSharing(const std::string &neuronType, const std::vector<size_t> &neuronSizes,
                                           const size_t nbNeurons) {
    if (neuronType == "SimpleCell") {
        if (m_networkConf.getSharingType() == "none") {
            for (size_t i = 0; i < nbNeurons; ++i) {
                m_sharedWeightsSimple.push_back(
                        Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(m_networkConf.getNbCameras()),
                                                  static_cast<long>(m_networkConf.getNeuron1Synapses()),
                                                  static_cast<long>(neuronSizes[0]),
                                                  static_cast<long>(neuronSizes[1])));
            }
        }
        size_t patch_size;
        if (m_networkConf.getSharingType() == "full") {
            patch_size = 1;
        } else if (m_networkConf.getSharingType() == "patch") {
            patch_size = m_networkConf.getLayerPatches()[0][0].size() * m_networkConf.getLayerPatches()[0][1].size();
        } else {
            patch_size = 0;
            std::cout << "Wrong type of sharing" << std::endl;
        }
        for (size_t patch = 0; patch < patch_size; ++patch) {
            for (size_t j = 0; j < m_networkConf.getLayerSizes()[0][2]; ++j) {
                m_sharedWeightsSimple.push_back(
                        Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(m_networkConf.getNbCameras()),
                                                  static_cast<long>(m_networkConf.getNeuron1Synapses()),
                                                  static_cast<long>(neuronSizes[0]),
                                                  static_cast<long>(neuronSizes[1])));
            }
        }
    }
    if (neuronType == "ComplexCell") {
        for (size_t i = 0; i < nbNeurons; ++i) {
            m_sharedWeightsComplex.push_back(
                    Util::uniformMatrixComplex(static_cast<long>(neuronSizes[0]),
                                               static_cast<long>(neuronSizes[1]),
                                               static_cast<long>(neuronSizes[2])));
        }
    }
    if (neuronType == "CriticCell") {
        for (size_t i = 0; i < nbNeurons; ++i) {
            m_sharedWeightsCritic.push_back(
                    Util::uniformMatrixComplex(static_cast<long>(neuronSizes[0]),
                                               static_cast<long>(neuronSizes[1]),
                                               static_cast<long>(neuronSizes[2])));
        }
    }
    if (neuronType == "ActorCell") {
        for (size_t i = 0; i < nbNeurons; ++i) {
            m_sharedWeightsActor.push_back(
                    Util::uniformMatrixComplex(static_cast<long>(neuronSizes[0]),
                                               static_cast<long>(neuronSizes[1]),
                                               static_cast<long>(neuronSizes[2])));
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
                            } else if (sharingType == "full" || sharingType == "patch") {
                                weightIndex = countWeightSharing * layerSizes[2] + k;
                            }

                            auto pos = Position(x * layerSizes[0] + i, y * layerSizes[1] + j, z * layerSizes[2] + k);
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
    connectLayer(inhibition, layerToConnect, layerSizes, neuronSizes);
}

void SpikingNetwork::connectLayer(const bool inhibition, const size_t layerToConnect,
                                  const std::vector<size_t> &layerSizes, const std::vector<size_t> &neuronSizes) {
    auto currLayer = m_neurons.size() - 1;
    for (auto &neuron: m_neurons[currLayer]) {
        for (size_t i = neuron.get().getOffset().x(); i < neuron.get().getOffset().x() + neuronSizes[0]; ++i) {
            for (size_t j = neuron.get().getOffset().y(); j < neuron.get().getOffset().y() + neuronSizes[1]; ++j) {
                for (size_t k = neuron.get().getOffset().z(); k < neuron.get().getOffset().z() + neuronSizes[2]; ++k) {
                    if (currLayer == 0) {
                        m_pixelMapping[i * Conf::HEIGHT + j].push_back(neuron.get().getIndex());
                    } else {
                        neuron.get().addInConnection(m_neurons[layerToConnect][m_layout[layerToConnect][{i, j, k}]]);
                        m_neurons[layerToConnect][m_layout[layerToConnect][{i, j, k}]].get().addOutConnection(
                                neuron.get());
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

void SpikingNetwork::updateNeuronsStates(long timeInterval) {
    size_t layer;
    for (auto &neurons: m_neurons) {
        for (auto &neuron: neurons) {
            neuron.get().updateState(timeInterval, 0.05);
            if (layer == 0) {
                neuron.get().thresholdAdaptation();
            }
        }
    }
}

void SpikingNetwork::normalizeActions() {
    auto norm0 = m_neurons[3][0].get().computeNormWeights();
    auto norm1 = m_neurons[3][1].get().computeNormWeights();

    if (norm0 >= norm1) {
        m_neurons[3][1].get().rescaleWeights(norm0 / norm1);
    } else {
        m_neurons[3][0].get().rescaleWeights(norm1 / norm0);
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

void SpikingNetwork::saveNeuronsStates() {
    size_t count, layer = 0;
    std::string fileName;

    for (auto &neurons: m_neurons) {
        count = 0;
        for (auto &neuron: neurons) {
            fileName =
                    m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/" + std::to_string(count);
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

    for (auto &neurons: m_neurons) {
        count = 0;
        std::string path(m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/0.npy");
        if (Util::fileExist(path)) {
            for (auto &neuron: neurons) {
                fileName = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/" +
                           std::to_string(count);
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

std::reference_wrapper<Neuron> &SpikingNetwork::getNeuron(const size_t index, const size_t layer) {
    if (layer < m_neurons.size()) {
        if (index < m_neurons[layer].size()) {
            return m_neurons[layer][index];
        }
    }
    throw std::runtime_error("Wrong layer or index for neuron selection");
}
