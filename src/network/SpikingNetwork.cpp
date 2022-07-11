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
                                                                 m_pixelMapping(std::vector<std::vector<uint64_t>>(Conf::WIDTH * Conf::HEIGHT,
                                                                                                                   std::vector<uint64_t>(0))), 
                                                                 m_simpleWeightsOrientations(m_networkConf.getLayerSizes()[0][2],std::vector<int>(0)),
                                                                 m_complexCellsOrientations(m_networkConf.getLayerSizes()[1][0]*m_networkConf.getLayerSizes()[1][1]*m_networkConf.getLayerSizes()[1][2],std::vector<int>(0)){
    for (size_t i = 0; i < m_networkConf.getLayerCellTypes().size(); ++i) {
        addLayer(m_networkConf.getLayerCellTypes()[i], m_networkConf.getSharingType(),
                 m_networkConf.getLayerInhibitions()[i],
                 m_networkConf.getLayerPatches()[i], m_networkConf.getLayerSizes()[i],
                 m_networkConf.getNeuronSizes()[i],
                 m_networkConf.getNeuronOverlap()[i], m_networkConf.getInterLayerConnections()[i]);
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
/*    if(event.x() < 150) { 
        for(int v: m_pixelMapping[static_cast<uint32_t>(event.x()) * Conf::HEIGHT +static_cast<uint32_t>(event.y())]) {
                        std::cout << "x, y, timestamp init = " << static_cast<uint32_t>(event.x()) << " ; " << static_cast<uint32_t>(event.y()) << " ; " << event.timestamp() << " ; event pol = " << event.polarity() << std::endl;
                        std::cout << "v = " << v << std::endl;
        }
            //    std::cout << "v = " << v << std::endl;
            }*/
    for (size_t ind: m_pixelMapping[static_cast<uint32_t>(event.x()) * Conf::HEIGHT +
                                    static_cast<uint32_t>(event.y())]) {

    /*    if(event.x() < 150) { 
                std::cout << "x, y, timestamp init = " << event.x() << " ; " << event.y() << " ; " << event.timestamp() << " ; event pol = " << event.polarity() << std::endl;
            }*/
        auto eventPos = Position(event.x() - static_cast<int16_t>(m_neurons[0][ind].get().getOffset().x()),
                                 event.y() - static_cast<int16_t>(m_neurons[0][ind].get().getOffset().y()));
    /*        if(event.x() < 150) { 
                std::cout << "x, y, timestamp = " << event.x() << " ; " << event.y() << " ; " << event.timestamp() << " ; event pos = " << eventPos.x() << " ; " << eventPos.y() << " ; event pol = " << event.polarity() << std::endl;
                std::cout << "ind = " << ind << std::endl << std::endl;
            }*/
        if (m_networkConf.getNeuron1Synapses() == 1) {
            bool spiked = m_neurons[0][ind].get().newEvent(Event(event.timestamp(), eventPos.x(), eventPos.y(), event.polarity(), event.camera()));
            double wi = m_neurons[0][ind].get().getWeights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y());
            neuronsStatistics(event.timestamp(), 0, m_neurons[0][ind].get().getPos(), m_neurons[0][ind].get(), wi, spiked);
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
            updateNeurons(event.timestamp());
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
        bool spiked = forwardNeuron.get().newEvent(NeuronEvent(neuron.getSpikingTime(), neuronPos.x(), neuronPos.y(), neuronPos.z()));
        double wi = forwardNeuron.get().getWeights(neuronPos.x(), neuronPos.y(), neuronPos.z());
        neuronsStatistics(neuron.getSpikingTime(), 0, forwardNeuron.get().getPos(), forwardNeuron.get(), wi, spiked);
        if (spiked) {
//            if (forwardNeuron.get().getLayer() == 2) { // critic neuromodulation
//                neuromodulation(forwardNeuron.get());
//            }
            forwardNeuron.get().weightUpdate();
            lateralStaticInhibition(forwardNeuron.get());
        //    topDownDynamicInhibition(forwardNeuron.get());

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
        Position pos(neuron.getPos().x(), neuron.getPos().y(),neuron.getIndex());
        neuronsStatistics(event.timestamp(), 3, pos, previousNeuron.get(), previousNeuron.get().getTopDownInhibitionWeights(event.id()));
    }
}

/**
 * @brief Propagation of an inhibitory event to lateral neurons in the same layer.
 * @param neuron - Neuron that triggered the event.
 */
void SpikingNetwork::lateralDynamicInhibition(Neuron &neuron) {
    for (auto &lateralNeuron: neuron.getLateralDynamicInhibitionConnections()) {
        Position pos = neuron.getPos();

        //angle = 135° network = network6test
    /*    if(!(pos.z()==9 || pos.z()==10 || pos.z() == 17 || pos.z()==22 || pos.z()==24 || pos.z() == 25 || pos.z()==28 || pos.z() == 37 || pos.z() ==43 || pos.z() == 45 || pos.z() == 51 || pos.z() == 52
                                || pos.z() == 56 || pos.z() == 59 || pos.z() == 80 || pos.z() == 86 || pos.z() == 87 || pos.z() == 88 || pos.z() == 97 || pos.z() == 98 || pos.z() == 106 
                                || pos.z() == 112 || pos.z() == 120 || pos.z() == 124 || pos.z() == 133 || pos.z() == 139 || pos.z() == 144 || pos.z() == 146 || pos.z() == 148 || pos.z() == 151 || pos.z() == 152 || pos.z() == 157 || pos.z() == 158 || pos.z() == 162 || pos.z() == 171 || pos.z() == 176 || pos.z() == 181 || pos.z() == 182 || pos.z() == 178 || pos.z() == 183
                                || pos.z() == 184 || pos.z() == 194 || pos.z() == 196 || pos.z() == 200 || pos.z() == 201 || pos.z() == 202 || pos.z() == 205 || pos.z() == 208 || pos.z() == 212 || pos.z() == 220 || pos.z() == 235 || pos.z() == 223 || pos.z() == 228 || pos.z() == 231 || pos.z() == 245 || pos.z() == 246 || pos.z() == 248)) {*/

    //angle = 0° network = network6test        
    /*    if(!(pos.z()==7 || pos.z()==8 || pos.z() == 16 || pos.z() == 23 || pos.z() == 26 || pos.z() == 50 || pos.z() == 57 || pos.z() == 61 || pos.z() == 64 || pos.z() == 70 || pos.z() == 85 || pos.z() == 90 
             || pos.z() == 92 || pos.z() == 101 || pos.z() == 105 || pos.z() == 123 || pos.z() == 129 || pos.z()== 132 || pos.z() == 141 || pos.z() == 142 || pos.z() == 147 || pos.z() == 153 || pos.z() == 160 
             || pos.z() == 165 || pos.z() == 187 || pos.z() == 195 || pos.z() == 202 || pos.z() == 224 || pos.z() == 225)) {*/
            
    //angle = 90° network = network6test
    /*    if( !(pos.z()==25 || pos.z()==34 || pos.z() == 44 || pos.z() == 46 || pos.z() == 54 || pos.z() == 55 || pos.z() == 58 || pos.z() == 71 || pos.z() == 79 || pos.z() == 80 || pos.z() == 83 || pos.z() == 99
            || pos.z() == 108 || pos.z() == 111 || pos.z() == 120 || pos.z() == 137 || pos.z() == 146 || pos.z() == 150 || pos.z() == 163 || pos.z() == 181 || pos.z() == 185 || pos.z() == 188 || pos.z() == 189 
            || pos.z() == 206 || pos.z() == 215 || pos.z() == 217 || pos.z() == 218 || pos.z() == 240)) {*/       
            
            auto event = NeuronEvent(neuron.getSpikingTime(), neuron.getIndex());
            lateralNeuron.get().newLateralInhibitoryEvent(event);
            neuronsStatistics(event.timestamp(), 2, neuron.getPos(), lateralNeuron.get(), lateralNeuron.get().getlateralInhibitionWeights(event.id()));
                                
    //    }
        
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
        neuronsStatistics(neuron.getSpikingTime(), 1, neuron.getPos(), lateralNeuron.get(), lateralNeuron.get().getConf().ETA_INH);
    }
}

/**
 * @brief Stores a reward as a neuromodulator for subsequent computation.
 * @param reward - The reward to be stored.
 */
void SpikingNetwork::transmitReward(double reward) {
    m_reward = reward;
}

//void SpikingNetwork::neuromodulation(Neuron &neuron) {
//    neuron.setNeuromodulator(computeNeuromodulator(neuron.getSpikingTime()));
//}

void SpikingNetwork::updateNeurons(const long time) {
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

//inline double SpikingNetwork::computeNeuromodulator(long time) {
//    double value = 0;
//    for (const auto &critic: m_neurons[2]) {
//        value += critic.get().updateKernelSpikingRate(time);
//    }
//    return -value / static_cast<double>(m_neurons[2].size()) + m_reward;
////    auto V = m_networkConf.getNu() * value / static_cast<double>(m_neurons[2].size()) + m_networkConf.getV0(); //TODO: reintroduce params
////    return -V / m_networkConf.getTauR() + m_reward;
//}

/**
 * @brief Previous update function used with synaptic delays.
 * Not in use at the moment.
 * @param time
 */
//void SpikingNetwork::updateNeurons(const long time) {
//    for (auto &simpleNeuron: m_simpleNeurons) {
//        while (simpleNeuron.checkRemainingEvents(time)) {
//            if (simpleNeuron.update()) {
//                for (auto &simpleNeuronToInhibit: simpleNeuron.getLateralStaticInhibitionConnections()) {
//                    simpleNeuronToInhibit.get().newStaticInhibitoryEvent();
//                }
//                addNeuronEvent(simpleNeuron);
//            }
//        }
//    }
//}

/**
 * @brief Generate all the weight matrices that will be used by neurons in the network.
 * The number of weight matrices depends on the type of weight sharing used (only for simple cells, 1st layer at the moment).
 * "none": no weight sharing, every neuron has its own weight matrix.
 * "patch": weight matrices are shared among neurons of similar depth (z axis) and of the same patch.
 * "full": weight matrices are shared among neurons of similar depth (z axis), and for all patches.
 * @param neuronType
 * @param neuronSizes
 * @param nbNeurons
 */
void SpikingNetwork::generateWeightSharing(const std::string &neuronType, const std::vector<size_t> &neuronSizes,
                                           const size_t nbNeurons) {
    long x = static_cast<long>(neuronSizes[0]);
    long y = static_cast<long>(neuronSizes[1]);
    long z = static_cast<long>(neuronSizes[2]);
    if (neuronType == "SimpleCell") {
        if (m_networkConf.getSharingType() == "none") {
            for (size_t i = 0; i < nbNeurons; ++i) {
                m_sharedWeightsSimple.push_back(Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(m_networkConf.getNbCameras()),
                                                                          static_cast<long>(m_networkConf.getNeuron1Synapses()), x, y,
                                                                          m_simpleNeuronConf.NORM_FACTOR));
            }
        } else if (m_networkConf.getSharingType() == "patch") {
            size_t patch_size = m_networkConf.getLayerPatches()[0][0].size() * m_networkConf.getLayerPatches()[0][1].size();
            for (size_t patch = 0; patch < patch_size; ++patch) {
                for (size_t i = 0; i < m_networkConf.getLayerSizes()[0][2]; ++i) {
                    m_sharedWeightsSimple.push_back(
                            Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(m_networkConf.getNbCameras()),
                                                      static_cast<long>(m_networkConf.getNeuron1Synapses()), x, y, m_simpleNeuronConf.NORM_FACTOR));
                }
            }
        } else if (m_networkConf.getSharingType() == "full") {
            for (size_t i = 0; i < m_networkConf.getLayerSizes()[0][2]; ++i) {
                m_sharedWeightsSimple.push_back(
                        Util::uniformMatrixSimple(NBPOLARITY, static_cast<long>(m_networkConf.getNbCameras()),
                                                  static_cast<long>(m_networkConf.getNeuron1Synapses()), x, y, m_simpleNeuronConf.NORM_FACTOR));
            }
        } else {
            std::cout << "Wrong type of sharing" << std::endl;
        }
    }
    if (neuronType == "ComplexCell") {
        for (size_t i = 0; i < nbNeurons; ++i) {
            m_sharedWeightsComplex.push_back(Util::uniformMatrixComplex(x, y, z, m_complexNeuronConf.NORM_FACTOR));
        }
    /*    for (size_t i = 0; i < m_networkConf.getLayerSizes()[1][2]; ++i) {
                            m_sharedWeightsComplex.push_back(Util::uniformMatrixComplex(x, y, z, m_complexNeuronConf.NORM_FACTOR));
            }*/

    }
    if (neuronType == "CriticCell") {
        for (size_t i = 0; i < nbNeurons; ++i) {
            m_sharedWeightsCritic.push_back(Util::uniformMatrixComplex(x, y, z, m_criticNeuronConf.NORM_FACTOR));
        }
    }
    if (neuronType == "ActorCell") {
        for (size_t i = 0; i < nbNeurons; ++i) {
            m_sharedWeightsActor.push_back(Util::uniformMatrixComplex(x, y, z, m_actorNeuronConf.NORM_FACTOR));
        }
    }
}

/**
 *
 * @param neuronType
 * @param sharingType
 * @param inhibition
 * @param layerPatches
 * @param layerSizes
 * @param neuronSizes
 * @param neuronOverlap
 * @param layerToConnect
 */
void SpikingNetwork::addLayer(const std::string &neuronType, const std::string &sharingType,
                              const std::vector<std::string> &inhibition,
                              const std::vector<std::vector<size_t>> &layerPatches,
                              const std::vector<size_t> &layerSizes,
                              const std::vector<size_t> &neuronSizes,
                              const std::vector<size_t> &neuronOverlap,
                              const size_t layerToConnect) {
    auto nbNeurons = layerPatches[0].size() * layerSizes[0] * layerPatches[1].size() * layerSizes[1] * layerPatches[2].size() *
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
                            } else if (sharingType == "full") {
                                weightIndex = k;
                            }

                            // Position of the neuron in the neuronal layer space (x, y, z)
                            auto pos = Position(x * layerSizes[0] + i, y * layerSizes[1] + j, z * layerSizes[2] + k);

                            // Receptive field starting position (x, y) compared to the previous layer
                            auto offset = Position(layerPatches[0][x] + i * (neuronSizes[0] - neuronOverlap[0]),
                                                   layerPatches[1][y] + j * (neuronSizes[1] - neuronOverlap[1]));

                            if (neuronType == "SimpleCell") {
                                m_simpleNeurons.emplace_back(SimpleNeuron(neuronIndex, layer, m_simpleNeuronConf, pos, offset,
                                                                          m_sharedWeightsSimple[weightIndex], m_networkConf.getNeuron1Synapses()));
                                m_simpleNeurons.at(m_simpleNeurons.size()-1).setInhibitionRange(m_networkConf.getNeuronInhibitionRange());
                            } else if (neuronType == "ComplexCell") {
                                m_complexNeurons.emplace_back(ComplexNeuron(neuronIndex, layer, m_complexNeuronConf, pos, offset,
                                                                            m_sharedWeightsComplex[neuronIndex]));
                            //    m_complexNeurons.emplace_back(ComplexNeuron(neuronIndex, layer, m_complexNeuronConf, pos, offset,
                                                                    //        m_sharedWeightsComplex[weightIndex]));
                            } else if (neuronType == "CriticCell") {
                                m_criticNeurons.emplace_back(MotorNeuron(neuronIndex, layer, m_criticNeuronConf, pos,
                                                                         m_sharedWeightsCritic[neuronIndex]));
                            } else if (neuronType == "ActorCell") {
                                m_actorNeurons.emplace_back(MotorNeuron(neuronIndex, layer, m_actorNeuronConf, pos,
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
    connectLayer(layerToConnect, inhibition, layerPatches, layerSizes, neuronSizes);
}

/**
 *
 * @param layerToConnect
 * @param inhibition
 * @param layerPatches
 * @param layerSizes
 * @param neuronSizes
 */
void SpikingNetwork::connectLayer(const size_t layerToConnect, const std::vector<std::string> &inhibition,
                                  const std::vector<std::vector<size_t>> &layerPatches,
                                  const std::vector<size_t> &layerSizes, const std::vector<size_t> &neuronSizes) {
    auto currLayer = m_neurons.size() - 1;
    for (auto &neuron: m_neurons[currLayer]) {
        topDownConnection(neuron.get(), currLayer, inhibition, layerToConnect, neuronSizes);
        if (std::find(inhibition.begin(), inhibition.end(), "static") != inhibition.end()) {
            lateralStaticInhibitionConnection(neuron.get(), currLayer, layerSizes);
        }
        if (std::find(inhibition.begin(), inhibition.end(), "lateral") != inhibition.end()) {
            lateralDynamicInhibitionConnection(neuron.get(), currLayer, layerPatches, layerSizes);
        }
    }
}

/**
 *
 * @param neuron
 * @param currLayer
 * @param inhibition
 * @param layerToConnect
 * @param neuronSizes
 */
void SpikingNetwork::topDownConnection(Neuron &neuron, const size_t currLayer,
                                       const std::vector<std::string> &inhibition,
                                       const size_t layerToConnect,
                                       const std::vector<size_t> &neuronSizes) {
    for (size_t i = neuron.getOffset().x(); i < neuron.getOffset().x() + neuronSizes[0]; ++i) {
        for (size_t j = neuron.getOffset().y(); j < neuron.getOffset().y() + neuronSizes[1]; ++j) {
            for (size_t k = neuron.getOffset().z(); k < neuron.getOffset().z() + neuronSizes[2]; ++k) {
                if (currLayer == 0) {
                    m_pixelMapping[i * Conf::HEIGHT + j].push_back(neuron.getIndex());
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
    
    for (int x = static_cast<int>(neuron.getPos().x() - neuron.getInhibitionRange().at(0)); x < static_cast<int>(neuron.getPos().x() + neuron.getInhibitionRange().at(0)+1); ++x) {
        for (int y = static_cast<int>(neuron.getPos().y() - neuron.getInhibitionRange().at(1)); y < static_cast<int>(neuron.getPos().y() + neuron.getInhibitionRange().at(1) + 1); ++y) {
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
 * @param nbEvents
 */
void SpikingNetwork::updateNeuronsStates(long timeInterval, size_t nbEvents) {
    size_t layer;
    m_averageActivity = 0;
    for (auto &neurons: m_neurons) {
        for (auto &neuron: neurons) {
            neuron.get().updateState(timeInterval, 1);

//            neuron.get().learningDecay(static_cast<double>(nbEvents) / E6);
            if (layer == 0) {
                neuron.get().thresholdAdaptation();
                m_averageActivity += neuron.get().getSpikingRate();
            }
        }
    }
    m_averageActivity /= static_cast<double>(m_neurons[0].size());
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
        std::vector<size_t> data(
                static_cast<size_t>(m_networkConf.getLayerPatches()[layer][0].size() *
                                    m_networkConf.getLayerSizes()[layer][0] *
                                    m_networkConf.getLayerPatches()[layer][1].size() *
                                    m_networkConf.getLayerSizes()[layer][1] *
                                    m_networkConf.getLayerPatches()[layer][2].size() *
                                    m_networkConf.getLayerSizes()[layer][2]));
        count = 0;
        for (size_t i = 0; i < m_networkConf.getLayerPatches()[layer][0].size() * m_networkConf.getLayerSizes()[layer][0]; ++i) {
            for (size_t j = 0; j < m_networkConf.getLayerPatches()[layer][1].size() * m_networkConf.getLayerSizes()[layer][1]; ++j) {
                for (size_t k = 0; k < m_networkConf.getLayerPatches()[layer][2].size() * m_networkConf.getLayerSizes()[layer][2]; ++k) {
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
        for (size_t i = 0; i < m_networkConf.getLayerSizes()[layer][2]; ++i) {
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
        if (std::find(m_networkConf.getLayerInhibitions()[layer].begin(),
                      m_networkConf.getLayerInhibitions()[layer].end(), "lateral") != m_networkConf.getLayerInhibitions()[layer].end()) {
            li = true;
        } else {
            li = false;
        }
        if (std::find(m_networkConf.getLayerInhibitions()[layer].begin(),
                      m_networkConf.getLayerInhibitions()[layer].end(), "topdown") != m_networkConf.getLayerInhibitions()[layer].end()) {
            tdi = true;
        } else {
            tdi = false;
        }

        if (layer == 0 && m_networkConf.getSharingType() == "patch") {
            size_t step = m_networkConf.getLayerSizes()[0][0] * m_networkConf.getLayerSizes()[0][1] * m_networkConf.getLayerSizes()[0][2];
            size_t patch_size = m_networkConf.getLayerPatches()[0][0].size() * m_networkConf.getLayerPatches()[0][1].size();
            for (size_t patch = 0; patch < patch_size; ++patch) {
                for (size_t i = 0; i < m_networkConf.getLayerSizes()[0][2]; ++i) {
                    fileName = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
                    neurons[patch * step + i].get().saveWeights(fileName);
                }
            }
        } else if (layer == 0 && m_networkConf.getSharingType() == "full") {
            for (size_t i = 0; i < m_networkConf.getLayerSizes()[0][2]; ++i) {
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
//        if (std::find(m_networkConf.getLayerInhibitions()[layer].begin(),
//                      m_networkConf.getLayerInhibitions()[layer].end(), "lateral") != m_networkConf.getLayerInhibitions()[layer].end()) {
//            cnpy::npz_save(liFilePath, "", &emptyArray[0], {0}, "w");
//            li = true;
//        }
//        if (std::find(m_networkConf.getLayerInhibitions()[layer].begin(),
//                      m_networkConf.getLayerInhibitions()[layer].end(), "topdown") != m_networkConf.getLayerInhibitions()[layer].end()) {
//            cnpy::npz_save(tdiFilePath, "", &emptyArray[0], {0}, "w");
//            tdi = true;
//        }
//
//        if (layer == 0 && m_networkConf.getSharingType() == "patch") {
//            size_t step = m_networkConf.getLayerSizes()[0][0] * m_networkConf.getLayerSizes()[0][1] * m_networkConf.getLayerSizes()[0][2];
//            size_t patch_size = m_networkConf.getLayerPatches()[0][0].size() * m_networkConf.getLayerPatches()[0][1].size();
//            for (size_t patch = 0; patch < patch_size; ++patch) {
//                for (size_t i = 0; i < m_networkConf.getLayerSizes()[0][2]; ++i) {
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
        if (Util::fileExist(liFilePath)) {
            li = true;
        } else {
            li = false;
        }
        if (Util::fileExist(tdiFilePath)) {
            tdi = true;
        } else {
            tdi = false;
        }
        if (Util::fileExist(path)) {
            if (layer == 0 && m_networkConf.getSharingType() == "patch") {
                size_t step = m_networkConf.getLayerSizes()[0][0] * m_networkConf.getLayerSizes()[0][1] * m_networkConf.getLayerSizes()[0][2];
                size_t patch_size = m_networkConf.getLayerPatches()[0][0].size() * m_networkConf.getLayerPatches()[0][1].size();
                for (size_t patch = 0; patch < patch_size; ++patch) {
                    for (size_t i = 0; i < m_networkConf.getLayerSizes()[0][2]; ++i) {
                        filePath = m_networkConf.getNetworkPath() + "weights/" + std::to_string(layer) + "/";
                        neurons[patch * step + i].get().loadWeights(filePath);
                    }
                }
            } else if (layer == 0 && m_networkConf.getSharingType() == "full") {
                for (size_t i = 0; i < m_networkConf.getLayerSizes()[0][2]; ++i) {
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
//                size_t step = m_networkConf.getLayerSizes()[0][0] * m_networkConf.getLayerSizes()[0][1] * m_networkConf.getLayerSizes()[0][2];
//                size_t patch_size = m_networkConf.getLayerPatches()[0][0].size() * m_networkConf.getLayerPatches()[0][1].size();
//                for (size_t patch = 0; patch < patch_size; ++patch) {
//                    for (size_t i = 0; i < m_networkConf.getLayerSizes()[0][2]; ++i) {
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

/**
 * Updates the statistics of a tracked neuron everytime it receives an event.
 * @param time 
 * @param type_ 
 * @param pos 
 * @param neuron 
 * @param wi 
 * @param spike 
 */
void SpikingNetwork::neuronsStatistics(uint64_t time, int type_, Position pos, Neuron &neuron, double wi, bool spike) {
    if (neuron.getConf().POTENTIAL_TRACK[0] == neuron.getPos().x() && neuron.getConf().POTENTIAL_TRACK[1] == neuron.getPos().y()) {
        neuron.assignToAmountOfEvents(type_);
        if(spike) {
            neuron.assignToPotentialTrain(std::make_pair(neuron.getSpikingPotential(), time));
            neuron.assignToPotentialThreshold();
        }
        neuron.assignToPotentialThreshold();
        neuron.assignToPotentialTrain(std::make_pair(neuron.getPotential(time), time));
        if (type_ != 0) {
            type_ -= 1;
            if (type_ != 2) {
                neuron.assignToSumLateralWeights(type_, pos, wi);
            }
            if(type_ == 2) {
                neuron.assignToSumTopDownWeights(pos.z(),wi,16);
            }
            std::tuple<double, double, uint64_t> var = {neuron.getBeforeInhibitionPotential(), neuron.getPotential(time), time};
            neuron.assignToTimingOfInhibition(type_, var);
        }
        else {
            std::tuple<double,uint64_t> ev = {neuron.getPotential(time), time};
            neuron.assignToExcitatoryEvents(ev);
        }
    }
}

/**
 * Saves the statistics of all tracked neurons while making sure to erase potential old statistics that exists in the folder.
 * @param simulation 
 * @param sequence 
 */
void SpikingNetwork::saveStatistics(int simulation, int sequence) {
    std::string fileName;
    size_t layer = 0;
    std::filesystem::file_status s = std::filesystem::file_status{};
    static bool entered = false;
    for (auto &neurons: m_neurons) {
        if(!entered) {
        std::filesystem::remove_all(m_networkConf.getNetworkPath() + "statistics/" + std::to_string(layer) + "/"); 
        std::filesystem::create_directory(
                    m_networkConf.getNetworkPath() + "statistics/" + std::to_string(layer) + "/");
        entered = true;
        }
        std::string path(m_networkConf.getNetworkPath() + "statistics/" + std::to_string(layer));
        if (std::filesystem::status_known(s) ? std::filesystem::exists(s) : std::filesystem::exists(path)) {
            std::filesystem::create_directory(
                    m_networkConf.getNetworkPath() + "statistics/" + std::to_string(layer) + "/" + std::to_string(simulation));
            std::filesystem::create_directory(
                    m_networkConf.getNetworkPath() + "statistics/" + std::to_string(layer) + "/" + std::to_string(simulation) + "/" + std::to_string(sequence));

            for (auto &neuron: neurons) {
                if (neuron.get().getConf().POTENTIAL_TRACK[0] == neuron.get().getPos().x() &&
                    neuron.get().getConf().POTENTIAL_TRACK[1] == neuron.get().getPos().y()) {
                    fileName = m_networkConf.getNetworkPath() + "statistics/" + std::to_string(layer) + "/" + std::to_string(simulation) + "/" + std::to_string(sequence) + "/";
                    saveStatesStatistics(fileName, neuron.get());
                }
            }
            ++layer;
        }
    }
}

/**
 * Writes the json file of the statistics of a neuron.
 * @param state 
 * @param neuron 
 */
void SpikingNetwork::writeJsonNeuronsStatistics(nlohmann::json &state, Neuron &neuron) {
    state["amount_of_events"] = neuron.getAmountOfEvents();
    state["potentials_thresholds"] = neuron.getPotentialThreshold();
    state["potential_train"] = neuron.getPotentialTrain();
    state["sum_inhib_weights"] = neuron.getSumLateralWeights();
    state["sum_topdown_weights"] = neuron.getSumTopDownWeights();
    state["timing_of_inhibition"] = neuron.getTimingOfInhibition();
    state["excitatory_ev"] = neuron.getExcitatoryEvents();
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

/**
 * Change the coordinates of the neuron to track regardless of what is in the config file.
 * @param n_x 
 * @param n_y 
 */
void SpikingNetwork::changeTrack(int n_x, int n_y) {
    std::vector<double>val{n_x,n_y};
    int layer=0;
    for (auto &neurons: m_neurons) {
            for (auto &neuron: neurons) {
                neuron.get().setPotentialTrack(val);
            }
            ++layer;
            if(layer!=0)
            {
                break;
            }
    }
}

/**
 * Re-initializes the lateral inhibition weights to random values sampled from an uniform distribution.
 */
void SpikingNetwork::randomLateralInhibition() {
    for (auto &neurons: m_neurons) {
        for (auto &neuron: neurons) {
                neuron.get().randomInhibition();
            }
    }
}
/**
 * Shuffles a specific type or all inhibitory weights.
 * @param cases 
 */
void SpikingNetwork::shuffleInhibition(int cases) {
    size_t depth = m_networkConf.getNeuronSizes()[0][2];
    int index = m_simpleNeuronConf.POTENTIAL_TRACK[0] *m_networkConf.getLayerSizes()[0][1]*m_networkConf.getLayerSizes()[0][2] + m_simpleNeuronConf.POTENTIAL_TRACK[1]*m_networkConf.getLayerSizes()[0][2];
    switch (cases) {
        case 1:
            for (auto &neurons: m_neurons[0]) {
                neurons.get().shuffleLateralInhibition();
            }
            break;
        case 2:
            for (auto &neurons: m_neurons[0]) {
                neurons.get().shuffleTopDownInhibition();
            }
            break;
        case 3:
            for (auto &neurons: m_neurons[0]) {
                neurons.get().shuffleLateralInhibition();
                neurons.get().shuffleTopDownInhibition();
            }
            break;
        default:
            std::cout << "No argument passed in \"shuffleInhibition()\". Function aborted." << std::endl;
            break;
    } 
}

/**
 * Assigns an orientation to a simple cell.
 * @param index_z 
 * @param orientation 
 */
void SpikingNetwork::assignOrientations(int index_z, int orientation) {
    m_simpleWeightsOrientations.at(index_z).push_back(orientation);
}

/**
 * Assigns an orientation to a complex cell.
 * @param id 
 * @param orientation 
 */
void SpikingNetwork::assignComplexOrientations(int id, int orientation) {
    m_complexCellsOrientations.at(id).push_back(orientation);
}

/**
 * Saves the orientation of simple and complex cells in a json file.
 */
void SpikingNetwork::saveOrientations() {
    std::string fileName;
    size_t layer = 0;
    std::filesystem::file_status s = std::filesystem::file_status{};
    std::string path(m_networkConf.getNetworkPath() + "statistics/");
    if (std::filesystem::status_known(s) ? std::filesystem::exists(s) : std::filesystem::exists(path)) {
        std::filesystem::create_directory(m_networkConf.getNetworkPath() + "statistics/" + "orientations");
        fileName = m_networkConf.getNetworkPath() + "statistics/" + "orientations" + "/";
        
        nlohmann::json state;
        state["orientations"] = m_simpleWeightsOrientations;
        state["complex_orientations"] = m_complexCellsOrientations;
        std::ofstream ofs(fileName + "orientations.json");
        if (ofs.is_open()) {
            ofs << std::setw(4) << state << std::endl;
        } else {
            std::cout << "cannot save neuron state statistics file" << std::endl;
        }
        ofs.close();
    }
}

/**
 * Resets the spike train of all neurons.
 */
void SpikingNetwork::resetSTrain() {
    size_t layer = 0;
    for (auto &neurons: m_neurons) {
            for (auto &neuron: neurons) {
                neuron.get().resetSpikeTrain();
            }
            ++layer;
    }
}