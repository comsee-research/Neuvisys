//
// Created by Thomas on 14/04/2021.
//

#ifndef NEUVISYS_DV_SPIKING_NETWORK_HPP
#define NEUVISYS_DV_SPIKING_NETWORK_HPP

#include <utility>
#include "neurons/SimpleNeuron.hpp"
#include "neurons/ComplexNeuron.hpp"
#include "neurons/MotorNeuron.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

class SpikingNetwork {
    NetworkConfig m_networkConf;
    NeuronConfig m_simpleNeuronConf;
    NeuronConfig m_complexNeuronConf;
    NeuronConfig m_criticNeuronConf;
    NeuronConfig m_actorNeuronConf;

    std::vector<WeightMatrix> m_sharedWeightsSimple;

    std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp> m_eventsList;
    long m_lastEventTs;

    std::vector<std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t>> m_layout;
    std::vector<size_t> m_structure;
    std::vector<std::vector<std::reference_wrapper<Neuron>>> m_neurons;

    std::vector<SimpleNeuron> m_simpleNeurons;
    std::vector<ComplexNeuron> m_complexNeurons;
    std::vector<MotorNeuron> m_criticNeurons;
    std::vector<MotorNeuron> m_actorNeurons;
    std::vector<std::vector<uint64_t>> m_pixelMapping;

    double m_neuromodulator{};
    std::vector<std::vector<int>> m_simpleWeightsOrientations;
    std::vector<std::vector<int>> m_complexCellsOrientations;
    double m_averageActivity{};

public:
    SpikingNetwork();

    explicit SpikingNetwork(const std::string &networkPath);

    void addLayer(const std::string &sharingType, const LayerConnectivity &connections);

    void addEvent(const Event &event);

    void updateNeuronsStates(long timeInterval);

    void loadWeights();

    void saveNetwork();

    void transmitNeuromodulator(double neuromodulator);

    void normalizeActions();

    [[nodiscard]] double getAverageActivity() const { return m_averageActivity; }

    std::reference_wrapper<Neuron> &getNeuron(size_t index, size_t layer);

    const std::vector<size_t> &getNetworkStructure() { return m_structure; }

    std::vector<std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t>> &getLayout() { return m_layout; }

    void intermediateSave(size_t saveCount);

    void saveStatistics(int simulation, int sequence);

    void changeTrack(int n_x, int n_y);

    void randomLateralInhibition();

    void shuffleInhibition(int cases);

    void assignOrientations(int index_z, int orientation);

    void assignComplexOrientations(int id, int orientation);

    void saveOrientations();

    void resetSTrain();

    void processSynapticEvent();


private:
    void updateMultiSynapticNeurons(long time);

    void saveNeuronsStates();

    static void neuronsStatistics(uint64_t time, int type_, Position pos, Neuron &neuron, double wi, bool spike=false);

    static void saveStatesStatistics(std::string &fileName, Neuron &neuron);

    static void writeJsonNeuronsStatistics(nlohmann::json &state, Neuron &neuron);

    void generateWeightSharing(const LayerConnectivity &connections, size_t nbNeurons);

    void addNeuronEvent(const Neuron &neuron);

    void connectLayer(const LayerConnectivity &connections);

    static void topDownDynamicInhibition(Neuron &neuron);

    static void lateralStaticInhibition(Neuron &neuron);

    static void lateralDynamicInhibition(Neuron &neuron);

    void neuromodulation(Neuron &neuron);

    void topDownConnection(Neuron &neuron, const std::vector<int> &interConnections, const size_t currLayer, const std::vector<std::vector<size_t>> &neuronSizes, const
    std::vector<std::string> &inhibition);

    void lateralStaticInhibitionConnection(Neuron &neuron, size_t currLayer, const std::vector<size_t> &layerSizes);

    void lateralDynamicInhibitionConnection(Neuron &neuron, size_t currLayer,
                                            const std::vector<std::vector<size_t>> &layerPatches,
                                            const std::vector<size_t> &layerSizes);

    void saveNetworkLayout();
};

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP
