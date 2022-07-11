//
// Created by Thomas on 14/04/2021.
//

#ifndef NEUVISYS_DV_SPIKING_NETWORK_HPP
#define NEUVISYS_DV_SPIKING_NETWORK_HPP

#include <utility>
#include <vector>
#include <array>
#include <stack>
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

    std::vector<Eigen::Tensor<double, SIMPLEDIM>> m_sharedWeightsSimple;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsInhib;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsComplex;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsCritic;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsActor;
    std::vector<std::vector<std::vector<double>>> m_gaborParameters;

    std::vector<std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t>> m_layout;
    std::vector<size_t> m_structure;
    std::vector<std::vector<std::reference_wrapper<Neuron>>> m_neurons;

    std::vector<SimpleNeuron> m_simpleNeurons;
    std::vector<ComplexNeuron> m_complexNeurons;
    std::vector<MotorNeuron> m_criticNeurons;
    std::vector<MotorNeuron> m_actorNeurons;
    std::vector<std::vector<uint64_t>> m_pixelMapping;

    std::vector<std::vector<int>> m_simpleWeightsOrientations;
    std::vector<std::vector<int>> m_complexCellsOrientations;

    double m_reward{};
    double m_averageActivity{};

public:
    SpikingNetwork();

    explicit SpikingNetwork(const std::string &networkPath);

    void addLayer(const std::string &neuronType, const std::string &sharingType,
                  const std::vector<std::string> &inhibition,
                  const std::vector<std::vector<size_t>> &layerPatches,
                  const std::vector<size_t> &layerSizes, const std::vector<size_t> &neuronSizes,
                  const std::vector<size_t> &neuronOverlap, size_t layerToConnect);

    void addEvent(const Event &event);

    void updateNeuronsStates(long timeInterval, size_t nbEvents);

    void loadWeights();

    void saveNetwork();

    double computeNeuromodulator(long time);

    void transmitReward(double reward);

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


private:
    void updateNeurons(long time);

    void saveNeuronsStates();

    static void neuronsStatistics(uint64_t time, int type_, Position pos, Neuron &neuron, double wi, bool spike=false);

    static void saveStatesStatistics(std::string &fileName, Neuron &neuron);

    static void writeJsonNeuronsStatistics(nlohmann::json &state, Neuron &neuron);

    void generateWeightSharing(const std::string &neuronType, const std::vector<size_t> &neuronSizes, size_t nbNeurons);

    void addNeuronEvent(const Neuron &neuron);

    void connectLayer(size_t layerToConnect, const std::vector<std::string> &inhibition,
                      const std::vector<std::vector<size_t>> &layerPatches, const std::vector<size_t> &layerSizes,
                      const std::vector<size_t> &neuronSizes);

    static void topDownDynamicInhibition(Neuron &neuron);

    static void lateralStaticInhibition(Neuron &neuron);

    static void lateralDynamicInhibition(Neuron &neuron);

    void neuromodulation(Neuron &neuron);

    void topDownConnection(Neuron &neuron, size_t currLayer, const std::vector<std::string> &inhibition,
                           size_t layerToConnect, const std::vector<size_t> &neuronSizes);

    void lateralStaticInhibitionConnection(Neuron &neuron, size_t currLayer, const std::vector<size_t> &layerSizes);

    void lateralDynamicInhibitionConnection(Neuron &neuron, size_t currLayer,
                                            const std::vector<std::vector<size_t>> &layerPatches,
                                            const std::vector<size_t> &layerSizes);

    void saveNetworkLayout();

};

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP
