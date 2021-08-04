#ifndef NEUVISYS_DV_SPIKING_NETWORK_HPP
#define NEUVISYS_DV_SPIKING_NETWORK_HPP

#include <utility>
#include <vector>
#include <array>
#include "neurons/SimpleNeuron.hpp"
#include "neurons/ComplexNeuron.hpp"
#include "neurons/MotorNeuron.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

class SpikingNetwork {
    NetworkConfig m_conf;
    NeuronConfig m_simpleNeuronConf;
    NeuronConfig m_complexNeuronConf;
    NeuronConfig m_motorNeuronConf;
    long m_iterations{};

    double m_reward{};
    std::vector<double> m_listReward;

    std::vector<Eigen::Tensor<double, SIMPLEDIM>> m_sharedWeightsSimple;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsComplex;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsMotor;

    std::vector<std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t>> m_layout;

    std::vector<SimpleNeuron> m_simpleNeurons;
    std::vector<ComplexNeuron> m_complexNeurons;
    std::vector<MotorNeuron> m_motorNeurons;
    std::vector<std::vector<uint64_t>> m_pixelMapping;
    std::vector<uint64_t> m_motorActivation;

    std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t> m_layout1;
    std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t> m_layout2;
    std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t> m_layout3;

    uint64_t m_nbSimpleNeurons{};
    uint64_t m_nbComplexNeurons{};
    uint64_t m_nbMotorNeurons{};

//    Luts m_simpleluts;
//    Luts m_complexluts;
public:
    explicit SpikingNetwork(const std::string &conf);
    void runEvents(const std::vector<Event> &eventPacket, double reward);
    void runEvent(const Event &event);
    void addEvent(const Event &event);
    void updateNeuronsParameters(long time);

    Neuron &getNeuron(size_t index, size_t neuronType);
    [[nodiscard]] size_t getNumberSimpleNeurons() const { return m_nbSimpleNeurons; }
    [[nodiscard]] size_t getNumberComplexNeurons() const { return m_nbComplexNeurons; }
    [[nodiscard]] size_t getNumberMotorNeurons() const { return m_nbMotorNeurons; }
    NetworkConfig getNetworkConfig() { return m_conf; }
    NeuronConfig getSimpleNeuronConfig() { return m_simpleNeuronConf; }
    NeuronConfig getComplexNeuronConfig() { return m_complexNeuronConf; }
    uint64_t getLayout1(uint64_t x, uint64_t y, uint64_t z) { return m_layout1[{x, y, z}]; }
    uint64_t getLayout2(uint64_t x, uint64_t y, uint64_t z) { return m_layout2[{x, y, z}]; }
    std::vector<double> &getRewards() { return m_listReward; }
    void trackNeuron(long time, size_t simpleId = 0, size_t complexId = 0);
    cv::Mat getWeightNeuron(size_t idNeuron, size_t camera, size_t synapse, size_t neuronType, size_t layer);
    const std::vector<long> &getSpikingNeuron(size_t idNeuron, size_t neuronType);
    const std::vector<std::pair<double, long>> &getPotentialNeuron(size_t idNeuron, size_t neuronType);
    void saveNetwork(size_t nbRun, const std::string& eventFileName);
    void setReward(double reward);
    void resetMotorActivation() { std::fill(m_motorActivation.begin(), m_motorActivation.end(), 0); }
    const std::vector<uint64_t> &getMotorActivation() { return m_motorActivation; }

private:
    void addComplexEvent(SimpleNeuron &neuron);
    void addMotorEvent(ComplexNeuron &neuron);
    void updateNeurons(long time);
    void saveNeuronsStates();
    bool simpleNeuronsFilesExists() const;
    bool complexNeuronsFilesExists() const;
    bool motorNeuronsFilesExists() const;
    void loadWeights(bool simpleNeuronStored, bool complexNeuronStored, bool motorNeuronStored);
    void generateWeightSharing(bool simpleNeuronStored, bool complexNeuronStored, bool motorNeuronStored);
    void generateNeuronConfiguration();
    void assignNeurons();
    Position findPixelComplexNeuron(ComplexNeuron &neuron);

    void addLayer(const std::string &neuronType, const std::string &sharingType, const std::vector<std::vector<size_t>> &layerPatches,
                  const std::vector<size_t> &layerSizes, const std::vector<size_t> &neuronSizes);
    void connectLayer(std::vector<Neuron> &prevLayer, std::vector<Neuron> &currLayer, const std::string &neuronType, bool inhibition,
                      size_t xSize, size_t ySize, size_t zSize);
};

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP
