//
// Created by Thomas on 14/04/2021.
//

#ifndef NEUVISYS_DV_NEURON_HPP
#define NEUVISYS_DV_NEURON_HPP

#include "../Config.hpp"
#include "../../utils/Util.hpp"
#include "../../utils/WeightMap.hpp"
#include <cmath>
#include <list>
#include <iomanip>
#include <opencv2/core/mat.hpp>

/* Abstract class defining a Neuron.
 */
class Neuron {
protected:
    size_t m_index;
    size_t m_layer;
    NeuronConfig &m_conf;
    Position m_pos{};
    Position m_offset{};
    WeightMap m_weights;
    WeightMap m_topDownInhibitionWeights;
    WeightMap m_lateralInhibitionWeights;
    std::vector<std::reference_wrapper<Neuron>> m_outConnections;
    std::vector<std::reference_wrapper<Neuron>> m_inConnections;
    std::vector<std::reference_wrapper<Neuron>> m_lateralStaticInhibitionConnections;
    std::vector<std::reference_wrapper<Neuron>> m_topDownDynamicInhibitionConnections;
    std::vector<std::reference_wrapper<Neuron>> m_lateralDynamicInhibitionConnections;
    int m_range_x;
    int m_range_y;
    size_t m_spikingTime{};
    size_t m_lastSpikingTime{};
    size_t m_totalSpike{};
    size_t m_spikeRateCounter{};
    size_t m_activityCounter{};
    double m_decay;
    double m_potential{};
    double m_adaptationPotential{};
    double m_threshold;
    size_t m_timestampLastEvent{};
    bool m_spike;
    size_t m_lifeSpan{};
    double m_spikingRateAverage{};
    std::vector<size_t> m_trackingSpikeTrain;
    std::vector<std::pair<double, uint64_t>> m_trackingPotentialTrain;
    std::vector<double> m_potentialThreshold;
    std::vector<size_t> m_amount_of_events;
    std::vector<std::vector<double>> m_sumOfInhibWeights;
    std::vector<std::vector<std::tuple<double, double, uint64_t>>> m_timingOfInhibition;

    void writeJson(nlohmann::json &state);

    void readJson(const nlohmann::json &state);

    virtual void potentialDecay(size_t time);

    virtual double refractoryPotential(size_t time);

    virtual void adaptationPotentialDecay(size_t time);

    virtual void spikeRateAdaptation();

private:
    virtual void spike(size_t time) {};

public:
    Neuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset);

    [[nodiscard]] virtual size_t getIndex() const { return m_index; }

    [[nodiscard]] virtual size_t getLayer() const { return m_layer; }

    [[nodiscard]] virtual Position getPos() const { return m_pos; }

    [[nodiscard]] virtual Position getOffset() const { return m_offset; }

    [[nodiscard]] virtual double getThreshold() const { return m_threshold; }

    [[nodiscard]] virtual double getSpikingRate() const { return m_spikingRateAverage; }

    [[nodiscard]] virtual size_t getSpikingTime() const { return m_spikingTime; }

    [[nodiscard]] virtual double getDecay() const { return m_decay; }

    [[nodiscard]] virtual double getAdaptationPotential() const { return m_adaptationPotential; }

    [[nodiscard]] virtual size_t getActivityCount();

    virtual void resetActivityCount();

    [[nodiscard]] virtual NeuronConfig getConf() const { return m_conf; }

    virtual double getTopDownInhibitionWeights(size_t neuronId) { return m_topDownInhibitionWeights.at(neuronId); }

    virtual double getlateralInhibitionWeights(size_t neuronId) { return m_lateralInhibitionWeights.at(neuronId); }

    [[nodiscard]] virtual WeightMap &getWeightsMap() { return m_weights; }

    virtual WeightMatrix &getWeightsMatrix() {};

    virtual std::vector<size_t> getWeightsDimension() { return m_weights.getDimensions(); }

    virtual void weightUpdate() {};

    virtual cv::Mat summedWeightMatrix() {};

    virtual void resetSpike() { m_spike = false; }

    [[nodiscard]] virtual std::vector<std::reference_wrapper<Neuron>>
    getOutConnections() const { return m_outConnections; }

    [[nodiscard]] virtual std::vector<std::reference_wrapper<Neuron>>
    getInConnections() const { return m_inConnections; }

    [[nodiscard]] virtual std::vector<std::reference_wrapper<Neuron>>
    getLateralStaticInhibitionConnections() const { return m_lateralStaticInhibitionConnections; }

    [[nodiscard]] virtual std::vector<std::reference_wrapper<Neuron>>
    getTopDownDynamicInhibitionConnections() const { return m_topDownDynamicInhibitionConnections; }

    [[nodiscard]] virtual std::vector<std::reference_wrapper<Neuron>>
    getLateralDynamicInhibitionConnections() const { return m_lateralDynamicInhibitionConnections; }

    virtual const std::vector<size_t> &getTrackingSpikeTrain() { return m_trackingSpikeTrain; }

    virtual const std::vector<std::pair<double, size_t>> &getTrackingPotentialTrain() { return m_trackingPotentialTrain; }

    virtual bool hasSpiked();

    virtual double getPotential(size_t time);

    virtual void saveState(std::string &filePath);

    virtual void loadState(std::string &filePath);

    virtual void saveWeights(const std::string &filePath) {};

    virtual void saveLateralInhibitionWeights(std::string &filePath) {};

    virtual void assignToPotentialTrain(std::pair<double, uint64_t> potential);

    virtual void assignToPotentialThreshold();

    virtual void assignToAmountOfEvents(int type);

    virtual void assignToSumInhibWeights(int type, Position pos, double wi);

    virtual void assignToTimingOfInhibition(int type, std::tuple<double, double, uint64_t> variation);

    virtual std::vector<std::pair<double, uint64_t>> getPotentialTrain();

    virtual std::vector<double> getPotentialThreshold();

    virtual std::vector<size_t> getAmountOfEvents();

    virtual std::vector<std::vector<double>> getSumInhibWeights();

    virtual std::vector<std::vector<std::tuple<double, double, uint64_t>>> getTimingOfInhibition();

    virtual void saveTopDownInhibitionWeights(std::string &filePath) {};

    virtual void loadWeights(std::string &filePath) {};

    virtual void loadLateralInhibitionWeights(std::string &filePath) {};

    virtual void loadTopDownInhibitionWeights(std::string &filePath) {};

    virtual void loadWeights(cnpy::npz_t &arrayNPZ) {};

    virtual void loadLateralInhibitionWeights(cnpy::npz_t &arrayNPZ) {};

    virtual void loadTopDownInhibitionWeights(cnpy::npz_t &arrayNPZ) {};

    virtual void thresholdAdaptation();

    virtual void addOutConnection(Neuron &neuron);

    virtual void addInConnection(Neuron &neuron);

    virtual void initInWeights(size_t id);

    virtual void addTopDownDynamicInhibitionConnection(Neuron &neuron);

    virtual void initTopDownDynamicInhibitionWeights(size_t id);

    virtual void addLateralStaticInhibitionConnections(Neuron &neuron);

    virtual void addLateralDynamicInhibitionConnections(Neuron &neuron);

    virtual void initLateralDynamicInhibitionWeights(size_t id);

    virtual bool newEvent(Event event) {};

    virtual bool newEvent(NeuronEvent event) {};

    virtual void newStaticInhibitoryEvent(NeuronEvent event);

    virtual void newTopDownInhibitoryEvent(NeuronEvent event) {};

    virtual void newLateralInhibitoryEvent(NeuronEvent event) {};

    virtual bool update() {};

    virtual void setNeuromodulator(double neuromodulator) {};

    virtual void trackPotential(size_t time);

    virtual void updateState(size_t timeInterval);

    virtual double updateKernelSpikingRate(long time) {};

    virtual void rescaleWeights(double scale) {};

    virtual void learningDecay(double count);

    virtual void resetNeuron();
};

#endif //NEUVISYS_DV_NEURON_HPP
