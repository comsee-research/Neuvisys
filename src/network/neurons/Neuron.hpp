#ifndef NEUVISYS_DV_NEURON_HPP
#define NEUVISYS_DV_NEURON_HPP

#include "../Config.hpp"
#include "../Utils.hpp"
#include "../Event.hpp"
#include <boost/circular_buffer.hpp>
#include <cmath>
#include <list>
#include <iomanip>
#include <opencv2/core/mat.hpp>

class Neuron {
protected:
    size_t m_index;
    size_t m_layer;
    NeuronConfig &conf;
    Position m_pos{};
    Position m_offset{};
    std::list<size_t> m_recentSpikes;
    std::vector<std::reference_wrapper<Neuron>> m_outConnections;
    std::vector<std::reference_wrapper<Neuron>> m_inConnections;
    std::vector<std::reference_wrapper<Neuron>> m_inhibitionConnections;
    long m_spikingTime{};
    long m_lastSpikingTime{};
    size_t m_totalSpike{};
    size_t m_countSpike{};
    double m_learningDecay;
    double m_potential{};
    double m_adaptation_potential{};
    double m_threshold;
    long m_timestampLastEvent{};
    bool m_spike;
    long m_creationTime{};
    double m_recentSpikeRate{};
    double m_spikingRate{};
    long m_lifeSpan{};
    long m_referenceTime{};

    std::vector<double> m_trackingThresholds;
    std::vector<long> m_trackingSpikeTrain;
    std::vector<std::pair<double, long>> m_trackingPotentialTrain;

public:
    Neuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset);
    [[nodiscard]] virtual size_t getIndex() const { return m_index; }
    [[nodiscard]] virtual size_t getLayer() const { return m_layer; }
    [[nodiscard]] virtual Position getPos() const { return m_pos; }
    [[nodiscard]] virtual Position getOffset() const { return m_offset; }
    [[nodiscard]] virtual double getThreshold() const { return m_threshold; }
    [[nodiscard]] virtual double getSpikingRate() const { return m_spikingRate; }
    [[nodiscard]] virtual long getSpikingTime() const { return m_spikingTime; }
    [[nodiscard]] virtual double getLearningDecay() const { return m_learningDecay; }
    [[nodiscard]] virtual double getAdaptationPotential() const { return m_adaptation_potential; }
    [[nodiscard]] virtual size_t getSpikeCount() const { return m_countSpike; }
    virtual double getWeights(long x, long y, long z) {};
    virtual double getWeights(long p, long c, long s, long x, long y) {};
    virtual std::vector<long> getWeightsDimension() {};

    virtual void weightUpdate() {};
    virtual cv::Mat summedWeightMatrix() {};
    virtual void resetSpikeCounter() { m_countSpike = 0; }
    virtual void resetSpike() { m_spike = false; }
    [[nodiscard]] virtual std::vector<std::reference_wrapper<Neuron>> getOutConnections() const { return m_outConnections; }
    [[nodiscard]] virtual std::vector<std::reference_wrapper<Neuron>> getInConnections() const { return m_inConnections; }
    [[nodiscard]] virtual std::vector<std::reference_wrapper<Neuron>> getInhibitionConnections() const { return m_inhibitionConnections; }
    virtual const std::vector<double> &getTrackingThresholds() { return m_trackingThresholds; }
    virtual const std::vector<long> &getTrackingSpikeTrain() { return m_trackingSpikeTrain; }
    virtual const std::vector<std::pair<double, long>> &getTrackingPotentialTrain() { return m_trackingPotentialTrain; }
    virtual bool hasSpiked();
    virtual double getPotential(long time);
    virtual double potentialDecay(long time);
    virtual double refractoryPotential(long time);
    virtual double adaptationPotentialDecay(long time);
    virtual void inhibition();
    virtual void saveState(std::string &fileName);
    virtual void loadState(std::string &fileName);
    virtual void saveWeights(std::string &fileName) {};
    virtual void loadWeights(std::string &fileName) {};
    virtual void normalizeWeights() {};
    virtual void thresholdAdaptation();
    virtual void spikeRateAdaptation();
    virtual void addOutConnection(Neuron &neuron) { m_outConnections.emplace_back(neuron); }
    virtual void addInConnection(Neuron &neuron) { m_inConnections.emplace_back(neuron); }
    virtual void addInhibitionConnection(Neuron &neuron) { m_inhibitionConnections.emplace_back(neuron); }
    virtual bool newEvent(Event event) {}
    virtual bool newEvent(NeuronEvent event) {}
    virtual bool update() {}
    virtual void setNeuromodulator(double neuromodulator) {}
    virtual void trackPotential(long time);
    virtual void updateState(long time);
    virtual void spike(long time) {};
    virtual double updateKernelSpikingRate(double time) {};
    virtual double computeNormWeights() {};
    virtual void rescaleWeights(double scale) {};

protected:
    void writeJson(json &state);
    void readJson(const json &state);
};

#endif //NEUVISYS_DV_NEURON_HPP
