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

/* Abstract class defining a Neuron.
 */
class Neuron {
protected:
    size_t m_index;
    size_t m_layer;
    NeuronConfig &conf;
    Position m_pos{};
    Position m_offset{};
    std::vector<std::reference_wrapper<Neuron>> m_outConnections;
    std::vector<std::reference_wrapper<Neuron>> m_inConnections;
    std::vector<std::reference_wrapper<Neuron>> m_inhibitionConnections;
    long m_spikingTime{};
    long m_lastSpikingTime{};
    size_t m_totalSpike{};
    size_t m_spikeRateCounter{};
    size_t m_activityCounter{};
    double m_decay;
    double m_potential{};
    double m_adaptationPotential{};
    double m_threshold;
    long m_timestampLastEvent{};
    bool m_spike;
    long m_lifeSpan{};

    double m_spikingRateAverage{};
    std::vector<long> m_trackingSpikeTrain;
    std::vector<std::pair<double, long>> m_trackingPotentialTrain;

public:
    Neuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset);
    [[nodiscard]] virtual size_t getIndex() const { return m_index; }
    [[nodiscard]] virtual size_t getLayer() const { return m_layer; }
    [[nodiscard]] virtual Position getPos() const { return m_pos; }
    [[nodiscard]] virtual Position getOffset() const { return m_offset; }
    [[nodiscard]] virtual double getThreshold() const { return m_threshold; }
    [[nodiscard]] virtual double getSpikingRate() const { return m_spikingRateAverage; }
    [[nodiscard]] virtual long getSpikingTime() const { return m_spikingTime; }
    [[nodiscard]] virtual double getDecay() const { return m_decay; }
    [[nodiscard]] virtual double getAdaptationPotential() const { return m_adaptationPotential; }
    [[nodiscard]] virtual size_t getActivityCount();
    virtual double getWeights(long x, long y, long z) {};
    virtual double getInhibWeights(long x, long y, long z) {};
    virtual double getWeights(long p, long c, long s, long x, long y) {};
    virtual std::vector<long> getWeightsDimension() {};

    virtual void weightUpdate() {};
    virtual cv::Mat summedWeightMatrix() {};
    virtual void resetSpike() { m_spike = false; }
    [[nodiscard]] virtual std::vector<std::reference_wrapper<Neuron>> getOutConnections() const { return m_outConnections; }
    [[nodiscard]] virtual std::vector<std::reference_wrapper<Neuron>> getInConnections() const { return m_inConnections; }
    [[nodiscard]] virtual std::vector<std::reference_wrapper<Neuron>> getInhibitionConnections() const { return m_inhibitionConnections; }
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
    virtual void saveInhibWeights(std::string &fileName) {};
    virtual void loadWeights(std::string &fileName) {};
    virtual void loadInhibWeights(std::string &fileName) {};
    virtual void normalizeWeights() {};
    virtual void thresholdAdaptation();
    virtual void spikeRateAdaptation();
    virtual void addOutConnection(Neuron &neuron) { m_outConnections.emplace_back(neuron); }
    virtual void addInConnection(Neuron &neuron) { m_inConnections.emplace_back(neuron); }
    virtual void addInhibitionConnection(Neuron &neuron) { m_inhibitionConnections.emplace_back(neuron); }
    virtual bool newEvent(Event event) {};
    virtual bool newEvent(NeuronEvent event) {};
    virtual void newInhibitoryEvent(NeuronEvent event) {};
    virtual bool update() {};
    virtual void setNeuromodulator(double neuromodulator) {};
    virtual void trackPotential(long time);
    virtual void updateState(long timeInterval, double alpha);
    virtual void spike(long time) {};
    virtual double updateKernelSpikingRate(double time) {};
    virtual double computeNormWeights() {};
    virtual void rescaleWeights(double scale) {};
    virtual void learningDecay(double count);

protected:
    void writeJson(json &state);
    void readJson(const json &state);
};

#endif //NEUVISYS_DV_NEURON_HPP
