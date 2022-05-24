#include "SimpleNeuron.hpp"

/* Similar to the abstract neuron class.
 * It also takes as input a weight tensor and the number of synapses used when delayed synapses are defined.
 */
SimpleNeuron::SimpleNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset, Eigen::Tensor<double, SIMPLEDIM> &weights, size_t nbSynapses) :
        Neuron(index, layer, conf, pos, offset),
        m_events(boost::circular_buffer<Event>(1000)),
        m_topDownInhibitionEvents(boost::circular_buffer<NeuronEvent>(1000)),
        m_lateralInhibitionEvents(boost::circular_buffer<NeuronEvent>(1000)),
        m_weights(weights),
        m_waitingList(std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp>()) {
    for (size_t synapse = 0; synapse < nbSynapses; synapse++) {
        m_delays.push_back(static_cast<size_t>(synapse * conf.SYNAPSE_DELAY));
    }
}

/* Updates neuron internal state after the arrival of an event
 * Checks first if there is some synaptic delays defined in the network.
 */
inline bool SimpleNeuron::newEvent(Event event) {
    if ((m_delays.size() == 1) && (m_delays[0] == 0)) {
        m_events.push_back(event);
        return membraneUpdate(event);
    } else {
        size_t synapse = 0;
        for (auto delay : m_delays) {
            m_waitingList.emplace(event.timestamp() + delay, event.x(), event.y(), event.polarity(), event.camera(), synapse++);
        }
        return false;
    }
}

void SimpleNeuron::newTopDownInhibitoryEvent(NeuronEvent event, Neuron &neuron) {
    m_topDownInhibitionEvents.push_back(event);
    m_adaptationPotential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / m_conf.TAU_SRA);
    m_potential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / m_conf.TAU_M);
    m_timestampLastEvent = event.timestamp();
    m_potential -= m_topDownInhibitionWeights.at(event.id());
    m_timestampLastEvent = event.timestamp();
    savePotentials(event.timestamp(), 2, neuron, m_topDownInhibitionWeights.at(event.id()));
}

void SimpleNeuron::newLateralInhibitoryEvent(NeuronEvent event, Neuron &neuron) {
    m_counter_lateral+=1;
    m_lateralInhibitionEvents.push_back(event);
    m_potential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / m_conf.TAU_M);
    m_potential -= m_lateralInhibitionWeights.at(event.id());
    m_timestampLastEvent = event.timestamp();
    savePotentials(event.timestamp(), 1, neuron, m_lateralInhibitionWeights.at(event.id()));
}

bool SimpleNeuron::update() {
    Event event = m_waitingList.top();
    m_waitingList.pop();
    m_events.push_back(event);
    return membraneUpdate(event);
}

/* Updates the membrane potential using the newly arrived event.
 * Updates some homeostatic mechanisms such as the refractory period, potential decay and spike rate adaptation.
 * If the membrane potential exceeds the threshold, the neuron spikes.
 */
inline bool SimpleNeuron::membraneUpdate(Event event) {
    m_counter_events+=1;
    m_potential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / m_conf.TAU_M);
    m_adaptationPotential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / m_conf.TAU_SRA);
    m_potential += m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y())
                   - refractoryPotential( event.timestamp() - m_lastSpikingTime )
                   - m_adaptationPotential;
    m_timestampLastEvent = event.timestamp();
    savePotentials(event.timestamp(),-1, *this, 0);

    if (m_potential > m_threshold) {
        spike(event.timestamp());
    //    savePotentials(event.timestamp()+100,-1, *this, 0); 
        return true;
    }
    return false;
}

/* Updates the spike timings and spike counters.
 * Also increases the secondary membrane potential use in the spike rate adaptation mechanism.
 */
inline void SimpleNeuron::spike(const size_t time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_spikeRateCounter;
    ++m_totalSpike;
    m_potential = m_conf.VRESET;

    spikeRateAdaptation();

    if (m_conf.TRACKING == "partial") {
        m_trackingSpikeTrain.push_back(time);
    }
}

/* Updates the synaptic weights using the STDP learning rule.
 * Only the synapses from which events arrived are updated.
 * Normalizes the weights after the update.
 */
inline void SimpleNeuron::weightUpdate() {
    if (m_conf.STDP_LEARNING == "excitatory" || m_conf.STDP_LEARNING == "all") {
        for (Event &event: m_events) {
            m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) +=
                    m_decay * m_conf.ETA_LTP * exp(-static_cast<double>(m_spikingTime - event.timestamp()) / m_conf.TAU_LTP);
            m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) +=
                    m_decay * m_conf.ETA_LTD * exp(-static_cast<double>(event.timestamp() - m_lastSpikingTime) / m_conf.TAU_LTD);
            if (m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) < 0) {
                m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) = 0;
            }
        }
        normalizeWeights();
    }
    if (m_conf.STDP_LEARNING == "inhibitory" || m_conf.STDP_LEARNING == "all") {
        for (NeuronEvent &event : m_topDownInhibitionEvents) {
            m_topDownInhibitionWeights.at(event.id()) += m_conf.ETA_ILTP * exp(- static_cast<double>(m_spikingTime - event.timestamp() - m_offset_inhib) / m_conf.TAU_LTP);
            m_topDownInhibitionWeights.at(event.id()) += m_conf.ETA_ILTD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime + m_offset_inhib) / m_conf.TAU_LTD);
            if (m_topDownInhibitionWeights.at(event.id()) < 0) {
                m_topDownInhibitionWeights.at(event.id()) = 0;
            }
        }

        for (NeuronEvent &event : m_lateralInhibitionEvents) {
            m_lateralInhibitionWeights.at(event.id()) += m_conf.ETA_ILTP * exp(- static_cast<double>(m_spikingTime - event.timestamp() - m_offset_inhib) / m_conf.TAU_LTP);
            m_lateralInhibitionWeights.at(event.id()) += m_conf.ETA_ILTD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime + m_offset_inhib) / m_conf.TAU_LTD);
            if (m_lateralInhibitionWeights.at(event.id()) < 0) {
                m_lateralInhibitionWeights.at(event.id()) = 0;
            }
        }
        normalizeInhibWeights();
    }
    m_events.clear();
    m_topDownInhibitionEvents.clear();
    m_lateralInhibitionEvents.clear();
}

/* Weight normalization using tensor calculations.
 */
inline void SimpleNeuron::normalizeWeights() {
    const Eigen::Tensor<double, SIMPLEDIM>::Dimensions& d = m_weights.dimensions();

//    for (long p = 0; p < d[0]; ++p) {
//        for (int c = 0; c < d[1]; ++c) {
//            for (long s = 0; s < d[2]; ++s) {
//                Eigen::array<long, SIMPLEDIM> start = {p, c, s, 0, 0};
//                Eigen::array<long, SIMPLEDIM> size = {1, 1, 1, d[3], d[4]};
//                Eigen::Tensor<double, 0> norm = m_weights.slice(start, size).pow(2).sum().sqrt();
//
//                if (norm(0) != 0) {
//                    m_weights.slice(start, size) = m_conf.NORM_FACTOR * m_weights.slice(start, size) / norm(0);
//                }
//            }
//        }
//    }

    // weight normalization on the camera axes
//    for (long c = 0; c < d[1]; ++c) {
//        Eigen::array<long, SIMPLEDIM> start = {0, c, 0, 0, 0};
//        Eigen::array<long, SIMPLEDIM> size = {d[0], 1, d[2], d[3], d[4]};
//        Eigen::Tensor<double, 0> norm = m_weights.slice(start, size).pow(2).sum().sqrt();
//
//        if (norm(0) != 0) {
//            m_weights.slice(start, size) = m_conf.NORM_FACTOR * m_weights.slice(start, size) / norm(0);
//        }
//    }

     Eigen::Tensor<double, 0> norm = m_weights.pow(2).sum().sqrt();
 //   Eigen::Tensor<double, 0> norm = m_weights.sum();

    if (norm(0) != 0) {
        m_weights = m_conf.NORM_FACTOR * m_weights / norm(0);
    }

    

}

void SimpleNeuron::normalizeInhibWeights(){
    double norm_lateral = 0;
    for (auto neuron : m_lateralDynamicInhibitionConnections) 
    {
        norm_lateral+=m_lateralInhibitionWeights.at(neuron.get().getIndex())*m_lateralInhibitionWeights.at(neuron.get().getIndex());
    }
    norm_lateral = sqrt(norm_lateral);

    if(norm_lateral!=0)
    {
        for (auto neuron : m_lateralDynamicInhibitionConnections) 
        {
            m_lateralInhibitionWeights.at(neuron.get().getIndex())= m_conf.LATERAL_NORM_FACTOR * (m_lateralInhibitionWeights.at(neuron.get().getIndex()) / norm_lateral);
        }
    }

    double norm_topdown = 0;
    for (auto neuron : m_topDownDynamicInhibitionConnections) 
    {
        norm_topdown+=m_topDownInhibitionWeights.at(neuron.get().getIndex())*m_topDownInhibitionWeights.at(neuron.get().getIndex());
    }
    norm_topdown = sqrt(norm_topdown);

    if(norm_topdown!=0)
    {
        for (auto neuron : m_topDownDynamicInhibitionConnections) 
        {
            m_topDownInhibitionWeights.at(neuron.get().getIndex())= m_conf.TOPDOWN_NORM_FACTOR * (m_topDownInhibitionWeights.at(neuron.get().getIndex()) / norm_topdown);
        }
    }
}

inline void SimpleNeuron::savePotentials(uint64_t time, int type_, Neuron &neuron, double wi){
    /*type_ = 0 for static inhibition ; 
    type_ = 1 for lateral inhibition ; 
    type_ = 2 for topdown inhibition ;*/
    if (m_conf.POTENTIAL_TRACK[0] == m_pos.x() && m_conf.POTENTIAL_TRACK[1] == m_pos.y()) {
            m_trackingPotentialTrain.emplace_back(m_potential, time);
            if(type_==0)
            {
                m_inhibitionIndex[type_].emplace_back(m_counter);
                if(m_length_of_sequence.size()!=0)
                {
                    for(int j=0; j<m_length_of_sequence.size(); j++)
                    {
                        m_inhibitionIndex[type_][m_inhibitionIndex[type_].size()-1]+=m_length_of_sequence[j];
                    }                    
                }
                m_inhibWeightsStatLateralTopDown[type_].emplace_back(m_potential+wi,time);
            }
            else if(type_==1)
            {
                m_inhibitionIndex[type_].emplace_back(m_counter);
                if(m_length_of_sequence.size()!=0)
                {
                    for(int j=0; j<m_length_of_sequence.size(); j++)
                    {
                        m_inhibitionIndex[type_][m_inhibitionIndex[type_].size()-1]+=m_length_of_sequence[j];
                    }                    
                }
                m_inhibWeightsStatLateralTopDown[type_].emplace_back(m_potential+wi,time);
                if(neuron.getPos().x()==m_pos.x()-1 && neuron.getPos().y()==m_pos.y()-1)
                {
                    m_sumOfInhibWeightsLateral[m_sumOfInhibWeightsLateral.size()-1][0]+=wi;
                }
                else if(neuron.getPos().x()==m_pos.x()-1 && neuron.getPos().y()==m_pos.y())
                {
                    m_sumOfInhibWeightsLateral[m_sumOfInhibWeightsLateral.size()-1][1]+=wi;
                }
                else if(neuron.getPos().x()==m_pos.x()-1 && neuron.getPos().y()==m_pos.y()+1)
                {
                    m_sumOfInhibWeightsLateral[m_sumOfInhibWeightsLateral.size()-1][2]+=wi;
                }
                else if(neuron.getPos().x()==m_pos.x() && neuron.getPos().y()==m_pos.y()-1)
                {
                    m_sumOfInhibWeightsLateral[m_sumOfInhibWeightsLateral.size()-1][3]+=wi;
                }
                else if(neuron.getPos().x()==m_pos.x() && neuron.getPos().y()==m_pos.y()+1)
                {
                    m_sumOfInhibWeightsLateral[m_sumOfInhibWeightsLateral.size()-1][4]+=wi;
                }
                else if(neuron.getPos().x()==m_pos.x()+1 && neuron.getPos().y()==m_pos.y()-1)
                {
                    m_sumOfInhibWeightsLateral[m_sumOfInhibWeightsLateral.size()-1][5]+=wi;
                }
                else if(neuron.getPos().x()==m_pos.x()+1 && neuron.getPos().y()==m_pos.y())
                {
                    m_sumOfInhibWeightsLateral[m_sumOfInhibWeightsLateral.size()-1][6]+=wi;
                }
                else if(neuron.getPos().x()==m_pos.x()+1 && neuron.getPos().y()==m_pos.y()+1)
                {
                    m_sumOfInhibWeightsLateral[m_sumOfInhibWeightsLateral.size()-1][7]+=wi;
                }
            }
            else if(type_==2)
            {
                m_inhibitionIndex[type_].emplace_back(m_counter);
                if(m_length_of_sequence.size()!=0)
                {
                    for(int j=0; j<m_length_of_sequence.size(); j++)
                    {
                        m_inhibitionIndex[type_][m_inhibitionIndex[type_].size()-1]+=m_length_of_sequence[j];
                    }                    
                }
                m_inhibWeightsStatLateralTopDown[type_].emplace_back(m_potential+wi,time);
            }
            m_potentialThreshold.emplace_back(m_threshold);
            m_counter+=1;
        }
}

void SimpleNeuron::saveWeights(std::string &fileName) {
    auto weightsFile = fileName + std::to_string(m_index);
    Util::saveSimpleTensorToNumpyFile(m_weights, weightsFile);
}

void SimpleNeuron::saveInhibWeights(std::string &fileName) {
    auto weightsFile = fileName + std::to_string(m_index) + "tdi";
    Util::saveWeightsToNumpy(m_topDownInhibitionWeights, weightsFile);
    weightsFile = fileName + std::to_string(m_index) + "li";
    Util::saveWeightsToNumpy(m_lateralInhibitionWeights, weightsFile);
}

void SimpleNeuron::loadWeights(std::string &fileName) {
    auto weightsFile = fileName + std::to_string(m_index);
    Util::loadNumpyFileToSimpleTensor(m_weights, weightsFile);
}

void SimpleNeuron::loadInhibWeights(std::string &fileName) {
    auto weightsFile = fileName + std::to_string(m_index) + "tdi";
    Util::loadNumpyToWeights(m_topDownInhibitionWeights, weightsFile);
    weightsFile = fileName + std::to_string(m_index) + "li";
    Util::loadNumpyToWeights(m_lateralInhibitionWeights, weightsFile);
}

std::vector<long> SimpleNeuron::getWeightsDimension() {
    const Eigen::Tensor<double, SIMPLEDIM>::Dimensions& dimensions = m_weights.dimensions();
    std::vector<long> dim = { dimensions[3], dimensions[4] };
    return dim;
}
