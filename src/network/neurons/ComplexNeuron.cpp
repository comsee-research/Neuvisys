#include "ComplexNeuron.hpp"

ComplexNeuron::ComplexNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset, Eigen::Tensor<double, 3> &weights) :
    Neuron(index, layer, conf, pos, offset),
    m_events(boost::circular_buffer<NeuronEvent>(1000)),
    m_weights(weights) {
}

inline bool ComplexNeuron::newEvent(NeuronEvent event) {
    m_events.push_back(event);
    return membraneUpdate(event);
}

inline bool ComplexNeuron::membraneUpdate(NeuronEvent event) {
    m_potential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / m_conf.TAU_M);
//    potentialDecay(event.timestamp() - m_timestampLastEvent);

    m_potential += m_weights(event.x(), event.y(), event.z())
                   - refractoryPotential(event.timestamp() - m_spikingTime);
                   //- m_adaptationPotential;
    m_timestampLastEvent = event.timestamp();
    m_counter_events+=1;
    savePotentials(event.timestamp(),-1, *this, 0);
    if (m_potential > m_threshold) {
        spike(event.timestamp());
        return true;
    }
    return false;
}

inline void ComplexNeuron::spike(const size_t time) {
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

inline void ComplexNeuron::weightUpdate() {
    if (m_conf.STDP_LEARNING == "excitatory" || m_conf.STDP_LEARNING == "all") {
        for (NeuronEvent &event : m_events) {
            if (static_cast<double>(m_spikingTime - event.timestamp()) < m_conf.TAU_LTP) {
                m_weights(event.x(), event.y(), event.z()) += m_conf.ETA_LTP;
            }
            if (static_cast<double>(event.timestamp() - m_lastSpikingTime) < m_conf.TAU_LTD) {
                m_weights(event.x(), event.y(), event.z()) += m_conf.ETA_LTD;
            }
            if (m_weights(event.x(), event.y(), event.z()) < 0) {
                m_weights(event.x(), event.y(), event.z()) = 0;
            }
        }

        normalizeWeights();
        //    m_decay = 1 / (1 + exp(m_totalSpike - m_networkConf.DECAY_FACTOR));
    }
    m_events.clear();
}

inline void ComplexNeuron::normalizeWeights() {
    Eigen::Tensor<double, 0> norm = m_weights.pow(2).sum().sqrt();
   // Eigen::Tensor<double, 0> norm = m_weights.sum();
    if (norm(0) != 0) {
        m_weights = m_conf.NORM_FACTOR * m_weights / norm(0);
    }
}

inline cv::Mat ComplexNeuron::summedWeightMatrix() {
    Eigen::Tensor<double, 2> sum = m_weights.sum(Eigen::array<double, 1>{2});
    const Eigen::Tensor<long, 2>::Dimensions &dim = sum.dimensions();
    Eigen::Tensor<double, 0> max = sum.maximum();
    Eigen::Tensor<double, 2> result = sum * 255. / max(0);

    cv::Mat mat = cv::Mat::zeros(static_cast<int>(dim[1]), static_cast<int>(dim[0]), CV_8UC3);
    for (int i = 0; i < dim[0]; ++i) {
        for (int j = 0; j < dim[1]; ++j) {
            auto &color = mat.at<cv::Vec3b>(j, i);
            color[0] = static_cast<unsigned char>(result(i, j));
        }
    }
    return mat;
}

void ComplexNeuron::saveWeights(std::string &filePath) {
    auto weightFile = filePath + std::to_string(m_index);
    Util::saveComplexTensorToNumpyFile(m_weights, weightFile);
}

void ComplexNeuron::loadWeights(std::string &filePath) {
    auto weightFile = filePath + std::to_string(m_index);
    Util::loadNumpyFileToComplexTensor(m_weights, weightFile);
}

std::vector<long> ComplexNeuron::getWeightsDimension() {
    const Eigen::Tensor<double, COMPLEXDIM>::Dimensions& dimensions = m_weights.dimensions();
    std::vector<long> dim = { dimensions[0], dimensions[1], dimensions[2] };
    return dim;
}

void ComplexNeuron::savePotentials(uint64_t time, int type_, Neuron &neuron, double wi){

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
                m_inhibWeightsStatLateralTopDown[type_].emplace_back(wi,time);
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
                m_inhibWeightsStatLateralTopDown[type_].emplace_back(wi,time);
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
                m_inhibWeightsStatLateralTopDown[type_].emplace_back(wi,time);
            }
            m_potentialThreshold.emplace_back(m_threshold);
            m_counter+=1;
        }
}
