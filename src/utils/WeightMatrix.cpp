//
// Created by thomas on 06/07/22.
//

#include "WeightMatrix.hpp"

WeightMatrix::WeightMatrix() : m_data() {
    m_dimensions.push_back(0);
}

WeightMatrix::WeightMatrix(std::vector<size_t> dimensions, unsigned long seed, bool uniform) : m_dimensions(std::move(dimensions)) {
    generator = std::mt19937(seed);
    std::uniform_real_distribution<double> uniformRealDistr(0.0, 1.0);

    for (const auto d : m_dimensions) {
        for (size_t i = 0; i < d; ++i) {
            if (uniform) {
                m_data[i] = uniformRealDistr(generator);
            } else {
                m_data[i] = 0;
            }
        }
    }
}

void WeightMatrix::addNewWeight(size_t id) {
    m_data[id] = 0;
}

double WeightMatrix::getNorm() {
    double norm = 0;
    for (const auto d : m_dimensions) {
        for (size_t i = 0; i < d; ++i) {
            norm += m_data.at(i);
        }
    }
    return norm;
}

void WeightMatrix::normalize(const double normFactor) {
    auto norm = getNorm();
    for (const auto d : m_dimensions) {
        for (size_t i = 0; i < d; ++i) {
            m_data.at(i) *= normFactor / norm;
        }
    }
}

void WeightMatrix::loadNumpyFile(const std::string &filePath) {
    auto *weights = cnpy::npy_load(filePath).data<double>();
    weightsToMap(m_data, weights);
}

void WeightMatrix::loadNumpyFile(cnpy::npz_t &arrayNPZ, const std::string &arrayName) {
    auto *weights = arrayNPZ[arrayName].data<double>();
    weightsToMap(m_data, weights);
}

void WeightMatrix::saveWeightsToNumpyFile(const std::string &filePath) {
    std::vector<double> data(static_cast<size_t>(m_data.size()));
    mapToWeights(m_data, data);
    cnpy::npy_save(filePath + ".npy", &data[0], {m_data.size()}, "w");
}

void WeightMatrix::saveWeightsToNumpyFile(const std::string &filePath, const std::string &arrayName) {
    std::vector<double> data(static_cast<size_t>(m_data.size()));
    mapToWeights(m_data, data);
    cnpy::npz_save(filePath, arrayName, &data[0], {m_data.size()}, "a");
}

void WeightMatrix::mapToWeights(const std::unordered_map<size_t, double> &map, std::vector<double> &data) {
    size_t count = 0;
    for (auto const &element: map) {
        data[count] = element.second;
        ++count;
    }
}

void WeightMatrix::weightsToMap(std::unordered_map<size_t, double> &map, const double *weights) {
    size_t count = 0;
    for (auto &element: map) {
        element.second = weights[count];
        ++count;
    }
}
