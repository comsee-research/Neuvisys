//
// Created by thomas on 06/07/22.
//

#include "WeightMatrix.hpp"

#include <utility>

std::random_device WeightMatrix::m_rd;
std::mt19937 WeightMatrix::m_generator(WeightMatrix::m_rd());
std::uniform_real_distribution<double> WeightMatrix::m_uniformRealDistr(0.0, 1.0);

WeightMatrix::WeightMatrix() : m_data() {
    m_dimensions.push_back(0);
}

WeightMatrix::WeightMatrix(std::vector<size_t> dimensions) : m_data(), m_dimensions(std::move(dimensions)) {

}

WeightMatrix::WeightMatrix(std::vector<size_t> dimensions, bool uniform, double normFactor) : m_dimensions(std::move(dimensions)) {
    size_t nbWeights = 1;
    for (const auto d : m_dimensions) {
        nbWeights *= d;
    }

    for (size_t i = 0; i < nbWeights; ++i) {
        if (uniform) {
            m_data[i] = m_uniformRealDistr(m_generator);
        } else {
            m_data[i] = 0;
        }
    }
    normalize(normFactor);
}

void WeightMatrix::initWeights(const std::vector<size_t> &indices, bool uniform, double normFactor) {
    for (const auto &id : indices) {
        if (uniform) {
            m_data[id] = m_uniformRealDistr(m_generator);
        } else {
            m_data[id] = 0;
        }
    }
    normalize(normFactor);
}

double WeightMatrix::getNorm() {
    double norm = 0;
    for (const auto &element : m_data) {
        norm += pow(element.second, 2);
    }
    return sqrt(norm);
}

void WeightMatrix::normalize(const double normFactor) {
    auto norm = getNorm();
    if (norm != 0) {
        for (auto &element : m_data) {
            element.second *= normFactor / norm;
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
    cnpy::npy_save(filePath + ".npy", &data[0], m_dimensions, "w");
}

void WeightMatrix::saveWeightsToNumpyFile(const std::string &filePath, const std::string &arrayName) {
    std::vector<double> data(static_cast<size_t>(m_data.size()));
    mapToWeights(m_data, data);
    cnpy::npz_save(filePath, arrayName, &data[0], m_dimensions, "a");
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

double &WeightMatrix::at(size_t id) { // direct indexing
    return m_data.at(id);
}

double &WeightMatrix::get(size_t a) { // dimensional indexing
    return m_data.at(a);
}

double &WeightMatrix::get(size_t a, size_t b) {
    return m_data.at(b + a * m_dimensions[1]);
}

double &WeightMatrix::get(size_t a, size_t b, size_t c) {
    return m_data.at(c + b * m_dimensions[2] + a * m_dimensions[2] * m_dimensions[1]);
}

double &WeightMatrix::get(size_t a, size_t b, size_t c, size_t d) {
    return m_data.at(d + c * m_dimensions[3] + b * m_dimensions[3] * m_dimensions[2] + a * m_dimensions[3] * m_dimensions[2] * m_dimensions[1]);
}

double &WeightMatrix::get(size_t a, size_t b, size_t c, size_t d, size_t e) {
    return m_data.at(e + d * m_dimensions[4] + c * m_dimensions[4] * m_dimensions[3] + b * m_dimensions[4] * m_dimensions[3] * m_dimensions[2] + a * m_dimensions[4] *
    m_dimensions[3] * m_dimensions[2] * m_dimensions[1]);
}

void WeightMatrix::setSeed(size_t seed) {
    m_generator.seed(seed);
}
