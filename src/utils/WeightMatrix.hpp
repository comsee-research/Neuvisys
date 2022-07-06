//
// Created by thomas on 06/07/22.
//

#ifndef NEUVISYS_WEIGHTMATRIX_HPP
#define NEUVISYS_WEIGHTMATRIX_HPP

#include <random>
#include "Type.hpp"
#include "cnpy.h"

class WeightMatrix {
    std::unordered_map<size_t, double> m_data;
    std::vector<size_t> m_dimensions;

    std::mt19937 generator;

public:
    WeightMatrix();

    WeightMatrix(std::vector<size_t> dimensions, unsigned long seed, bool uniform=false);

    void normalize(double normFactor);

    void addNewWeight(size_t id);

    std::vector<size_t> getDimensions() const { return m_dimensions; }

    size_t getSize() const { return m_data.size(); }

    double &at(size_t id) { return m_data.at(id); }

    void loadNumpyFile(const std::string &filePath);

    void loadNumpyFile(cnpy::npz_t &arrayNPZ, const std::string &arrayName);

    void saveWeightsToNumpyFile(const std::string &filePath);

    void saveWeightsToNumpyFile(const std::string &filePath, const std::string &arrayName);

private:
    double getNorm();

    static void mapToWeights(const std::unordered_map<size_t, double> &map, std::vector<double> &data);

    static void weightsToMap(std::unordered_map<size_t, double> &map, const double *weights);
};

#endif //NEUVISYS_WEIGHTMATRIX_HPP
