//
// Created by thomas on 06/07/22.
//

#ifndef NEUVISYS_WEIGHTMATRIX_HPP
#define NEUVISYS_WEIGHTMATRIX_HPP

#include <random>
#include "Type.hpp"
#include "cnpy.h"

class WeightMatrix {
    std::vector<double> m_data;
    std::vector<size_t> m_dimensions;

    static std::random_device m_rd;
    static std::mt19937 m_generator;

public:
    WeightMatrix();

    WeightMatrix(std::vector<size_t> dimensions, bool uniform, double normFactor);

    static void setSeed(size_t seed);

    void normalize(double normFactor);

    void addNewWeight();

    std::vector<size_t> getDimensions() const { return m_dimensions; }

    size_t getSize() const { return m_data.size(); }

    double &at(size_t id);

    double &get(size_t a);

    double &get(size_t a, size_t b);

    double &get(size_t a, size_t b, size_t c);

    double &get(size_t a, size_t b, size_t c, size_t d);

    double &get(size_t a, size_t b, size_t c, size_t d, size_t e);

    void loadNumpyFile(const std::string &filePath);

    void loadNumpyFile(cnpy::npz_t &arrayNPZ, const std::string &arrayName);

    void saveWeightsToNumpyFile(const std::string &filePath);

    void saveWeightsToNumpyFile(const std::string &filePath, const std::string &arrayName);

private:
    double getNorm();

    static void mapToWeights(const std::vector<double> &map, std::vector<double> &data);

    static void weightsToMap(std::vector<double> &map, const double *weights);

};

#endif //NEUVISYS_WEIGHTMATRIX_HPP
