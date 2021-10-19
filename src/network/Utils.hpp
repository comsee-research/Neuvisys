#ifndef NEUVISYS_DV_UTILS_HPP
#define NEUVISYS_DV_UTILS_HPP

#include "../dependencies/unsupported/Eigen/CXX11/Tensor"
#include <random>
#include <chrono>
#include "cnpy.h"

#define SIMPLEDIM 5
#define COMPLEXDIM 3
#define NBPOLARITY 2

namespace Util {
    Eigen::Tensor<double, COMPLEXDIM> uniformMatrixComplex(long x, long y, long z);
    Eigen::Tensor<double, SIMPLEDIM> uniformMatrixSimple(long p, long c, long s, long x, long y);
    void loadNumpyFileToSimpleTensor(std::string &filePath, Eigen::Tensor<double, SIMPLEDIM> &tensor);
    void saveSimpleTensorToNumpyFile(Eigen::Tensor<double, SIMPLEDIM> tensor, std::string &saveFile);
    void loadNumpyFileToComplexTensor(std::string &filePath, Eigen::Tensor<double, COMPLEXDIM> &tensor);
    void saveComplexTensorToNumpyFile(Eigen::Tensor<double, COMPLEXDIM> tensor, std::string &saveFile);
    int winnerTakeAll(std::vector<size_t> v);
    double secondOrderNumericalDifferentiationMean(std::vector<double>::iterator first, std::vector<double>::iterator last);
    bool fileExist(std::string &path);
}

class Luts {
public:
    Luts(double tauM, double tauRP, double tauSRA);
    std::vector<double> const lutM;
    std::vector<double> const lutRP;
    std::vector<double> const lutSRA;
private:
    static std::vector<double> expLUT(double tau);
};

class Position {
    uint64_t m_posx{};
    uint64_t m_posy{};
    uint64_t m_posz{};
public:
    inline Position() : m_posx(0), m_posy(0), m_posz(0) {};
    inline Position(uint64_t x, uint64_t y, uint64_t z) : m_posx(x), m_posy(y), m_posz(z) {}
    inline Position(uint64_t x, uint64_t y) : m_posx(x), m_posy(y), m_posz(0) {}
    [[nodiscard]] inline uint64_t x() const { return m_posx; }
    [[nodiscard]] inline uint64_t y() const { return m_posy; }
    [[nodiscard]] inline uint64_t z() const { return m_posz; }
};

#endif //NEUVISYS_DV_UTILS_HPP
