#ifndef NEUVISYS_DV_UTILS_HPP
#define NEUVISYS_DV_UTILS_HPP

#include "dependencies/unsupported/Eigen/CXX11/Tensor"
#include <random>
#include <chrono>
#include "cnpy.h"

namespace Util {
    Eigen::Tensor<double, 3> uniformMatrixComplex(long x, long y, long z);
    Eigen::Tensor<double, 4> uniformMatrixSimple(long s, long x, long y);
    void loadNumpyFileTo3DTensor(std::string &filePath, Eigen::Tensor<double, 3> &tensor);
    void save3DTensorToNumpyFile(Eigen::Tensor<double, 3> tensor, std::string &saveFile);
    void loadNumpyFileTo4DTensor(std::string &filePath, Eigen::Tensor<double, 4> &tensor);
    void save4DTensorToNumpyFile(Eigen::Tensor<double, 4> tensor, std::string &saveFile);
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
    size_t m_posx;
    size_t m_posy;
    size_t m_posz;
public:
    Position() = default;
    inline Position(size_t x, size_t y, size_t z) : m_posx(x), m_posy(y), m_posz(z) {}
    inline Position(size_t x, size_t y) : m_posx(x), m_posy(y), m_posz(0) {}
    [[nodiscard]] inline size_t posx() const {return m_posx;}
    [[nodiscard]] inline size_t posy() const {return m_posy;}
    [[nodiscard]] inline size_t posz() const {return m_posz;}
};

#endif //NEUVISYS_DV_UTILS_HPP
