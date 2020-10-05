#ifndef NEUVISYS_DV_UTILS_HPP
#define NEUVISYS_DV_UTILS_HPP

#include <opencv2/core/mat.hpp>
#include "xtensor/xtensor.hpp"
#include "xtensor-blas/xlinalg.hpp"

namespace Util {
    [[maybe_unused]] xt::xarray<double> opencvMatToXarray(cv::Mat &mat, size_t row, size_t col);
    xt::xarray<double> uniformMatrixComplex(size_t row, size_t col, size_t layer);
    xt::xarray<double> uniformMatrixSimple(size_t row, size_t col, size_t nbSynapses);
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
