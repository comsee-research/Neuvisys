#ifndef NEUVISYS_DV_UTILS_HPP
#define NEUVISYS_DV_UTILS_HPP

#include <opencv2/core/mat.hpp>
#include "xtensor/xtensor.hpp"
#include "xtensor-blas/xlinalg.hpp"

namespace Util {
    [[maybe_unused]] xt::xarray<double> opencvMatToXarray(cv::Mat &mat, int row, int col);
    xt::xarray<double> uniformMatrixPooling(int row, int col, int layer);
    xt::xarray<double> uniformMatrixSynapses(int row, int col, int nbSynapses);
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
    int m_posx;
    int m_posy;
    int m_posz;
public:
    Position() = default;
    inline Position(int x, int y, int z) : m_posx(x), m_posy(y), m_posz(z) {}
    inline Position(int x, int y) : m_posx(x), m_posy(y), m_posz(0) {}
    [[nodiscard]] inline int posx() const {return m_posx;}
    [[nodiscard]] inline int posy() const {return m_posy;}
    [[nodiscard]] inline int posz() const {return m_posz;}
};

#endif //NEUVISYS_DV_UTILS_HPP
