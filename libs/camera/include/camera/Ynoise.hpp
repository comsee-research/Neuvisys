//
// Created by Thomas on 10/12/2021.
//

#ifndef NEUVISYS_YNOISE_HPP
#define NEUVISYS_YNOISE_HPP

//std
#include <cstdint>
#include <vector>

//externals
#include <libcaercpp/events/polarity.hpp>

//neuvisys
#include <network/Event.hpp>

struct matrixS {
    uint32_t timestamp; // keep only the lower part for performance
    bool polarity;
    matrixS() : timestamp{0}, polarity{false} {
    }
};

using densityMatrixType = bool;
using densityMatrixT = std::vector<densityMatrixType>;
using matrixBufferT = std::vector<matrixS>;
using dividedT = std::vector<uint32_t>;

class Ynoise {
public:
    Ynoise(uint32_t sizeX, uint32_t sizeY, uint32_t deltaT = 10000, uint8_t lParam = 3, uint8_t threshold = 2);
    std::vector<Event> run(const std::vector<Event> &inEvent);
    std::vector<Event> run(const libcaer::events::PolarityEventPacket &inEvent);

private:
    densityMatrixT densityMatrix;
    matrixBufferT matrixMem;
    uint32_t deltaT;
    uint8_t lParam;
    uint32_t squareLParam;
    uint8_t threshold;
    uint32_t sizeX;
    uint32_t sizeY;
    dividedT dividedLparam;
    dividedT modLparam;

    void updateMatrix(const Event &event);
    uint8_t calculateDensity(const Event &event);
    // generation of array of i/lParam and i%lParam for performance optimisation
    void regenerateDMLparam();

};


#endif //NEUVISYS_YNOISE_HPP
