//
// Created by Thomas on 10/12/2021.
//

#include <camera/Ynoise.hpp>

Ynoise::Ynoise(uint32_t sizeX, uint32_t sizeY, uint32_t deltaT, uint8_t lParam, uint8_t threshold) :
        deltaT(deltaT), lParam(lParam), threshold(threshold), sizeX(sizeX), sizeY(sizeY) {
    squareLParam = lParam * lParam;
    densityMatrix.resize(squareLParam);
    matrixMem.resize(sizeX * sizeY);
    regenerateDMLparam();

}

std::vector<Event> Ynoise::run(const std::vector<Event> &inEvent) {
    auto outEvent = std::vector<Event>();

    for (auto &evt: inEvent) {
        if (calculateDensity(evt) >= threshold) {
            outEvent.push_back(evt);
        }
        updateMatrix(evt);
    }
    return outEvent;
}

std::vector<Event> Ynoise::run(const libcaer::events::PolarityEventPacket &inEvent) {
    auto outEvent = std::vector<Event>();

    for (auto &evt: inEvent) {
        auto event = Event(evt.getTimestamp(), evt.getX(), evt.getY(), evt.getPolarity());
        if (calculateDensity(event) >= threshold) {
            outEvent.push_back(event);
        }
        updateMatrix(event);
    }
    return outEvent;
}

void Ynoise::updateMatrix(const Event &event) {
    auto address = event.x() * sizeY + event.y();
    matrixMem[address].polarity = event.polarity();
    matrixMem[address].timestamp = static_cast<uint32_t>(event.timestamp());
}

uint8_t Ynoise::calculateDensity(const Event &event) {
    auto sub = ((lParam - 1) / 2) - 1;
    auto addressX = event.x() - sub;
    auto addressY = event.y() - sub;
    auto timeToCompare = static_cast<uint32_t>(event.timestamp() - deltaT);
    auto polarity = event.polarity();
    uint8_t lInfNorm{0}; // event density performed with l infinity norm instead of l1 norm
    uint8_t sum{0};

    if (addressX >= 0 && addressY >= 0) {
        for (uint32_t i = 0; i < squareLParam; i++) {
            uint32_t newAddressY = static_cast<uint32_t>(addressY) + modLparam[i];
            uint32_t newAddressX = static_cast<uint32_t>(addressX) + dividedLparam[i];
            if (newAddressX < sizeX && newAddressY < sizeY) {
                auto &matrixElem = matrixMem[newAddressX * sizeY + newAddressY];
                if (polarity == matrixElem.polarity) {
                    if (timeToCompare < matrixElem.timestamp) {
                        if (modLparam[i] == 0) {
                            lInfNorm = std::max(lInfNorm, sum);
                            sum = 0;
                        }
                        sum++;
                    }
                }
            }
        }
    }

    return lInfNorm;
}

// generation of array of i/lParam and i%lParam for performance optimisation
void Ynoise::regenerateDMLparam() {
    dividedLparam.resize(squareLParam);
    modLparam.resize(squareLParam);
    for (uint32_t i = 0; i < squareLParam; i++) {
        dividedLparam[i] = i / lParam;
        modLparam[i] = i % lParam;
    }
}
