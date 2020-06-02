#include <dv-sdk/module.hpp>
#include <dv-sdk/processing/core.hpp>
#include "xtensor/xnpy.hpp"
#include "xtensor/xtensor.hpp"
#include "xtensor-blas/xlinalg.hpp"
#include <fstream>

class EventAnalysis : public dv::ModuleBase {
private:
public:
    EventAnalysis() {
    }

    static void initInputs(dv::InputDefinitionList &in) {
        //in.addEventInput("events");
    }

    static void initOutputs(dv::OutputDefinitionList &out) {
        //out.addEventOutput("events");
    }

    void run() override {

    }

    static const char *initDescription() {
        return ("Module that analyses different characteristics of a stream of events");
    }

    static void initConfigOptions(dv::RuntimeConfig &config) {

    }
};

registerModuleClass(EventAnalysis)
