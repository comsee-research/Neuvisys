#include <dv-sdk/module.hpp>
#include <dv-sdk/processing/core.hpp>
#include "cnpy.h"


class EventLoader : public dv::ModuleBase {
private:
public:
    EventLoader() {
        outputs.getEventOutput("events").setup(346, 260, "Events");
        log.info << config.getString("file") << dv::logEnd;
    }

    static void initInputs(dv::InputDefinitionList &in) {

    }

    static void initOutputs(dv::OutputDefinitionList &out) {
        out.addEventOutput("events");
    }

    void run() override {
        auto output = outputs.getEventOutput("events");

        log.info << config.getString("file") << dv::logEnd;
        auto array = cnpy::npy_load(config.getString("file"));
        size_t sizeArray = 4 * array.shape[0];
        auto *events = array.data<double>();

        for (size_t i = 0; i < sizeArray; ++i) {
            output << dv::Event(static_cast<int64_t>(events[i]),static_cast<int16_t>(events[i+1]), static_cast<int16_t>(events[i+2]),static_cast<bool>(events[i+3]));
            output << dv::commit;
        }
    }

    static const char *initDescription() {
        return ("Load npy files and return an Event Stream");
    }

    static void initConfigOptions(dv::RuntimeConfig &config) {
        config.add("file", dv::ConfigOption::fileOpenOption("Numpy event file", "npy"));
    }

    void configUpdate() override {
    }
};

registerModuleClass(EventLoader)
