#include <dv-sdk/module.hpp>
#include <dv-sdk/processing/core.hpp>

class EventAnalysis : public dv::ModuleBase {
private:
    dv::EventStreamSlicer slicer;
    dv::EventStore store;
    dv::EventStore newStore;
public:
    EventAnalysis() {
        outputs.getEventOutput("events").setup(inputs.getEventInput("events"));

        store = inputs.getEventInput("events").events();
//        long time = store.getHighestTime() - store.getLowestTime();
//        for (auto event: store) {
//            for (int loop = 0; loop < 20; ++loop) {
//                newStore += dv::Event(event.timestamp() + loop * time, event.x(), event.y(), event.polarity());
//            }
//        }

        outputs.getEventOutput("events") << store << dv::commit;
    }

    static void initInputs(dv::InputDefinitionList &in) {
        in.addEventInput("events");
    }

    static void initOutputs(dv::OutputDefinitionList &out) {
        out.addEventOutput("events");
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
