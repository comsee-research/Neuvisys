#include <dv-sdk/module.hpp>
#include <dv-sdk/processing/core.hpp>

class EventAnalysis : public dv::ModuleBase {
private:
    dv::EventStreamSlicer slicer;
    long time;
public:
    EventAnalysis() {
        outputs.getEventOutput("events").setup(inputs.getEventInput("events"));
        dv::EventStore store = inputs.getEventInput("events").events();
        time = store.getHighestTime() - store.getLowestTime();

        slicer.doEveryTimeInterval(1000000, [this](const dv::EventStore &data) {
            process(data);
        });
    }

    static void initInputs(dv::InputDefinitionList &in) {
        in.addEventInput("events");
    }

    static void initOutputs(dv::OutputDefinitionList &out) {
        out.addEventOutput("events");
    }

    void process(const dv::EventStore &events) {
        dv::EventStore store;
        for (auto event : events) {
            for (int loop = 0; loop < 20; ++loop) {
                store += dv::Event(event.timestamp() + loop * time, event.x(), event.y(), event.polarity());
            }
        }
        outputs.getEventOutput("events").events() << store << dv::commit;
    }

    void run() override {
        slicer.accept(inputs.getEventInput("events").events());
    }

    static const char *initDescription() {
        return ("Module that analyses different characteristics of a stream of events");
    }

    static void initConfigOptions(dv::RuntimeConfig &config) {

    }
};

registerModuleClass(EventAnalysis)
