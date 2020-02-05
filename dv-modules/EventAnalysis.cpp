#include <dv-sdk/module.hpp>
#include <dv-sdk/processing/core.hpp>

class EventAnalysis : public dv::ModuleBase {
private:
    dv::EventStreamSlicer slicer;
    unsigned long cumEvents;
    unsigned long cumOn;
    unsigned long cumOff;
public:
    EventAnalysis() {
        cumEvents = 0;
        cumOn = 0;
        cumOff = 0;
        slicer.doEveryTimeInterval(1000000, [this](const dv::EventStore &data) {
            analyseEvents(data);
        });
    }

    static void addInputs(dv::InputDefinitionList &in) {
        in.addEventInput("events");
    }


    void analyseEvents(const dv::EventStore &events) {
        eventFrequency(events);
    }

    void eventFrequency(const dv::EventStore &events) {
        unsigned long on = 0, off = 0;
        for (const dv::Event &event : events) {
            if (event.polarity()) {
                ++on;
            } else {
                ++off;
            }
        }
        cumEvents += events.getTotalLength();
        cumOn += on;
        cumOff += off;
        std::cout << "On frequency: " << 100. * static_cast<double>(on) / static_cast<double>(events.getTotalLength()) << "%" << std::endl;
        std::cout << "Off frequency: " << 100. * static_cast<double>(off) / static_cast<double>(events.getTotalLength()) << "%" << std::endl;

        std::cout << "Cumulative Frequency: " << 100. * static_cast<double>(cumOn) / static_cast<double>(cumEvents) << "% / " << 100. * static_cast<double>(cumOff) / static_cast<double>(cumEvents) << "%" << std::endl;
    }

    void run() override {
        slicer.accept(inputs.getEventInput("events").events());
    }

    static const char *getDescription() {
        return ("Module that analyses different characteristics of a stream of events");
    }

    static void getConfigOptions(dv::RuntimeConfig &config) {

    }
};

registerModuleClass(EventAnalysis)
