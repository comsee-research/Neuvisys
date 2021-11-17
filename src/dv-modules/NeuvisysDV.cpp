#include <dv-sdk/module.hpp>
#include <dv-sdk/processing/core.hpp>
#include <thread>
#include "../network/NetworkHandle.hpp"

#define CONFIG "/home/alphat/neuvisys-dv/configuration/network/configs/network_config.json"

class Neuvisys : public dv::ModuleBase {
private:
    dv::EventStreamSlicer slicer;
    NetworkHandle network = NetworkHandle(config.getString("networkPath"));

public:
    Neuvisys() {
        /***** Slicers *****/
        slicer.doEveryTimeInterval(Conf::EVENT_FREQUENCY, [this](const dv::EventStore &data) {
            computeEvents(data);
        });
    }

    ~Neuvisys() override {
        network.save(0, "realTime");
    }

    static void initInputs(dv::InputDefinitionList &in) {
		in.addEventInput("events1");
        //in.addEventInput("events2");
	}

	static void initOutputs(dv::OutputDefinitionList &out) {
	}

	void computeEvents(const dv::EventStore &events) {
        if (!events.isEmpty()) {
            for (const dv::Event &eve : events) {
//                network.transmitEvents(Event(eve.timestamp(), eve.m_jitterPos(), eve.y(), eve.polarity(), 0));
            }
        }
    }

	void run() override {
        slicer.accept(inputs.getEventInput("events1").events());
        //slicer.accept(inputs.getEventInput("events2").events());
    }

    static const char *initDescription() {
        return ("Neuvisys module.");
    }

    static void initConfigOptions(dv::RuntimeConfig &config) {
        config.add("networkPath", dv::ConfigOption::fileOpenOption("Network Path"));
    }

    void configUpdate() override {
    }
};

registerModuleClass(Neuvisys)
