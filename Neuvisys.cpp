#include <dv-sdk/module.hpp>
#include <dv-sdk/processing/core.hpp>
#include <thread>
#include "src/SpikingNetwork.hpp"

class Neuvisys : public dv::ModuleBase {
private:
    dv::EventStreamSlicer slicer;
	SpikingNetwork spinet;
    std::vector<cv::Mat> displays;
public:
    Neuvisys() {
        for (int i = 0; i < ADJACENT_NEURONS; ++i) {
            outputs.getFrameOutput(std::to_string(i)).setup(inputs.getEventInput("events"));
            displays.emplace_back(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1));
        }

        slicer.doEveryTimeInterval(1000, [this](const dv::EventStore &data) {
            doEvery1ms(data);
        });
    }

	static void addInputs(dv::InputDefinitionList &in) {
		in.addEventInput("events");
	}

	static void addOutputs(dv::OutputDefinitionList &out) {
        for (int i = 0; i < ADJACENT_NEURONS; ++i) {
            out.addFrameOutput(std::to_string(i));
        }
	}

	static const char *getDescription() {
		return ("This is an example module that counts positive events and logs the number to DV logging.");
	}

	static void getConfigOptions(dv::RuntimeConfig &config) {
		config.add("printInterval", dv::ConfigOption::intOption("Interval in number of events between consecutive printing of the event number.", 10000));
	}

	void doEvery1ms(const dv::EventStore &events) {
        if (!events.isEmpty()) {
//            for (unsigned int i = 0; i < NUMBER_THREADS; ++i) {
//                std::thread(&Neuvisys::parallel_events, this, std::ref(events), i * events.getTotalLength() / NUMBER_THREADS, events.getTotalLength() / NUMBER_THREADS).join();
//            }
            for (const dv::Event &event : events) {
                spinet.addEvent(event.timestamp(), event.x(), event.y(), event.polarity());
            }
            spinet.updateNeuronsInformation(events.getHighestTime(), displays);
        }

        for (size_t i = 0; i < ADJACENT_NEURONS; ++i) {
            outputs.getFrameOutput(std::to_string(i)) << displays[i];
        }
    }

    void parallel_events(const dv::EventStore &events, unsigned int start, unsigned int length) {
        for (const dv::Event &event : events.slice(start, length)) {
            spinet.addEvent(event.timestamp(), event.x(), event.y(), event.polarity());
        }
    }

	void run() override {
        slicer.accept(inputs.getEventInput("events").events());
    }
};

registerModuleClass(Neuvisys)
