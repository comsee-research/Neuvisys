#include <dv-sdk/module.hpp>
#include <dv-sdk/processing/core.hpp>
#include <thread>
#include "src/SpikingNetwork.hpp"

class Neuvisys : public dv::ModuleBase {
private:
    dv::EventStreamSlicer slicer;
	SpikingNetwork spinet;
    std::vector<cv::Mat> displays;
    long lastTime;
public:
    Neuvisys() {
        lastTime = 0;
/*        for (size_t i = 0; i < NUMBER_DISPLAY; ++i) {
            outputs.getFrameOutput(std::to_string(i)).setup(inputs.getEventInput("events"));
            displays.emplace_back(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1));
        }*/
        outputs.getFrameOutput("0").setup(inputs.getEventInput("events"));
        outputs.getFrameOutput("1").setup(NEURON_HEIGHT, NEURON_WIDTH, "weights");
        displays.emplace_back(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1));
        displays.emplace_back(cv::Mat::zeros(NEURON_HEIGHT, NEURON_WIDTH, CV_8UC3));

        slicer.doEveryTimeInterval(EVENT_FREQUENCY, [this](const dv::EventStore &data) {
            computeEvents(data);
        });
        slicer.doEveryTimeInterval(DISPLAY_FREQUENCY, [this](const dv::EventStore &data) {
            computeDisplays();
        });
        slicer.doEveryTimeInterval(1000000, [this](const dv::EventStore &data) {
            informations();
        });
    }

	static void addInputs(dv::InputDefinitionList &in) {
		in.addEventInput("events");
	}

	static void addOutputs(dv::OutputDefinitionList &out) {
        for (size_t i = 0; i < NUMBER_DISPLAY; ++i) {
            out.addFrameOutput(std::to_string(i));
        }
	}

	static const char *getDescription() {
		return ("Neuvisys module.");
	}

	static void getConfigOptions(dv::RuntimeConfig &config) {
		config.add("DELTA_VP", dv::ConfigOption::doubleOption("Synapse potentiation value", DELTA_VP));
	}

    void configUpdate() override {
        //DELTA_VP = config.getDouble("DELTA_VP");
    }

	void computeEvents(const dv::EventStore &events) {
        if (!events.isEmpty()) {
            lastTime = events.getHighestTime();
//            for (unsigned int i = 0; i < NUMBER_THREADS; ++i) {
//                std::thread(&Neuvisys::parallel_events, this, std::ref(events), i * events.getTotalLength() / NUMBER_THREADS, events.getTotalLength() / NUMBER_THREADS).join();
//            }
            for (const dv::Event &event : events) {
                spinet.addEvent(event.timestamp(), event.x(), event.y(), event.polarity());
            }
        }
        //spinet.updateNeurons(lastTime);
    }

    void computeDisplays() {
        spinet.updateDisplay(lastTime, displays);
        outputs.getFrameOutput("0") << displays[0];
        auto frame = outputs.getFrameOutput("1").frame();
        frame.setFormat(dv::FrameFormat::BGR);
        frame.commitMat(displays[1]);
/*        for (size_t i = 0; i < NUMBER_DISPLAY; ++i) {
            outputs.getFrameOutput(std::to_string(i)) << displays[i];
        }*/
    }

    void informations() {
        spinet.displayInformations();
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
