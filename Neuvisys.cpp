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
            displayInformations();
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

    void displayInformations() {
        spinet.neuronsInfos();
    }

    void parallel_events(const dv::EventStore &events, unsigned int start, unsigned int length) {
        for (const dv::Event &event : events.slice(start, length)) {
            spinet.addEvent(event.timestamp(), event.x(), event.y(), event.polarity());
        }
    }

	void run() override {
        slicer.accept(inputs.getEventInput("events").events());
    }

    static const char *getDescription() {
        return ("Neuvisys module.");
    }

    static void getConfigOptions(dv::RuntimeConfig &config) {
        config.add("A_LOAD_BUTTON", dv::ConfigOption::buttonOption("Load config file", "Load Config"));
        config.add("A_LOAD_CONFIG", dv::ConfigOption::fileOpenOption(("Config file load location"), "/home/thomas/neuvisys-dv/configs/config.json", "json"));
        config.add("A_SAVE_BUTTON", dv::ConfigOption::buttonOption("Save config file", "Save Config"));
        config.add("A_SAVE_CONFIG", dv::ConfigOption::fileSaveOption(("Config file save location"), "/home/thomas/neuvisys-dv/configs/config.json", "json"));

        config.add("NEURON_WIDTH", dv::ConfigOption::intOption("Width of the neurons' receptive field (pixels)", NEURON_WIDTH));
        config.add("NEURON_HEIGHT", dv::ConfigOption::intOption("Height of the neurons' receptive field (pixels)", NEURON_HEIGHT));
        config.add("TAU_M", dv::ConfigOption::doubleOption("Potential decay time constant (μs)", TAU_M));
        config.add("TAU_LTP", dv::ConfigOption::doubleOption("Potentiation learning time constant (μs)", TAU_LTP));
        config.add("TAU_LTD", dv::ConfigOption::doubleOption("Deprecation learning time constant (μs)", TAU_LTD));
        config.add("SPEED", dv::ConfigOption::intOption("Neurons speed sensitivity (μs)", SPEED));
        config.add("VRESET", dv::ConfigOption::doubleOption("Neurons potential reset value after a spike (mV)", VRESET));
        config.add("THRESHOLD", dv::ConfigOption::doubleOption("Neurons potential threshold at which a spike is triggerred (mV)", THRESHOLD));
        config.add("DELTA_VP", dv::ConfigOption::doubleOption("Potentiation learning value (mV)", DELTA_VP));
        config.add("DELTA_VD", dv::ConfigOption::doubleOption("Deprecation learning value (mV)", DELTA_VD));
        config.add("NORM_FACTOR", dv::ConfigOption::doubleOption("Normalization factor", NORM_FACTOR));
        config.add("NORM_THRESHOLD", dv::ConfigOption::intOption("Number of spikes needed for normalization to occur", NORM_THRESHOLD));
    }

    void configUpdate() override {
        if (config.getBool("A_LOAD_BUTTON")) {
            std::string conf = config.getString("A_LOAD_CONFIG");
            Config::loadConfig(conf);

            config.setInt("NEURON_WIDTH", NEURON_WIDTH);
            config.setInt("NEURON_HEIGHT", NEURON_HEIGHT);
            config.setDouble("TAU_M", TAU_M);
            config.setDouble("TAU_LTP", TAU_LTP);
            config.setDouble("TAU_LTD", TAU_LTD);
            config.setInt("SPEED", SPEED);
            config.setDouble("VRESET", VRESET);
            config.setDouble("THRESHOLD", THRESHOLD);
            config.setDouble("DELTA_VP", DELTA_VP);
            config.setDouble("DELTA_VD", DELTA_VD);
            config.setDouble("NORM_FACTOR", NORM_FACTOR);
            config.setInt("NORM_THRESHOLD", NORM_THRESHOLD);
            config.setBool("A_LOAD_BUTTON", false);
        } else if (config.getBool("A_SAVE_BUTTON")) {
            std::string conf = config.getString("A_SAVE_CONFIG");
            Config::loadConfig(conf);
            config.setBool("A_SAVE_BUTTON", false);
        } else {
            NEURON_WIDTH = config.getInt("NEURON_WIDTH");
            NEURON_HEIGHT = config.getInt("NEURON_HEIGHT");
            TAU_M = config.getDouble("TAU_M");
            TAU_LTP = config.getDouble("TAU_LTP");
            TAU_LTD = config.getDouble("TAU_LTD");
            SPEED = config.getInt("SPEED");
            VRESET = config.getDouble("VRESET");
            THRESHOLD = config.getDouble("THRESHOLD");
            DELTA_VP = config.getDouble("DELTA_VP");
            DELTA_VD = config.getDouble("DELTA_VD");
            NORM_FACTOR = config.getDouble("NORM_FACTOR");
            NORM_THRESHOLD = config.getInt("NORM_THRESHOLD");
        }
        std::cout << DELTA_VP << std::endl;
    }
};

registerModuleClass(Neuvisys)
