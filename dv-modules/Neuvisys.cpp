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
        // Initialize Network
        spinet.loadWeights();
        lastTime = 0;

        // Displays
        outputs.getFrameOutput("weights").setup(NEURON_HEIGHT, NEURON_WIDTH, "weights");
        displays.emplace_back(cv::Mat::zeros(NEURON_HEIGHT, NEURON_WIDTH, CV_8UC3));

        for (size_t i = 1; i < NUMBER_DISPLAY+1; ++i) {
            outputs.getFrameOutput(std::to_string(i)).setup(inputs.getEventInput("events"));
            displays.emplace_back(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1));
        }

        // Slicers
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

    ~Neuvisys() override {
        spinet.saveWeights();
    }

    static void initInputs(dv::InputDefinitionList &in) {
		in.addEventInput("events");
	}

	static void initOutputs(dv::OutputDefinitionList &out) {
        out.addFrameOutput("weights");
        for (size_t i = 1; i < NUMBER_DISPLAY+1; ++i) {
            out.addFrameOutput(std::to_string(i));
        }
	}

	void computeEvents(const dv::EventStore &events) {
        if (!events.isEmpty()) {
            lastTime = events.getHighestTime();
            for (const dv::Event &event : events) {
                spinet.addEvent(event.timestamp(), event.x(), event.y(), event.polarity());
            }
//            for (unsigned int i = 0; i < NUMBER_THREADS; ++i) {
//                std::thread(&Neuvisys::parallel_events, this, std::ref(events), i * events.getTotalLength() / NUMBER_THREADS, events.getTotalLength() / NUMBER_THREADS).join();
//            }
        }
        //spinet.updateNeurons(lastTime);
    }

    void computeDisplays() {
        spinet.updateDisplay(lastTime, displays);

        auto frame = outputs.getFrameOutput("weights").frame();
        frame.setFormat(dv::FrameFormat::BGR);
        frame << displays[0];
        frame.commit();
        for (size_t i = 1; i < NUMBER_DISPLAY+1; ++i) {
            outputs.getFrameOutput(std::to_string(i)).frame() << displays[i];
            outputs.getFrameOutput(std::to_string(i)).frame().commit();
        }
    }

    void displayInformations() {
        spinet.neuronsInfos();
    }

//    void parallel_events(const dv::EventStore &events, unsigned int start, unsigned int length) {
//        for (const dv::Event &event : events.slice(start, length)) {
//            spinet.addEvent(event.timestamp(), event.x(), event.y(), event.polarity());
//        }
//    }

	void run() override {
        slicer.accept(inputs.getEventInput("events").events());
    }

    static const char *initDescription() {
        return ("Neuvisys module.");
    }

    static void initConfigOptions(dv::RuntimeConfig &config) {
        std::string confFile = CONF_FILE;
        Config::loadConfiguration(confFile);

        std::string networkFile = CONF_FILES_LOCATION + "config_network_" + FILE_NUMBER + ".json";
        Config::loadNetworkLayout(networkFile);

        std::string neuronFile = CONF_FILES_LOCATION + "config_neuron_" + FILE_NUMBER + ".json";
        Config::loadNeuronsParameters(neuronFile);

        X_NEURON = 0;
        Y_NEURON = 0;
        LAYER = 0;
        IND = X_NEURON * NETWORK_HEIGHT * NETWORK_DEPTH + Y_NEURON * NETWORK_DEPTH + LAYER;

        config.add("A_LOAD_BUTTON", dv::ConfigOption::buttonOption("Load config file", "Load Config"));
        config.add("A_LOAD_CONFIG", dv::ConfigOption::fileOpenOption(("Config file load location"), neuronFile, "json"));

        config.add("X_NEURON", dv::ConfigOption::intOption("X Position of the neuron to display", X_NEURON, 0, NETWORK_WIDTH-1));
        config.add("Y_NEURON", dv::ConfigOption::intOption("Y Position of the neuron to display", Y_NEURON, 0, NETWORK_HEIGHT-1));
        config.add("LAYER", dv::ConfigOption::intOption("Layer of the neuron to display", LAYER, 0, NETWORK_DEPTH-1));

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

        setParameters(config);
    }

    void configUpdate() override {
        if (config.getBool("A_LOAD_BUTTON")) {
            std::string fileName = config.getString("A_LOAD_CONFIG");
            Config::loadNeuronsParameters(fileName);
            setParameters(config);
        } else {
            X_NEURON = config.getInt("X_NEURON");
            Y_NEURON = config.getInt("Y_NEURON");
            LAYER = config.getInt("LAYER");
            IND = X_NEURON * NETWORK_HEIGHT * NETWORK_DEPTH + Y_NEURON * NETWORK_DEPTH + LAYER;

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
    }

    static void setParameters(dv::RuntimeConfig &config) {
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
    }
};

registerModuleClass(Neuvisys)
