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
        /***** Initialize Network *****/
        if (SAVE_DATA) {
            spinet.loadWeights();
        }
        lastTime = 0;

        /***** Displays *****/
        outputs.getEventOutput("frames").setup(WIDTH, HEIGHT, "frames");
        displays.emplace_back(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3));

        outputs.getFrameOutput("potentials").setup(WIDTH, HEIGHT, "potentials");
        displays.emplace_back(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1));

        outputs.getFrameOutput("spikes").setup(WIDTH, HEIGHT, "spikes");
        displays.emplace_back(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1));

        outputs.getFrameOutput("weights").setup(NEURON_WIDTH, NEURON_HEIGHT, "weights");
        displays.emplace_back(cv::Mat::zeros(NEURON_HEIGHT, NEURON_WIDTH, CV_8UC3));

        outputs.getFrameOutput("zoom").setup(NEURON_WIDTH, NEURON_HEIGHT, "zoom");
        displays.emplace_back(cv::Mat::zeros(NEURON_HEIGHT, NEURON_WIDTH, CV_8UC3));

        /***** Slicers *****/
        slicer.doEveryTimeInterval(EVENT_FREQUENCY, [this](const dv::EventStore &data) {
            computeEvents(data);
        });
        slicer.doEveryTimeInterval(DISPLAY_FREQUENCY, [this](const dv::EventStore &data) {
            computeDisplays(data);
        });
        slicer.doEveryTimeInterval(UPDATE_PARAMETER_FREQUENCY, [this](const dv::EventStore &data) {
            computeParameters();
        });
    }

    ~Neuvisys() override {
        if (SAVE_DATA) {
            spinet.saveWeights();
        }
    }

    static void initInputs(dv::InputDefinitionList &in) {
		in.addEventInput("events");
	}

	static void initOutputs(dv::OutputDefinitionList &out) {
        out.addFrameOutput("frames");
        out.addFrameOutput("potentials");
        out.addFrameOutput("spikes");
        out.addFrameOutput("weights");
        out.addFrameOutput("zoom");
	}

    void computeDisplays(const dv::EventStore &events) {
        spinet.updateDisplay(lastTime, displays);

        auto frame = outputs.getFrameOutput("frames").frame();
        frame << displays[0];
        frame.commit();
        displays[0] = 0;

        outputs.getFrameOutput("potentials").frame() << displays[1];
        outputs.getFrameOutput("potentials").frame().commit();

        outputs.getFrameOutput("spikes").frame() << displays[2];
        outputs.getFrameOutput("spikes").frame().commit();

        auto weights = outputs.getFrameOutput("weights").frame();
        weights.setFormat(dv::FrameFormat::BGR);
        weights << displays[3];
        weights.commit();

        auto zoom = outputs.getFrameOutput("zoom").frame();
        weights.setFormat(dv::FrameFormat::BGR);
        zoom << displays[4];
        zoom.commit();
    }

	void computeEvents(const dv::EventStore &events) {
        if (!events.isEmpty()) {
            lastTime = events.getHighestTime();
            for (const dv::Event &event : events) {
                spinet.addEvent(event.timestamp(), event.x(), event.y(), event.polarity());

                displays[0].at<cv::Vec3b>(event.y(), event.x())[2-event.polarity()] = 255;
            }
        }
        spinet.updateNeurons(lastTime);
    }

    void computeParameters() {
        spinet.updateNeuronsParameters();

        config.setLong("spiking_rate", static_cast<long>(1000 * spinet.getNeuron(IND).getSpikingRate()));
        config.setLong("threshold", static_cast<long>(spinet.getNeuron(IND).getThreshold()));
    }

	void run() override {
        slicer.accept(inputs.getEventInput("events").events());
    }

    static const char *initDescription() {
        return ("Neuvisys module.");
    }

    static void initConfigOptions(dv::RuntimeConfig &config) {
        std::string confFile = CONF_FILE;
        Config::loadConfiguration(confFile);

        std::string configurationFile = CONF_FILES_LOCATION;
        Config::loadNetworkLayout(configurationFile);
        Config::loadNeuronsParameters(configurationFile);

        X_NEURON = 0;
        Y_NEURON = 0;
        LAYER = 0;
        IND = X_NEURON * NETWORK_HEIGHT * NETWORK_DEPTH + Y_NEURON * NETWORK_DEPTH + LAYER;

        config.add("A_LOAD_BUTTON", dv::ConfigOption::buttonOption("Load config file", "Load Config"));
        config.add("A_LOAD_CONFIG", dv::ConfigOption::fileOpenOption(("Config file load location"), configurationFile, "json"));

        config.add("X_NEURON", dv::ConfigOption::intOption("X Position of the neuron to display", X_NEURON, 0, NETWORK_WIDTH-1));
        config.add("Y_NEURON", dv::ConfigOption::intOption("Y Position of the neuron to display", Y_NEURON, 0, NETWORK_HEIGHT-1));
        config.add("SYNAPSE", dv::ConfigOption::intOption("Layer of the neuron to display", LAYER, 0, NEURON_SYNAPSES-1));
        config.add("LAYER", dv::ConfigOption::intOption("Layer of the neuron to display", LAYER, 0, NETWORK_DEPTH-1));

        config.add("spiking_rate", dv::ConfigOption::statisticOption("Spiking rate"));
        config.add("threshold", dv::ConfigOption::statisticOption("Threshold"));

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
            SYNAPSE = config.getInt("SYNAPSE");
            LAYER = config.getInt("LAYER");
            IND = X_NEURON * NETWORK_HEIGHT * NETWORK_DEPTH + Y_NEURON * NETWORK_DEPTH + LAYER;
        }
    }

    static void setParameters(dv::RuntimeConfig &config) {
        config.setBool("A_LOAD_BUTTON", false);
    }
};

registerModuleClass(Neuvisys)
