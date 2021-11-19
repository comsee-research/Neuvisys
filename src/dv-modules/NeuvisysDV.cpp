#include <dv-sdk/module.hpp>
#include <dv-sdk/processing/core.hpp>
#include <thread>
#include "../network/NetworkHandle.hpp"

#define CONFIG "/home/alphat/neuvisys-dv/configuration/network/configs/network_config.json"

class Neuvisys : public dv::ModuleBase {
private:
    dv::EventStreamSlicer slicer;
    NetworkHandle network = NetworkHandle(config.getString("networkPath"));
    std::chrono::high_resolution_clock::time_point time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point updateTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point actionTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point consoleTime = std::chrono::high_resolution_clock::now();
    size_t actor, iteration;

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
        time = std::chrono::high_resolution_clock::now();
        if (!events.isEmpty()) {
            network.transmitReward(0);
            for (const dv::Event &eve : events) {
                network.transmitEvent(Event(eve.timestamp(), eve.x(), eve.y(), eve.polarity(), 0));
            }

            if (std::chrono::duration<double>(time - updateTime).count() > static_cast<double>(UPDATE_INTERVAL) / E6) {
                updateTime = std::chrono::high_resolution_clock::now();
                network.updateNeuronStates(UPDATE_INTERVAL);
            }

            if (std::chrono::duration<double>(time - actionTime).count() > static_cast<double>(network.getNetworkConfig().getActionRate()) / E6) {
                actionTime = std::chrono::high_resolution_clock::now();
                if (actor != -1) {
                    network.updateActor(events.back().timestamp(), actor);
                }
                sim.motorAction(network.resolveMotor(), network.getNetworkConfig().getExplorationFactor(), actor);
            }

            if (std::chrono::duration<double>(time - consoleTime).count() > SCORE_INTERVAL) {
                consoleTime = std::chrono::high_resolution_clock::now();
                network.learningDecay(iteration);
                ++iteration;
                std::string msg = "Average reward: " + std::to_string(network.getScore(SCORE_INTERVAL * E3 / DT)) +
                        "\nExploration factor: " + std::to_string(network.getNetworkConfig().getExplorationFactor()) +
                        "\nAction rate: " + std::to_string(network.getNetworkConfig().getActionRate());
                std::cout << msg << std::endl;
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
