#include <dv-sdk/module.hpp>
#include <dv-sdk/processing/core.hpp>
#include <thread>

#include <src/robot-control/motor-control/BrushlessMotor.hpp>
#include <src/network/NetworkHandle.hpp>

#define CONFIG "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json"

class Neuvisys : public dv::ModuleBase {
private:
    dv::EventStreamSlicer slicer;
    NetworkHandle network = NetworkHandle(CONFIG);
    std::chrono::high_resolution_clock::time_point time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point updateTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point actionTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point consoleTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point motorTime = std::chrono::high_resolution_clock::now();
    int action = 0;
    size_t iteration = 0;
    double position = 0;
    std::vector<std::pair<uint64_t, float>> motorMapping;
    BrushlessMotor m_motor = BrushlessMotor("leftmotor1", 0, "/dev/ttyUSB0");

public:
    Neuvisys() {
        log.info << "Opening network : " + config.getString("networkPath") << dv::logEnd;

        motorMapping.emplace_back(std::make_pair(0, 250)); // left horizontal -> left movement
        motorMapping.emplace_back(std::make_pair(0, 0)); // no movement
        motorMapping.emplace_back(std::make_pair(0, -250)); // left horizontal  -> right movement

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
            network.transmitReward(80 * (55000 - abs(3000 - position)) / 55000);
            network.saveValueMetrics(static_cast<double>(events.back().timestamp()), events.size());
            for (const dv::Event &eve : events) {
                network.transmitEvent(Event(eve.timestamp(), eve.x(), eve.y(), eve.polarity(), 0));
            }

            if (std::chrono::duration<double>(time - updateTime).count() > static_cast<double>(UPDATE_INTERVAL) / E6) {
                updateTime = std::chrono::high_resolution_clock::now();
                network.updateNeuronStates(UPDATE_INTERVAL);
            }

            if (std::chrono::duration<double>(time - actionTime).count() > static_cast<double>(network.getNetworkConfig().getActionRate()) / E6) {
                actionTime = std::chrono::high_resolution_clock::now();
                if (action != -1) {
                    network.updateActor(events.back().timestamp(), action);
                }
                auto choice = network.actionSelection(network.resolveMotor(), network.getNetworkConfig().getExplorationFactor());
                action = choice.first;
                activateMotor(action);
                network.saveActionMetrics(action, choice.second);
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

            if (std::chrono::duration<double>(time - motorTime).count() > 1) {
                motorTime = std::chrono::high_resolution_clock::now();
                position = m_motor.getPosition();
                log.info << "Position : " << position << dv::logEnd;
                if (position < -55000) {
                    m_motor.setSpeed(250);
                } else if (position > 55000) {
                    m_motor.setSpeed(-250);
                }
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
        config.add("networkPath", dv::ConfigOption::directoryOption("Network Path"));
    }

    void configUpdate() override {
    }

private:
    void activateMotor(uint64_t motor) {
        switch (motorMapping[motor].first) {
            case 0:
                m_motor.setSpeed(motorMapping[motor].second);
                break;
        }
    }
};

registerModuleClass(Neuvisys)
