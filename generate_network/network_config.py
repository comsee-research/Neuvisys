# -*- coding: utf-8 -*-

class NetworkConfig:
    def __init__(self):
        self.params = {
            "network_params": {
                "NbCameras": 1,
                "L1Width": 4,
                "L1Height": 4,
                "L1Depth": 100,
                "L1XAnchor": [10, 148, 286],
                "L1YAnchor": [10, 105, 200],
                "Neuron1Width": 10,
                "Neuron1Height": 10,
                "Neuron1Synapses": 1,
                "L2Width": 1,
                "L2Height": 1,
                "L2Depth": 16,
                "L2XAnchor": [0, 4, 8],
                "L2YAnchor": [0, 4, 8],
                "Neuron2Width": 4,
                "Neuron2Height": 4,
                "Neuron2Depth": 100,
                "L3Size": 0,
                "SharingType": "patch",
                "SaveData": True,
            },
            "neuron_params": {
                "VTHRESH": 30,
                "VRESET": -20,
                "TRACKING": "partial",
                "TAU_SRA": 100000,
                "TAU_RP": 20000,
                "TAU_M": 18000,
                "TAU_LTP": 7000,
                "TAU_LTD": 14000,
                "TARGET_SPIKE_RATE": 0.75,
                "SYNAPSE_DELAY": 0,
                "STDP_LEARNING": True,
                "NORM_FACTOR": 4,
                "MIN_THRESH": 4,
                "ETA_LTP": 0.0077,
                "ETA_LTD": -0.0021,
                "ETA_SRA": 0.6,
                "ETA_TA": 1,
                "ETA_RP": 1,
                "ETA_INH": 20,
                "DECAY_FACTOR": 0,
            },
            "pooling_neuron_params": {
                "VTHRESH": 3,
                "VRESET": -20,
                "TRACKING": "partial",
                "TAU_M": 20000,
                "TAU_LTP": 20000,
                "TAU_LTD": 20000,
                "STDP_LEARNING": True,
                "NORM_FACTOR": 10,
                "ETA_LTP": 0.2,
                "ETA_LTD": 0.2,
                "ETA_INH": 25,
                "ETA_RP": 1,
                "TAU_RP": 20000,
                "DECAY_FACTOR": 0,
            },
            "motor_neuron_params": {
                "VTHRESH": 2,
                "VRESET": -20,
                "TRACKING": "partial",
                "TAU_M": 20000,
                "TAU_E": 100000,
                "ETA_INH": 25,
                "TAU_LTP": 7000,
                "TAU_LTD": 14000,
                "ETA_LTP": 0.077,
                "ETA_LTD": -0.021,
                "NORM_FACTOR": 10,
                "STDP_LEARNING": True,
                "DELTA_INH": 10,
            },
        }
