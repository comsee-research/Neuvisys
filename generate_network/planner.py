#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 12 10:11:17 2020

@author: thomas
"""

import json
import subprocess
import os
from network_config import NetworkConfig


def generate_network(directory):
    conf = NetworkConfig()
    list_params = generate_list_params(conf.params)
    create_directories(directory, list_params)


def generate_list_params(params):
    network_params = params["network_params"]
    neuron_params = params["neuron_params"]
    pooling_neuron_params = params["pooling_neuron_params"]
    motor_neuron_params = params["motor_neuron_params"]

    return (
        network_params,
        neuron_params,
        pooling_neuron_params,
        motor_neuron_params,
    )


def create_directories(directory, list_params):
	n_layer = 4

    os.mkdir(directory + "network")
    os.mkdir(directory + "network/configs")
    os.mkdir(directory + "network/figures")
    os.mkdir(directory + "network/figures/complex_directions")
    os.mkdir(directory + "network/figures/complex_figures")
    os.mkdir(directory + "network/figures/complex_orientations")
    os.mkdir(directory + "network/figures/complex_weights_orientations")
    os.mkdir(directory + "network/figures/simple_figures")
    os.mkdir(directory + "network/gabors")
    os.mkdir(directory + "network/gabors/data")
    os.mkdir(directory + "network/gabors/figures")
    os.mkdir(directory + "network/gabors/hists")

    os.mkdir(directory + "network/images")
	for i in range(n_layer):
		os.mkdir(directory + "network/images/"+str(i))
    os.mkdir(directory + "network/weights")
	for i in range(n_layer):
		os.mkdir(directory + "network/weights/"+str(i))

    with open(
        directory + "network/configs/network_config.json", "w"
    ) as file:
        json.dump(list_params[0], file)

    with open(
        directory + "network/configs/simple_cell_config.json", "w"
    ) as file:
        json.dump(list_params[1], file)

    with open(
        directory + "network/configs/complex_cell_config.json", "w"
    ) as file:
        json.dump(list_params[2], file)

    with open(
        directory + "network/configs/motor_cell_config.json", "w"
    ) as file:
        json.dump(list_params[3], file)


def toggle_learning(spinet, switch):
    with open(spinet.path + "configs/complex_cell_config.json", "r") as file:
        conf = json.load(file)
    conf["STDP_LEARNING"] = switch
    with open(spinet.path + "configs/complex_cell_config.json", "w") as file:
        json.dump(conf, file)

    with open(spinet.path + "configs/simple_cell_config.json", "r") as file:
        conf = json.load(file)
    conf["STDP_LEARNING"] = switch
    with open(spinet.path + "configs/simple_cell_config.json", "w") as file:
        json.dump(conf, file)


def execute(cmd):
    popen = subprocess.Popen(cmd, stdout=subprocess.PIPE, universal_newlines=True)
    for stdout_line in iter(popen.stdout.readline, ""):
        yield stdout_line
    popen.stdout.close()
    return_code = popen.wait()
    if return_code:
        raise subprocess.CalledProcessError(return_code, cmd)


def launch_neuvisys_multi_pass(exec_path, network_path, event_file, nb_pass):
    for path in execute([exec_path, network_path, event_file, str(nb_pass),]):
        print(path, end="")
