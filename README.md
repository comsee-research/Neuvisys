# Neuvisys Project

The Neuvisys project stands for Neuromorphic Vision System. It is a library offering access to a Spiking Neural Network (SNN) with different possible kinds of neurons.
The library is written in c++.
It can be launched with command lines, via a Qt gui or via Inivation DV software as a DV module. There is also a possible connection with the Coppeliasim simulator, also known as V-REP, via a ROS interface.

## Requirements

### Neuvisys

- OpenCV
- Python

Neuvisys uses libraries such as Eigen, a json parser and cnpy, all linked locally from src/dependencies

### Qt
install QT 5 with the **Qt Charts** module.

You may need to install this:

``sudo apt install libqt5charts5-dev``

### DV software:
install **dv-software**: https://inivation.gitlab.io/dv/dv-docs/docs/getting-started.html

**Documentation**: https://inivation.gitlab.io/dv/dv-docs

### To connect the neuvisys library with the dv-software:
Run the dv graphical interface either in your application launcher or by running ``dv-gui``

#### Set up DV

To test the neuvisys module, you will have to set up DV first. See the following documentation: https://inivation.gitlab.io/dv/dv-docs/docs/first-module/

Once setup, you can launch the dv-software with:
- Run ``/usr/bin/dv-runtime -b0``

### Coppeliasim / ROS

Download and install the Coppeliasim framework: https://www.coppeliarobotics.com/
Install ROS Noetic (Other ROS distribution might work, but this is uncertain): http://wiki.ros.org/noetic/Installation/Ubuntu

### Create empty Network

In the folder "generate_network", there is a python script that you can use to create and initialize a network structure:

- Run ``cd  generate_network``, ``python -c 'import planner; planner.generate_network("../")'``

## Launch

To compile the Neuvisys library:
- Run ``mkdir build``, ``cd build``, ``cmake -DCMAKE_BUILD_TYPE=Release ..``

If there is some errors, you may have to install the following python packages:
``pip install empy``
``pip install catkin-pkg``

- Run ``make`` to compile all targets

or

- Run ``make [target-name]`` to compile only one target. possible targets are:
- ``neuvisys-exe`` is the command line executable.
- ``neuvisys-qt`` is similar to neuvisys but with an added Qt interface.
- ``neuvisys-dv.so`` is a module that can be used with dv-software.
- ``neuvisys-ros`` is an executable that connects to Coppeliasim via ROS.

compiled target are found in the "build/src" folder.

An example of use with the ``neuvisys-exe`` target:

- ``./neuvisys-exe [networkPath] [eventPath] [nbPass]``

``networkPath`` correspond to the path of the network structure. If you used the python script for creating an empty network, that would be:
``./configuration/network/configs/network_config.json``.

``eventPath`` is the relative path to an event file in the .npz format.

``nbPass`` is the number of times the events will be presented to the network.
