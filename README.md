# Neuvisys Project

The Neuvisys project stands for Neuromorphic Vision System. It is a library offering access to a Spiking Neural Network (SNN) with different possible kinds of neurons.
The library is written in c++.
It can be launched with command lines or via a Qt gui. There is also a possible connection with the Coppeliasim simulator, also known as V-REP, via a ROS interface.

## Requirements

### Neuvisys

- OpenCV
- Python

Neuvisys uses libraries such as Eigen, a json parser and cnpy, all linked locally from src/dependencies

### Event Based Cameras

To connect to event based cameras, and use the SNN live, you need to install caer: https://github.com/breznak/caer

### Qt
install QT 5 with the **Qt Charts** module:

``sudo apt install qt5-default``
``sudo apt install libqt5charts5-dev``

### Coppeliasim / ROS

Download and install the Coppeliasim framework: https://www.coppeliarobotics.com/

There is 3 available versions, **player**, **edu** and **pro**. Player is enough to work with neuvisys, but you will not be able to save the scene if you change it.

Install ROS Noetic (Other ROS distribution might work, but this is uncertain): http://wiki.ros.org/noetic/Installation/Ubuntu

It is advised to use the **Desktop-Full Install**, though other lighter version may also work.

## Neuvisys libraries

By default, only the neuvisys library core is compiled.
There is four more libraries that adds functionnality:

- Camera: allows the connection to event based camera thanks to caer. With it activated, you can then use event based cameras and feed the events directly to the SNN in real time.
- Simulator: allows the connection to Coppeliasim. With it activated, you can create complex environments and simulate event based cameras outputs, and feed it to the SNN in real time.
- Motor control: allows the connection with Faulhaber Brushless motors. With it activated, you can pilot the motors in real time.
- GUI: allows the use of a graphical user interface.

## Launch

To compile the Neuvisys library, in the root folder:
- Run ``mkdir build``, ``cd build``, ``cmake -DCMAKE_BUILD_TYPE=Release ..``

If you want to use some of the abovementioned functionnalities, you can compile them with:
- ``cmake -DBUILD_CAMERA=ON -DBUILD_SIMULATOR=ON -DBUILD_MOTOR_CONTROL=ON -DBUILD_GUI=ON -DCMAKE_BUILD_TYPE=Release ..``
(put ``OFF`` on the functionnalities you do not want to use and compile).

The core neuvisys library does not need any installation requirements except OPENCV. But adding more functionnalities means installing the adequate libraries (see Requirements section).

If there is some errors, you may have to install the following python packages:
``pip install empy``
``pip install catkin-pkg``

- Run ``make -j`` to compile all targets

or

- Run ``make [target-name]`` to compile only one target. possible targets are:
- ``neuvisys-exe`` is the command line executable.
- ``event-camera`` is a module to connect an event based camera (davis by default).
- ``neuvisys-simulator`` is an executable that connects to Coppeliasim via ROS.
- ``neuvisys-qt`` is similar to neuvisys but with an added Qt interface.

Compiled target are found in the "build/src" folder.

An example of use with the ``neuvisys-exe`` target:

- ``./neuvisys-exe [networkPath] [eventPath] [nbPass]``

``networkPath`` correspond to the path of the network structure. This must link to the network config file, such as: ``./network/configs/network_config.json``.

``eventPath`` is the relative path to an event file in the .npz format.

``nbPass`` is the number of times the events will be presented to the network.

### Create empty Network

You can generate an empty spiking network ready to use from:

- Run ``cd  build``, ``./neuvisys-exe [networkDirectory]``

The parameters will be set to their default values, but you can change them afterwards using the gui or directly via the json config files. 
