# Neuvisys Project

The Neuvisys project stands for Neuromorphic Vision System. It is a library offering access to a SpikingNeuralNetwork with different possible kinds of neurons.
The library is written in c++.
It can be launched with command lines, via a Qt gui or via Inivation DV software as a DV module.

## Requirements

### Qt
install QT 5 with the Qt Charts module.

### Neuvisys library:
- OpenCV
- Python

Neuvisys uses libraries such as Eigen, a json parser and cnpy, all linked locally from src/dependencies

### DV software:
install **dv-software**: https://inivation.gitlab.io/dv/dv-docs/docs/getting-started.html

**Documentation**: https://inivation.gitlab.io/dv/dv-docs

## Launch

To compile the Neuvisys library:
- Run ``mkdir build-release`` , ``cd build-release``, ``cmake -DCMAKE_BUILD_TYPE=Release ..``

- Run ``make``


There is 4 executables/libraries:
- ``./qt-gui`` launches the Qt interface.
- ``./neuvisys`` is the command line executable.
- ``./neuvisys-test`` is a test utility.
- ``./libneuvisys-dv.so`` is a shared library used by DV-software.

### To connect the neuvisys library with the dv-software:
Run the dv graphical interface either in your application launcher or by running ``dv-gui``

#### Set up DV

To test the neuvisys module, you will have to set up DV first. See the following documentation: https://inivation.gitlab.io/dv/dv-docs/docs/first-module/

Once setup, you can launch the dv-software with:
- Run ``/usr/bin/dv-runtime -b0``

