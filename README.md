# Neuvisys Project

The Neuvisys project stands for Neuromorphic Vision System. It is a library offering access to a SpikingNeuralNetwork with different possible kinds of neurons.
The library is written in c++.
It offers a direct link with Inivation DV software so that it can be used with one of their DVS cameras.

## Requirements

### Neuvisys library:
install **xtensor** -> https://xtensor.readthedocs.io/en/latest/installation.html
install **xtensor-blas** -> https://github.com/xtensor-stack/xtensor-blas
(You may need to install BLAS and LAPACK libraries with ``sudo apt install libblas-dev liblapack-dev``)
### DV software:
install **dv-software**: https://inivation.gitlab.io/dv/dv-docs/docs/getting-started.html

**Documentation**: https://inivation.gitlab.io/dv/dv-docs

## Launch

To compile the Neuvisys library:
- Run ``cmake .`` in the directory folder
or
- Run ``mkdir cmake-build-release`` , ``cd cmake-build-release``, ``cmake ..``

- Run ``make``

You can test if the software is working properly by running ``./neuvisys-test``
### To connect the neuvisys library with the dv-software:
Run the dv graphical interface either in your application launcher or by running ``dv-gui``
#### Set up DV

To test the neuvisys module, you will have to set up DV first. See the following documentation: https://inivation.gitlab.io/dv/dv-docs/docs/first-module/

Once setup, you can launch the dv-software with:
- Run ``/usr/bin/dv-runtime -b0``

