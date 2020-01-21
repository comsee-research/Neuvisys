# DV Toolkit C++ Example Module

Example project for a Dynamic Vision Toolkit module written in C++.
Use this as a starting point to develop your own DV Toolkit modules.

### Getting started

**Documentation: [inivation.gitlab.io/dv/dv-docs](https://inivation.gitlab.io/dv/dv-docs)**

**Configure, build and install:**
*(Make sure you have the newest dv-runtime installed)*

```sh
 cmake .
 make
 sudo make install
```

**Open in an IDE**

Most IDEs will support importing a project from cmake directly.
Just import the project's `CMakeLists.txt` file.

**Change name**

1. Edit the line `PROJECT(dv_example_module_cpp C CXX)` in `CMakeLists.txt` to the name of your module.
2. Ensure the proper inputs and outputs are defined with the `addInputs()` and `addOutputs()` static
functions in `ExampleModule.hpp`.
3. Change the return value of the `getDescription` static function in `ExampleModule.hpp`.
4. Rename the class in `ExampleModule.hpp`. Make sure to rename the argument at the `registerModuleClass`
invocation at the bottom of the file too.
5. Change the run() function in `ExampleModule.cpp` to implement your own algorithms.

### More information

Get the full documentation on how to write modules on [inivation.gitlab.io/dv/dv-docs](https://inivation.gitlab.io/dv/dv-docs).
