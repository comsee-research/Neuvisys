# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build

# Include any dependencies generated for this target.
include test/CMakeFiles/neuvisys_tests.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/neuvisys_tests.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/neuvisys_tests.dir/flags.make

test/CMakeFiles/neuvisys_tests.dir/SpikingNetworkTest.cpp.o: test/CMakeFiles/neuvisys_tests.dir/flags.make
test/CMakeFiles/neuvisys_tests.dir/SpikingNetworkTest.cpp.o: ../test/SpikingNetworkTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/neuvisys_tests.dir/SpikingNetworkTest.cpp.o"
	cd /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/neuvisys_tests.dir/SpikingNetworkTest.cpp.o -c /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/test/SpikingNetworkTest.cpp

test/CMakeFiles/neuvisys_tests.dir/SpikingNetworkTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/neuvisys_tests.dir/SpikingNetworkTest.cpp.i"
	cd /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/test/SpikingNetworkTest.cpp > CMakeFiles/neuvisys_tests.dir/SpikingNetworkTest.cpp.i

test/CMakeFiles/neuvisys_tests.dir/SpikingNetworkTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/neuvisys_tests.dir/SpikingNetworkTest.cpp.s"
	cd /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/test/SpikingNetworkTest.cpp -o CMakeFiles/neuvisys_tests.dir/SpikingNetworkTest.cpp.s

test/CMakeFiles/neuvisys_tests.dir/NetworkHandleTest.cpp.o: test/CMakeFiles/neuvisys_tests.dir/flags.make
test/CMakeFiles/neuvisys_tests.dir/NetworkHandleTest.cpp.o: ../test/NetworkHandleTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object test/CMakeFiles/neuvisys_tests.dir/NetworkHandleTest.cpp.o"
	cd /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/neuvisys_tests.dir/NetworkHandleTest.cpp.o -c /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/test/NetworkHandleTest.cpp

test/CMakeFiles/neuvisys_tests.dir/NetworkHandleTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/neuvisys_tests.dir/NetworkHandleTest.cpp.i"
	cd /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/test/NetworkHandleTest.cpp > CMakeFiles/neuvisys_tests.dir/NetworkHandleTest.cpp.i

test/CMakeFiles/neuvisys_tests.dir/NetworkHandleTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/neuvisys_tests.dir/NetworkHandleTest.cpp.s"
	cd /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/test/NetworkHandleTest.cpp -o CMakeFiles/neuvisys_tests.dir/NetworkHandleTest.cpp.s

# Object files for target neuvisys_tests
neuvisys_tests_OBJECTS = \
"CMakeFiles/neuvisys_tests.dir/SpikingNetworkTest.cpp.o" \
"CMakeFiles/neuvisys_tests.dir/NetworkHandleTest.cpp.o"

# External object files for target neuvisys_tests
neuvisys_tests_EXTERNAL_OBJECTS =

test/neuvisys_tests: test/CMakeFiles/neuvisys_tests.dir/SpikingNetworkTest.cpp.o
test/neuvisys_tests: test/CMakeFiles/neuvisys_tests.dir/NetworkHandleTest.cpp.o
test/neuvisys_tests: test/CMakeFiles/neuvisys_tests.dir/build.make
test/neuvisys_tests: libs/network/libnetwork.a
test/neuvisys_tests: libs/motor_control/gmock/lib/libgtest.a
test/neuvisys_tests: libs/motor_control/gmock/lib/libgtest_main.a
test/neuvisys_tests: ../externals/cnpy/libcnpy.so
test/neuvisys_tests: /usr/local/lib/libopencv_gapi.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_stitching.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_aruco.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_barcode.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_bgsegm.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_bioinspired.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_ccalib.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_dnn_objdetect.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_dnn_superres.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_dpm.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_face.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_freetype.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_fuzzy.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_hfs.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_img_hash.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_intensity_transform.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_line_descriptor.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_mcc.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_quality.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_rapid.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_reg.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_rgbd.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_saliency.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_stereo.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_structured_light.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_phase_unwrapping.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_superres.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_optflow.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_surface_matching.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_tracking.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_highgui.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_datasets.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_plot.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_text.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_videostab.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_videoio.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_wechat_qrcode.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_xfeatures2d.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_ml.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_shape.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_ximgproc.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_video.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_xobjdetect.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_imgcodecs.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_objdetect.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_calib3d.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_dnn.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_features2d.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_flann.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_xphoto.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_photo.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_imgproc.so.4.5.5
test/neuvisys_tests: /usr/local/lib/libopencv_core.so.4.5.5
test/neuvisys_tests: /usr/lib/x86_64-linux-gnu/libpython3.8.so
test/neuvisys_tests: /usr/lib/x86_64-linux-gnu/hdf5/serial/libhdf5_cpp.so
test/neuvisys_tests: /usr/lib/x86_64-linux-gnu/hdf5/serial/libhdf5.so
test/neuvisys_tests: /usr/lib/x86_64-linux-gnu/libpthread.so
test/neuvisys_tests: /usr/lib/x86_64-linux-gnu/libsz.so
test/neuvisys_tests: /usr/lib/x86_64-linux-gnu/libz.so
test/neuvisys_tests: /usr/lib/x86_64-linux-gnu/libdl.so
test/neuvisys_tests: /usr/lib/x86_64-linux-gnu/libm.so
test/neuvisys_tests: test/CMakeFiles/neuvisys_tests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable neuvisys_tests"
	cd /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/neuvisys_tests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/neuvisys_tests.dir/build: test/neuvisys_tests

.PHONY : test/CMakeFiles/neuvisys_tests.dir/build

test/CMakeFiles/neuvisys_tests.dir/clean:
	cd /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build/test && $(CMAKE_COMMAND) -P CMakeFiles/neuvisys_tests.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/neuvisys_tests.dir/clean

test/CMakeFiles/neuvisys_tests.dir/depend:
	cd /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/test /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build/test /home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/build/test/CMakeFiles/neuvisys_tests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/neuvisys_tests.dir/depend

