# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.30.2/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.30.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/build

# Include any dependencies generated for this target.
include optitrack/CMakeFiles/simviz_stage.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include optitrack/CMakeFiles/simviz_stage.dir/compiler_depend.make

# Include the progress variables for this target.
include optitrack/CMakeFiles/simviz_stage.dir/progress.make

# Include the compile flags for this target's objects.
include optitrack/CMakeFiles/simviz_stage.dir/flags.make

optitrack/CMakeFiles/simviz_stage.dir/simviz.cpp.o: optitrack/CMakeFiles/simviz_stage.dir/flags.make
optitrack/CMakeFiles/simviz_stage.dir/simviz.cpp.o: /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/optitrack/simviz.cpp
optitrack/CMakeFiles/simviz_stage.dir/simviz.cpp.o: optitrack/CMakeFiles/simviz_stage.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object optitrack/CMakeFiles/simviz_stage.dir/simviz.cpp.o"
	cd /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/build/optitrack && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT optitrack/CMakeFiles/simviz_stage.dir/simviz.cpp.o -MF CMakeFiles/simviz_stage.dir/simviz.cpp.o.d -o CMakeFiles/simviz_stage.dir/simviz.cpp.o -c /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/optitrack/simviz.cpp

optitrack/CMakeFiles/simviz_stage.dir/simviz.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/simviz_stage.dir/simviz.cpp.i"
	cd /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/build/optitrack && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/optitrack/simviz.cpp > CMakeFiles/simviz_stage.dir/simviz.cpp.i

optitrack/CMakeFiles/simviz_stage.dir/simviz.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/simviz_stage.dir/simviz.cpp.s"
	cd /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/build/optitrack && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/optitrack/simviz.cpp -o CMakeFiles/simviz_stage.dir/simviz.cpp.s

# Object files for target simviz_stage
simviz_stage_OBJECTS = \
"CMakeFiles/simviz_stage.dir/simviz.cpp.o"

# External object files for target simviz_stage
simviz_stage_EXTERNAL_OBJECTS =

/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: optitrack/CMakeFiles/simviz_stage.dir/simviz.cpp.o
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: optitrack/CMakeFiles/simviz_stage.dir/build.make
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /Users/rheamalhotra/Desktop/robotics/OpenSai/core/sai2-interfaces/build/libsai2-interfaces.a
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /Users/rheamalhotra/Desktop/robotics/OpenSai/core/sai2-model/build/libsai2-model.a
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /Users/rheamalhotra/Desktop/robotics/OpenSai/core/sai2-model/rbdl/build/librbdl.dylib
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /Users/rheamalhotra/Desktop/robotics/OpenSai/core/sai2-graphics/build/libsai2-graphics.a
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /opt/homebrew/lib/libglfw.dylib
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /Users/rheamalhotra/Desktop/robotics/OpenSai/core/sai2-simulation/build/libsai2-simulation.a
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /Users/rheamalhotra/Desktop/robotics/OpenSai/core/sai2-simulation/lib/macOS/arm64/libsai2-simulation-core.a
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /Users/rheamalhotra/Desktop/robotics/OpenSai/core/sai2-common/build/libsai2-common.a
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /opt/homebrew/lib/libjsoncpp.dylib
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /opt/homebrew/lib/libhiredis.dylib
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /Users/rheamalhotra/Desktop/robotics/OpenSai/core/sai2-urdfreader/build/libsai2-urdf.a
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /opt/homebrew/lib/libtinyxml2.dylib
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /Users/rheamalhotra/Desktop/robotics/OpenSai/core/chai3d/build/libchai3d.a
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /opt/homebrew/lib/libhiredis.dylib
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /Users/rheamalhotra/Desktop/robotics/OpenSai/core/sai2-primitives/build/libsai2-primitives.a
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /Users/rheamalhotra/Desktop/robotics/OpenSai/core/sai2-primitives/ruckig/build/libruckig.dylib
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: libcenter-demo.a
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /opt/homebrew/lib/libtinyxml2.dylib
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: /Users/rheamalhotra/Desktop/robotics/OpenSai/core/sai2-primitives/ruckig/build/libruckig.dylib
/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage: optitrack/CMakeFiles/simviz_stage.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage"
	cd /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/build/optitrack && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simviz_stage.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
optitrack/CMakeFiles/simviz_stage.dir/build: /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/bin/optitrack/simviz_stage
.PHONY : optitrack/CMakeFiles/simviz_stage.dir/build

optitrack/CMakeFiles/simviz_stage.dir/clean:
	cd /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/build/optitrack && $(CMAKE_COMMAND) -P CMakeFiles/simviz_stage.dir/cmake_clean.cmake
.PHONY : optitrack/CMakeFiles/simviz_stage.dir/clean

optitrack/CMakeFiles/simviz_stage.dir/depend:
	cd /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/optitrack /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/build /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/build/optitrack /Users/rheamalhotra/Desktop/robotics/optitrack-robot-dance/build/optitrack/CMakeFiles/simviz_stage.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : optitrack/CMakeFiles/simviz_stage.dir/depend

