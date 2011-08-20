# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rbtying/robot/cereal_port

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rbtying/robot/cereal_port/build

# Include any dependencies generated for this target.
include CMakeFiles/cereal_port.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cereal_port.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cereal_port.dir/flags.make

CMakeFiles/cereal_port.dir/src/CerealPort.o: CMakeFiles/cereal_port.dir/flags.make
CMakeFiles/cereal_port.dir/src/CerealPort.o: ../src/CerealPort.cpp
CMakeFiles/cereal_port.dir/src/CerealPort.o: ../manifest.xml
CMakeFiles/cereal_port.dir/src/CerealPort.o: /home/rbtying/ros_tutorials/roscpp/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rbtying/robot/cereal_port/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/cereal_port.dir/src/CerealPort.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/cereal_port.dir/src/CerealPort.o -c /home/rbtying/robot/cereal_port/src/CerealPort.cpp

CMakeFiles/cereal_port.dir/src/CerealPort.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cereal_port.dir/src/CerealPort.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/rbtying/robot/cereal_port/src/CerealPort.cpp > CMakeFiles/cereal_port.dir/src/CerealPort.i

CMakeFiles/cereal_port.dir/src/CerealPort.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cereal_port.dir/src/CerealPort.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/rbtying/robot/cereal_port/src/CerealPort.cpp -o CMakeFiles/cereal_port.dir/src/CerealPort.s

CMakeFiles/cereal_port.dir/src/CerealPort.o.requires:
.PHONY : CMakeFiles/cereal_port.dir/src/CerealPort.o.requires

CMakeFiles/cereal_port.dir/src/CerealPort.o.provides: CMakeFiles/cereal_port.dir/src/CerealPort.o.requires
	$(MAKE) -f CMakeFiles/cereal_port.dir/build.make CMakeFiles/cereal_port.dir/src/CerealPort.o.provides.build
.PHONY : CMakeFiles/cereal_port.dir/src/CerealPort.o.provides

CMakeFiles/cereal_port.dir/src/CerealPort.o.provides.build: CMakeFiles/cereal_port.dir/src/CerealPort.o
.PHONY : CMakeFiles/cereal_port.dir/src/CerealPort.o.provides.build

# Object files for target cereal_port
cereal_port_OBJECTS = \
"CMakeFiles/cereal_port.dir/src/CerealPort.o"

# External object files for target cereal_port
cereal_port_EXTERNAL_OBJECTS =

../lib/libcereal_port.so: CMakeFiles/cereal_port.dir/src/CerealPort.o
../lib/libcereal_port.so: CMakeFiles/cereal_port.dir/build.make
../lib/libcereal_port.so: CMakeFiles/cereal_port.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library ../lib/libcereal_port.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cereal_port.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cereal_port.dir/build: ../lib/libcereal_port.so
.PHONY : CMakeFiles/cereal_port.dir/build

CMakeFiles/cereal_port.dir/requires: CMakeFiles/cereal_port.dir/src/CerealPort.o.requires
.PHONY : CMakeFiles/cereal_port.dir/requires

CMakeFiles/cereal_port.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cereal_port.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cereal_port.dir/clean

CMakeFiles/cereal_port.dir/depend:
	cd /home/rbtying/robot/cereal_port/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rbtying/robot/cereal_port /home/rbtying/robot/cereal_port /home/rbtying/robot/cereal_port/build /home/rbtying/robot/cereal_port/build /home/rbtying/robot/cereal_port/build/CMakeFiles/cereal_port.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cereal_port.dir/depend

