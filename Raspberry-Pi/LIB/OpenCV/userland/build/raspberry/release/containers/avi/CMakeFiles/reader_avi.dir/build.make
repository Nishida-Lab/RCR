# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/pi/RCR/LIB/OpenCV/userland

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release

# Include any dependencies generated for this target.
include containers/avi/CMakeFiles/reader_avi.dir/depend.make

# Include the progress variables for this target.
include containers/avi/CMakeFiles/reader_avi.dir/progress.make

# Include the compile flags for this target's objects.
include containers/avi/CMakeFiles/reader_avi.dir/flags.make

containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.o: containers/avi/CMakeFiles/reader_avi.dir/flags.make
containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.o: ../../../containers/avi/avi_reader.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.o"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/avi && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/reader_avi.dir/avi_reader.c.o   -c /home/pi/RCR/LIB/OpenCV/userland/containers/avi/avi_reader.c

containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/reader_avi.dir/avi_reader.c.i"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/avi && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/pi/RCR/LIB/OpenCV/userland/containers/avi/avi_reader.c > CMakeFiles/reader_avi.dir/avi_reader.c.i

containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/reader_avi.dir/avi_reader.c.s"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/avi && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/pi/RCR/LIB/OpenCV/userland/containers/avi/avi_reader.c -o CMakeFiles/reader_avi.dir/avi_reader.c.s

containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.o.requires:
.PHONY : containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.o.requires

containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.o.provides: containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.o.requires
	$(MAKE) -f containers/avi/CMakeFiles/reader_avi.dir/build.make containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.o.provides.build
.PHONY : containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.o.provides

containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.o.provides.build: containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.o

# Object files for target reader_avi
reader_avi_OBJECTS = \
"CMakeFiles/reader_avi.dir/avi_reader.c.o"

# External object files for target reader_avi
reader_avi_EXTERNAL_OBJECTS =

../../lib/reader_avi.so: containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.o
../../lib/reader_avi.so: containers/avi/CMakeFiles/reader_avi.dir/build.make
../../lib/reader_avi.so: ../../lib/libcontainers.so
../../lib/reader_avi.so: ../../lib/libvcos.so
../../lib/reader_avi.so: containers/avi/CMakeFiles/reader_avi.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C shared library ../../../../lib/reader_avi.so"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/avi && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/reader_avi.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
containers/avi/CMakeFiles/reader_avi.dir/build: ../../lib/reader_avi.so
.PHONY : containers/avi/CMakeFiles/reader_avi.dir/build

containers/avi/CMakeFiles/reader_avi.dir/requires: containers/avi/CMakeFiles/reader_avi.dir/avi_reader.c.o.requires
.PHONY : containers/avi/CMakeFiles/reader_avi.dir/requires

containers/avi/CMakeFiles/reader_avi.dir/clean:
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/avi && $(CMAKE_COMMAND) -P CMakeFiles/reader_avi.dir/cmake_clean.cmake
.PHONY : containers/avi/CMakeFiles/reader_avi.dir/clean

containers/avi/CMakeFiles/reader_avi.dir/depend:
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/RCR/LIB/OpenCV/userland /home/pi/RCR/LIB/OpenCV/userland/containers/avi /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/avi /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/avi/CMakeFiles/reader_avi.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : containers/avi/CMakeFiles/reader_avi.dir/depend
