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
include containers/mpeg/CMakeFiles/reader_ps.dir/depend.make

# Include the progress variables for this target.
include containers/mpeg/CMakeFiles/reader_ps.dir/progress.make

# Include the compile flags for this target's objects.
include containers/mpeg/CMakeFiles/reader_ps.dir/flags.make

containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.o: containers/mpeg/CMakeFiles/reader_ps.dir/flags.make
containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.o: ../../../containers/mpeg/ps_reader.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.o"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/mpeg && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/reader_ps.dir/ps_reader.c.o   -c /home/pi/RCR/LIB/OpenCV/userland/containers/mpeg/ps_reader.c

containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/reader_ps.dir/ps_reader.c.i"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/mpeg && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/pi/RCR/LIB/OpenCV/userland/containers/mpeg/ps_reader.c > CMakeFiles/reader_ps.dir/ps_reader.c.i

containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/reader_ps.dir/ps_reader.c.s"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/mpeg && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/pi/RCR/LIB/OpenCV/userland/containers/mpeg/ps_reader.c -o CMakeFiles/reader_ps.dir/ps_reader.c.s

containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.o.requires:
.PHONY : containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.o.requires

containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.o.provides: containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.o.requires
	$(MAKE) -f containers/mpeg/CMakeFiles/reader_ps.dir/build.make containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.o.provides.build
.PHONY : containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.o.provides

containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.o.provides.build: containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.o

# Object files for target reader_ps
reader_ps_OBJECTS = \
"CMakeFiles/reader_ps.dir/ps_reader.c.o"

# External object files for target reader_ps
reader_ps_EXTERNAL_OBJECTS =

../../lib/reader_ps.so: containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.o
../../lib/reader_ps.so: containers/mpeg/CMakeFiles/reader_ps.dir/build.make
../../lib/reader_ps.so: ../../lib/libcontainers.so
../../lib/reader_ps.so: ../../lib/libvcos.so
../../lib/reader_ps.so: containers/mpeg/CMakeFiles/reader_ps.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C shared library ../../../../lib/reader_ps.so"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/mpeg && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/reader_ps.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
containers/mpeg/CMakeFiles/reader_ps.dir/build: ../../lib/reader_ps.so
.PHONY : containers/mpeg/CMakeFiles/reader_ps.dir/build

containers/mpeg/CMakeFiles/reader_ps.dir/requires: containers/mpeg/CMakeFiles/reader_ps.dir/ps_reader.c.o.requires
.PHONY : containers/mpeg/CMakeFiles/reader_ps.dir/requires

containers/mpeg/CMakeFiles/reader_ps.dir/clean:
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/mpeg && $(CMAKE_COMMAND) -P CMakeFiles/reader_ps.dir/cmake_clean.cmake
.PHONY : containers/mpeg/CMakeFiles/reader_ps.dir/clean

containers/mpeg/CMakeFiles/reader_ps.dir/depend:
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/RCR/LIB/OpenCV/userland /home/pi/RCR/LIB/OpenCV/userland/containers/mpeg /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/mpeg /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/containers/mpeg/CMakeFiles/reader_ps.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : containers/mpeg/CMakeFiles/reader_ps.dir/depend

