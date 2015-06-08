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
include interface/mmal/core/CMakeFiles/mmal_core.dir/depend.make

# Include the progress variables for this target.
include interface/mmal/core/CMakeFiles/mmal_core.dir/progress.make

# Include the compile flags for this target's objects.
include interface/mmal/core/CMakeFiles/mmal_core.dir/flags.make

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.o: interface/mmal/core/CMakeFiles/mmal_core.dir/flags.make
interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.o: ../../../interface/mmal/core/mmal_format.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.o"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mmal_core.dir/mmal_format.c.o   -c /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_format.c

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_core.dir/mmal_format.c.i"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_format.c > CMakeFiles/mmal_core.dir/mmal_format.c.i

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_core.dir/mmal_format.c.s"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_format.c -o CMakeFiles/mmal_core.dir/mmal_format.c.s

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.o.requires:
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.o.requires

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.o.provides: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.o.requires
	$(MAKE) -f interface/mmal/core/CMakeFiles/mmal_core.dir/build.make interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.o.provides.build
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.o.provides

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.o.provides.build: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.o

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.o: interface/mmal/core/CMakeFiles/mmal_core.dir/flags.make
interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.o: ../../../interface/mmal/core/mmal_port.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.o"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mmal_core.dir/mmal_port.c.o   -c /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_port.c

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_core.dir/mmal_port.c.i"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_port.c > CMakeFiles/mmal_core.dir/mmal_port.c.i

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_core.dir/mmal_port.c.s"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_port.c -o CMakeFiles/mmal_core.dir/mmal_port.c.s

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.o.requires:
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.o.requires

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.o.provides: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.o.requires
	$(MAKE) -f interface/mmal/core/CMakeFiles/mmal_core.dir/build.make interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.o.provides.build
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.o.provides

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.o.provides.build: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.o

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.o: interface/mmal/core/CMakeFiles/mmal_core.dir/flags.make
interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.o: ../../../interface/mmal/core/mmal_port_clock.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.o"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mmal_core.dir/mmal_port_clock.c.o   -c /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_port_clock.c

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_core.dir/mmal_port_clock.c.i"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_port_clock.c > CMakeFiles/mmal_core.dir/mmal_port_clock.c.i

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_core.dir/mmal_port_clock.c.s"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_port_clock.c -o CMakeFiles/mmal_core.dir/mmal_port_clock.c.s

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.o.requires:
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.o.requires

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.o.provides: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.o.requires
	$(MAKE) -f interface/mmal/core/CMakeFiles/mmal_core.dir/build.make interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.o.provides.build
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.o.provides

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.o.provides.build: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.o

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.o: interface/mmal/core/CMakeFiles/mmal_core.dir/flags.make
interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.o: ../../../interface/mmal/core/mmal_component.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.o"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mmal_core.dir/mmal_component.c.o   -c /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_component.c

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_core.dir/mmal_component.c.i"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_component.c > CMakeFiles/mmal_core.dir/mmal_component.c.i

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_core.dir/mmal_component.c.s"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_component.c -o CMakeFiles/mmal_core.dir/mmal_component.c.s

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.o.requires:
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.o.requires

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.o.provides: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.o.requires
	$(MAKE) -f interface/mmal/core/CMakeFiles/mmal_core.dir/build.make interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.o.provides.build
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.o.provides

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.o.provides.build: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.o

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.o: interface/mmal/core/CMakeFiles/mmal_core.dir/flags.make
interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.o: ../../../interface/mmal/core/mmal_buffer.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.o"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mmal_core.dir/mmal_buffer.c.o   -c /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_buffer.c

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_core.dir/mmal_buffer.c.i"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_buffer.c > CMakeFiles/mmal_core.dir/mmal_buffer.c.i

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_core.dir/mmal_buffer.c.s"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_buffer.c -o CMakeFiles/mmal_core.dir/mmal_buffer.c.s

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.o.requires:
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.o.requires

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.o.provides: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.o.requires
	$(MAKE) -f interface/mmal/core/CMakeFiles/mmal_core.dir/build.make interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.o.provides.build
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.o.provides

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.o.provides.build: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.o

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.o: interface/mmal/core/CMakeFiles/mmal_core.dir/flags.make
interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.o: ../../../interface/mmal/core/mmal_queue.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.o"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mmal_core.dir/mmal_queue.c.o   -c /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_queue.c

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_core.dir/mmal_queue.c.i"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_queue.c > CMakeFiles/mmal_core.dir/mmal_queue.c.i

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_core.dir/mmal_queue.c.s"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_queue.c -o CMakeFiles/mmal_core.dir/mmal_queue.c.s

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.o.requires:
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.o.requires

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.o.provides: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.o.requires
	$(MAKE) -f interface/mmal/core/CMakeFiles/mmal_core.dir/build.make interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.o.provides.build
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.o.provides

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.o.provides.build: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.o

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.o: interface/mmal/core/CMakeFiles/mmal_core.dir/flags.make
interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.o: ../../../interface/mmal/core/mmal_pool.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.o"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mmal_core.dir/mmal_pool.c.o   -c /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_pool.c

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_core.dir/mmal_pool.c.i"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_pool.c > CMakeFiles/mmal_core.dir/mmal_pool.c.i

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_core.dir/mmal_pool.c.s"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_pool.c -o CMakeFiles/mmal_core.dir/mmal_pool.c.s

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.o.requires:
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.o.requires

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.o.provides: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.o.requires
	$(MAKE) -f interface/mmal/core/CMakeFiles/mmal_core.dir/build.make interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.o.provides.build
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.o.provides

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.o.provides.build: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.o

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.o: interface/mmal/core/CMakeFiles/mmal_core.dir/flags.make
interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.o: ../../../interface/mmal/core/mmal_events.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.o"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mmal_core.dir/mmal_events.c.o   -c /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_events.c

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_core.dir/mmal_events.c.i"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_events.c > CMakeFiles/mmal_core.dir/mmal_events.c.i

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_core.dir/mmal_events.c.s"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_events.c -o CMakeFiles/mmal_core.dir/mmal_events.c.s

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.o.requires:
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.o.requires

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.o.provides: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.o.requires
	$(MAKE) -f interface/mmal/core/CMakeFiles/mmal_core.dir/build.make interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.o.provides.build
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.o.provides

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.o.provides.build: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.o

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.o: interface/mmal/core/CMakeFiles/mmal_core.dir/flags.make
interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.o: ../../../interface/mmal/core/mmal_logging.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.o"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mmal_core.dir/mmal_logging.c.o   -c /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_logging.c

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_core.dir/mmal_logging.c.i"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_logging.c > CMakeFiles/mmal_core.dir/mmal_logging.c.i

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_core.dir/mmal_logging.c.s"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_logging.c -o CMakeFiles/mmal_core.dir/mmal_logging.c.s

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.o.requires:
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.o.requires

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.o.provides: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.o.requires
	$(MAKE) -f interface/mmal/core/CMakeFiles/mmal_core.dir/build.make interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.o.provides.build
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.o.provides

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.o.provides.build: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.o

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.o: interface/mmal/core/CMakeFiles/mmal_core.dir/flags.make
interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.o: ../../../interface/mmal/core/mmal_clock.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/CMakeFiles $(CMAKE_PROGRESS_10)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.o"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mmal_core.dir/mmal_clock.c.o   -c /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_clock.c

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mmal_core.dir/mmal_clock.c.i"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -E /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_clock.c > CMakeFiles/mmal_core.dir/mmal_clock.c.i

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mmal_core.dir/mmal_clock.c.s"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && /usr/bin/gcc  $(C_DEFINES) $(C_FLAGS) -S /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core/mmal_clock.c -o CMakeFiles/mmal_core.dir/mmal_clock.c.s

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.o.requires:
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.o.requires

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.o.provides: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.o.requires
	$(MAKE) -f interface/mmal/core/CMakeFiles/mmal_core.dir/build.make interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.o.provides.build
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.o.provides

interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.o.provides.build: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.o

# Object files for target mmal_core
mmal_core_OBJECTS = \
"CMakeFiles/mmal_core.dir/mmal_format.c.o" \
"CMakeFiles/mmal_core.dir/mmal_port.c.o" \
"CMakeFiles/mmal_core.dir/mmal_port_clock.c.o" \
"CMakeFiles/mmal_core.dir/mmal_component.c.o" \
"CMakeFiles/mmal_core.dir/mmal_buffer.c.o" \
"CMakeFiles/mmal_core.dir/mmal_queue.c.o" \
"CMakeFiles/mmal_core.dir/mmal_pool.c.o" \
"CMakeFiles/mmal_core.dir/mmal_events.c.o" \
"CMakeFiles/mmal_core.dir/mmal_logging.c.o" \
"CMakeFiles/mmal_core.dir/mmal_clock.c.o"

# External object files for target mmal_core
mmal_core_EXTERNAL_OBJECTS =

../../lib/libmmal_core.so: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.o
../../lib/libmmal_core.so: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.o
../../lib/libmmal_core.so: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.o
../../lib/libmmal_core.so: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.o
../../lib/libmmal_core.so: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.o
../../lib/libmmal_core.so: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.o
../../lib/libmmal_core.so: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.o
../../lib/libmmal_core.so: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.o
../../lib/libmmal_core.so: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.o
../../lib/libmmal_core.so: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.o
../../lib/libmmal_core.so: interface/mmal/core/CMakeFiles/mmal_core.dir/build.make
../../lib/libmmal_core.so: ../../lib/libvcos.so
../../lib/libmmal_core.so: interface/mmal/core/CMakeFiles/mmal_core.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C shared library ../../../../../lib/libmmal_core.so"
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mmal_core.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
interface/mmal/core/CMakeFiles/mmal_core.dir/build: ../../lib/libmmal_core.so
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/build

interface/mmal/core/CMakeFiles/mmal_core.dir/requires: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_format.c.o.requires
interface/mmal/core/CMakeFiles/mmal_core.dir/requires: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port.c.o.requires
interface/mmal/core/CMakeFiles/mmal_core.dir/requires: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_port_clock.c.o.requires
interface/mmal/core/CMakeFiles/mmal_core.dir/requires: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_component.c.o.requires
interface/mmal/core/CMakeFiles/mmal_core.dir/requires: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_buffer.c.o.requires
interface/mmal/core/CMakeFiles/mmal_core.dir/requires: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_queue.c.o.requires
interface/mmal/core/CMakeFiles/mmal_core.dir/requires: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_pool.c.o.requires
interface/mmal/core/CMakeFiles/mmal_core.dir/requires: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_events.c.o.requires
interface/mmal/core/CMakeFiles/mmal_core.dir/requires: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_logging.c.o.requires
interface/mmal/core/CMakeFiles/mmal_core.dir/requires: interface/mmal/core/CMakeFiles/mmal_core.dir/mmal_clock.c.o.requires
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/requires

interface/mmal/core/CMakeFiles/mmal_core.dir/clean:
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core && $(CMAKE_COMMAND) -P CMakeFiles/mmal_core.dir/cmake_clean.cmake
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/clean

interface/mmal/core/CMakeFiles/mmal_core.dir/depend:
	cd /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/RCR/LIB/OpenCV/userland /home/pi/RCR/LIB/OpenCV/userland/interface/mmal/core /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core /home/pi/RCR/LIB/OpenCV/userland/build/raspberry/release/interface/mmal/core/CMakeFiles/mmal_core.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : interface/mmal/core/CMakeFiles/mmal_core.dir/depend

