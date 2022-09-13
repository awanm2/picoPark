# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sharp/lyons/gH/picoPark/bme280

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sharp/lyons/gH/picoPark/bme280/build

# Utility rule file for ELF2UF2Build.

# Include any custom commands dependencies for this target.
include examples/BasicReadings/CMakeFiles/ELF2UF2Build.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/BasicReadings/CMakeFiles/ELF2UF2Build.dir/progress.make

examples/BasicReadings/CMakeFiles/ELF2UF2Build: examples/BasicReadings/CMakeFiles/ELF2UF2Build-complete

examples/BasicReadings/CMakeFiles/ELF2UF2Build-complete: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-install
examples/BasicReadings/CMakeFiles/ELF2UF2Build-complete: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-mkdir
examples/BasicReadings/CMakeFiles/ELF2UF2Build-complete: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-download
examples/BasicReadings/CMakeFiles/ELF2UF2Build-complete: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-update
examples/BasicReadings/CMakeFiles/ELF2UF2Build-complete: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-patch
examples/BasicReadings/CMakeFiles/ELF2UF2Build-complete: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure
examples/BasicReadings/CMakeFiles/ELF2UF2Build-complete: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-build
examples/BasicReadings/CMakeFiles/ELF2UF2Build-complete: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sharp/lyons/gH/picoPark/bme280/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'ELF2UF2Build'"
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E make_directory /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/CMakeFiles
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E touch /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/CMakeFiles/ELF2UF2Build-complete
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E touch /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-done

examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-build: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sharp/lyons/gH/picoPark/bme280/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing build step for 'ELF2UF2Build'"
	cd /home/sharp/lyons/gH/picoPark/bme280/build/elf2uf2 && $(MAKE)

examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure: examples/BasicReadings/elf2uf2/tmp/ELF2UF2Build-cfgcmd.txt
examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sharp/lyons/gH/picoPark/bme280/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Performing configure step for 'ELF2UF2Build'"
	cd /home/sharp/lyons/gH/picoPark/bme280/build/elf2uf2 && /usr/bin/cmake "-GUnix Makefiles" /home/sharp/pico/pico-sdk/tools/elf2uf2
	cd /home/sharp/lyons/gH/picoPark/bme280/build/elf2uf2 && /usr/bin/cmake -E touch /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure

examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-download: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sharp/lyons/gH/picoPark/bme280/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "No download step for 'ELF2UF2Build'"
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E echo_append
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E touch /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-download

examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-install: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sharp/lyons/gH/picoPark/bme280/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No install step for 'ELF2UF2Build'"
	cd /home/sharp/lyons/gH/picoPark/bme280/build/elf2uf2 && /usr/bin/cmake -E echo_append
	cd /home/sharp/lyons/gH/picoPark/bme280/build/elf2uf2 && /usr/bin/cmake -E touch /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-install

examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sharp/lyons/gH/picoPark/bme280/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Creating directories for 'ELF2UF2Build'"
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E make_directory /home/sharp/pico/pico-sdk/tools/elf2uf2
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E make_directory /home/sharp/lyons/gH/picoPark/bme280/build/elf2uf2
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E make_directory /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/elf2uf2
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E make_directory /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/elf2uf2/tmp
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E make_directory /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E make_directory /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/elf2uf2/src
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E make_directory /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E touch /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-mkdir

examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-patch: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-update
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sharp/lyons/gH/picoPark/bme280/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No patch step for 'ELF2UF2Build'"
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E echo_append
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E touch /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-patch

examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-update: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sharp/lyons/gH/picoPark/bme280/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No update step for 'ELF2UF2Build'"
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E echo_append
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && /usr/bin/cmake -E touch /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-update

ELF2UF2Build: examples/BasicReadings/CMakeFiles/ELF2UF2Build
ELF2UF2Build: examples/BasicReadings/CMakeFiles/ELF2UF2Build-complete
ELF2UF2Build: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-build
ELF2UF2Build: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-configure
ELF2UF2Build: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-download
ELF2UF2Build: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-install
ELF2UF2Build: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-mkdir
ELF2UF2Build: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-patch
ELF2UF2Build: examples/BasicReadings/elf2uf2/src/ELF2UF2Build-stamp/ELF2UF2Build-update
ELF2UF2Build: examples/BasicReadings/CMakeFiles/ELF2UF2Build.dir/build.make
.PHONY : ELF2UF2Build

# Rule to build all files generated by this target.
examples/BasicReadings/CMakeFiles/ELF2UF2Build.dir/build: ELF2UF2Build
.PHONY : examples/BasicReadings/CMakeFiles/ELF2UF2Build.dir/build

examples/BasicReadings/CMakeFiles/ELF2UF2Build.dir/clean:
	cd /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings && $(CMAKE_COMMAND) -P CMakeFiles/ELF2UF2Build.dir/cmake_clean.cmake
.PHONY : examples/BasicReadings/CMakeFiles/ELF2UF2Build.dir/clean

examples/BasicReadings/CMakeFiles/ELF2UF2Build.dir/depend:
	cd /home/sharp/lyons/gH/picoPark/bme280/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sharp/lyons/gH/picoPark/bme280 /home/sharp/lyons/gH/picoPark/bme280/examples/BasicReadings /home/sharp/lyons/gH/picoPark/bme280/build /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings /home/sharp/lyons/gH/picoPark/bme280/build/examples/BasicReadings/CMakeFiles/ELF2UF2Build.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/BasicReadings/CMakeFiles/ELF2UF2Build.dir/depend

