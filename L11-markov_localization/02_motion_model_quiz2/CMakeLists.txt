cmake_minimum_required(VERSION 2.8.11)
project(1D_FILTER)



# Use C++11
set(SRCS src/main.cpp src/bayesianFilter.cpp )
set_source_files_properties(${SRCS} PROPERTIES COMPILE_FLAGS -std=c++0x)

# Create the executable
add_executable(1d_markov_loc ${SRCS})

