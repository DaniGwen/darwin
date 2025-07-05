# CMake build script for the Darwin-OP ONNX Reinforcement Learning Agent

cmake_minimum_required(VERSION 3.10)
project(DarwinRLAgent)

# --- Project Configuration ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Library Paths (IMPORTANT: You MUST change these) ---
# Set the path to where you have the ONNX Runtime library installed.
set(ONNXRUNTIME_DIR "/path/to/your/onnxruntime-linux-x64-1.15.1" CACHE PATH "Path to ONNX Runtime directory")

# Set the path to your Darwin-OP C++ Framework directory.
# This should contain the 'include' and 'lib' folders for the robot's SDK.
set(DARWIN_FRAMEWORK_DIR "/path/to/your/DarwinFramework" CACHE PATH "Path to Darwin-OP Framework directory")

# --- Find ONNX Runtime Library ---
message(STATUS "Looking for ONNX Runtime in: ${ONNXRUNTIME_DIR}")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
          HINTS ${ONNXRUNTIME_DIR}/include)
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
             HINTS ${ONNXRUNTIME_DIR}/lib)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
    message(FATAL_ERROR "ONNX Runtime not found. Please set ONNXRUNTIME_DIR correctly.")
else()
    message(STATUS "Found ONNX Runtime Include: ${ONNXRUNTIME_INCLUDE_DIR}")
    message(STATUS "Found ONNX Runtime Library: ${ONNXRUNTIME_LIBRARY}")
endif()


# --- Find Darwin Framework Library ---
# This assumes the library is named 'libDARwIn.so' or similar. Adjust if needed.
message(STATUS "Looking for Darwin Framework in: ${DARWIN_FRAMEWORK_DIR}")
find_path(DARWIN_INCLUDE_DIR LinuxDARwIn.h
          HINTS ${DARWIN_FRAMEWORK_DIR}/include)
find_library(DARWIN_LIBRARY NAMES DARwIn
             HINTS ${DARWIN_FRAMEWORK_DIR}/lib)

if(NOT DARWIN_INCLUDE_DIR OR NOT DARWIN_LIBRARY)
    message(FATAL_ERROR "Darwin Framework not found. Please set DARWIN_FRAMEWORK_DIR correctly.")
else()
    message(STATUS "Found Darwin Include: ${DARWIN_INCLUDE_DIR}")
    message(STATUS "Found Darwin Library: ${DARWIN_LIBRARY}")
endif()


# --- Create the Executable ---
add_executable(darwin_rl_agent darwin_rl_agent.cpp)

# --- Link Libraries ---
target_include_directories(darwin_rl_agent PRIVATE
    ${ONNXRUNTIME_INCLUDE_DIR}
    ${DARWIN_INCLUDE_DIR}
)

target_link_libraries(darwin_rl_agent PRIVATE
    ${ONNXRUNTIME_LIBRARY}
    ${DARWIN_LIBRARY}
    pthread # The Darwin framework often requires the pthread library
)

# --- Installation ---
# This step is optional but good practice. It allows you to 'make install'.
install(TARGETS darwin_rl_agent DESTINATION bin)