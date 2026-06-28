cmake_minimum_required(VERSION 3.20)

project(UAMSimulatorModern LANGUAGES CXX OBJCXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_OBJCXX_STANDARD 20)
set(CMAKE_OBJCXX_STANDARD_REQUIRED ON)

if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(CMAKE_OSX_DEPLOYMENT_TARGET "15.0" CACHE STRING "Minimum macOS version" FORCE)
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    set(HOMEBREW_PREFIX "/opt/homebrew/opt")
else()
    set(HOMEBREW_PREFIX "/usr/local/opt")
endif()

find_package(yaml-cpp REQUIRED)
find_package(glm REQUIRED)

add_executable(UAM-Simulator-Modern
    src/main.mm
    src/core/SimulatorCore.cpp
    src/core/SceneMesh.cpp
    src/core/SensorGeometry.cpp
    src/metal/MetalMesh.mm
)

set_source_files_properties(src/main.mm PROPERTIES COMPILE_FLAGS "-fobjc-arc")

target_include_directories(UAM-Simulator-Modern PRIVATE
    include
    src
)

target_link_libraries(UAM-Simulator-Modern PRIVATE
    yaml-cpp::yaml-cpp
    glm::glm
    "-framework Cocoa"
    "-framework Metal"
    "-framework MetalKit"
    "-framework QuartzCore"
)

target_compile_options(UAM-Simulator-Modern PRIVATE -Wall -Wextra -Wpedantic)
