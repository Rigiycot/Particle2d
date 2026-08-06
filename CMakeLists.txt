cmake_minimum_required(VERSION 3.16)

project(Particle2d)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(Particle2d 
  src/world/world.cpp
  src/particle/particle.cpp 
  src/joint/joint.cpp 
  src/solver/solver.cpp 
  src/particle/vector.cpp
)

target_compile_features(Particle2d PUBLIC cxx_std_20)

target_include_directories( Particle2d
  PUBLIC include
)

