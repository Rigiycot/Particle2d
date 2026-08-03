cmake_minimum_required(VERSION 3.16)

project(PhysicsEngine)

add_library(PhysicsEngine 
  src/world/world.cpp
  src/particle/particle.cpp 
  src/joint/joint.cpp 
  src/solver/solver.cpp 
  src/particle/vector.cpp
)

target_compile_features(PhysicsEngine PUBLIC cxx_std_20)

target_include_directories( PhysicsEngine
  PUBLIC include
)

