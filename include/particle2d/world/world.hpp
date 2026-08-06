#pragma once

#include "particle2d/particle/particle.hpp"
#include "particle2d/joint/joint.hpp"
#include "particle2d/particle/shape.hpp"
#include "particle2d/types.hpp"

#include "particle2d/solver/solver.hpp"

#include <unordered_map>
#include <vector>
#include <cstdint>

class World 
{
  public:
    void step(float dt = 1.0f / 60.0f, uint8_t iterations = 3);

  private:
    Solver solver;

    std::vector<Particle> particles;
    std::unordered_map<ParticleID, uint32_t> particleIdxToVec;
    
    std::vector<Joint> joints;
    std::unordered_map<JointID, uint32_t> jointIdxToVec;

    std::vector<Shape> shapes;
    std::unordered_map<ShapeID, uint32_t> shapeIdxToVec;
};
