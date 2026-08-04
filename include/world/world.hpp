#pragma once

#include <particle/particle.hpp>

#include <unordered_map>
#include <vector>
#include <cstdint>

using ParticleID = uint32_t;

class World 
{
  public:
    void step(const float dt, uint8_t solvers);

  private:
    void integrate(ParticleID particleid, const Vec2& gravity);


    std::vector<Particle> particles;
    std::unordered_map<ParticleID, uint32_t> idxtovec;


};
