#include "particle2d/solver/solver.hpp"
#include "particle2d/world/world.hpp"
#include "particle2d/particle/particle.hpp"

void Solver::integrate(World& world, float dt)
{
  for (Particle& p : world.particles)
  {
    Vec2 velocity = p.pos - p.prevPos;

    p.prevPos = p.pos;

    p.pos += velocity;
    p.pos += world.gravity * dt * dt;
  }
}
