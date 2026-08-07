#include "particle2d/solver/solver.hpp"
#include "particle2d/joint/joint.hpp"
#include "particle2d/vector.hpp"
#include "particle2d/world/world.hpp"
#include "particle2d/particle/particle.hpp"

#include <cmath>
#include <cstdint>

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

void Solver::constraints(World& world, float dt, uint16_t iterations)
{
  for (Joint& joint : world.joints)
  {
    Particle& a = world.getParticle(joint.a);
    Particle& b = world.getParticle(joint.b);

    float k = powf(joint.shiftness, 1.0f / iterations);
    
    Vec2 delta = a.pos - b.pos;
    
    float dist = delta.length();

    if (dist == 0.0f)
      continue;

    Vec2 n = delta / dist;

    float error = dist - joint.length;

    float w1 = a.inverseMass;
    float w2 = b.inverseMass;

    float sum = w1 + w2;

    if (sum == 0.0f)
      continue;

    Vec2 correction = n * error;

    a.pos += correction * (w1 / sum) * k;
    b.pos -= correction * (w2 / sum) * k;
  }

  for (AngleJoint& anglej : world.anglejoints)
  {
    Particle& a = world.getParticle(anglej.a);
    Particle& b = world.getParticle(anglej.b);
    Particle& c = world.getParticle(anglej.c);

    float k = powf(anglej.shiftness, 1.0f / iterations);

    Vec2 ab = a.pos - b.pos;
    Vec2 bc = c.pos - b.pos;

    AngleRad angle = AngleRad(atan2f(cross(ab, bc), dot(ab, bc)) * (M_PI / 180));

    ab = ab.rotate(angle);
    bc = bc.rotate(angle);

    if (angle.rad < AngleRad(anglej.minAngle).rad)
    {

    }
  }
}
