#pragma once

class World;

class Solver
{
  public:
    void integrate(World& world, float dt);
    void constraints(World& world);
    void collisions(World& world);
};
