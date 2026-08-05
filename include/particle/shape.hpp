#pragma once

#include "types.hpp"

#include <array>
#include <cstddef>
#include <vector>

enum class BodyType
{
  Static,
  Dynamic
};

class Body
{
  BodyType type;
  std::vector<ShapeID> shapes;
};

class Shape
{
  public:
    virtual ~Shape() = default;
};

class RectangleShape : public Shape
{
  public:
    std::array<ParticleID, 4> points;
};

class CircleShape : public Shape
{
  public:
    float radius;
    ParticleID center;
};

class CapuleShape : public Shape
{
  public:
    ParticleID a;
    ParticleID b;

    float radius;
};

template <size_t N>
class PolygonShape : public Shape
{
  public:
    std::array<ParticleID, N> points;
};

class DynamicPolygonShape : public Shape
{
  public:
    std::vector<ParticleID> points;

    void addVerteVertex(ParticleID addParticle);
};
