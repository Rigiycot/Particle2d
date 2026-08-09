#pragma once

#include "particle2d/types.hpp"

#include <array>
#include <cstddef>
#include <vector>

class Body
{
  public:
    ShapeID shape;
    uint64_t category;
    uint64_t collides;
};

enum class ShapeType
{
  None,
  Rectangle,
  Circle,
  Capsule,
  Polygon,
  DynamicPolygon
};

class Shape
{
  private:
    ShapeType type;
    friend class Solver;
  protected:
    Shape(ShapeType type = ShapeType::None) : type(type) {};
  public:
    virtual ~Shape() = default;
};

class RectangleShape : public Shape
{
  public:
    RectangleShape()
      : Shape(ShapeType::Rectangle)
    {}

    std::array<ParticleID, 4> points;
};

class CircleShape : public Shape
{
  public:
    CircleShape()
      : Shape(ShapeType::Circle)
    {}
    float radius;
    ParticleID center;
};

class CapsuleShape : public Shape
{
  public:
    CapsuleShape()
      : Shape(ShapeType::Capsule)
    {}
    ParticleID a;
    ParticleID b;

    float radius;
};

template <size_t N>
class PolygonShape : public Shape
{
  public:
    PolygonShape()
      : Shape(ShapeType::Polygon)
    {}
    std::array<ParticleID, N> points;
};

class DynamicPolygonShape : public Shape
{
  public:
    DynamicPolygonShape()
      : Shape(ShapeType::DynamicPolygon)
    {}
    std::vector<ParticleID> points;

    void addVerteVertex(ParticleID addParticle);
};
