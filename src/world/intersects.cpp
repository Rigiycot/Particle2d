#include "particle2d/world/intersects.hpp"
#include "particle2d/particle/shape.hpp"
#include "particle2d/vector.hpp"

#include "particle2d/world/world.hpp"

#include <cfloat>

namespace 
{
  p2::Collision rectRect(const p2::RectangleShape& a, const p2::RectangleShape& b, p2::World world)
  {
    std::array<p2::Vec2, 4> pa;
    std::array<p2::Vec2, 4> pb;

    for (int i = 0; i < 4; ++i)
    {
      pa[i] = world.getParticle(a.points[i]).pos;
      pb[i] = world.getParticle(b.points[i]).pos;
    }

    p2::Vec2 axis1 = (pa[3] - pa[0]).perpendicular().norm();
    p2::Vec2 axis2 = (pa[1] - pa[0]).perpendicular().norm();

    p2::Vec2 axis3 = (pb[3] - pb[0]).perpendicular().norm();
    p2::Vec2 axis4 = (pb[1] - pb[0]).perpendicular().norm();

       
    
    float minOverlap = FLT_MAX;
    p2::Vec2 normal;

    for (const p2::Vec2& axis : {axis1, axis2, axis3, axis4})
    {
      float minA = dot(pa[0], axis);
      float maxA = minA;

      for (int i = 1; i < 4; ++i)
      {
        float p = dot(pa[i], axis);

        minA = std::min(minA, p);
        maxA = std::max(maxA, p);
      }

      float minB = dot(pa[0], axis);
      float maxB = minA;

      for (int i = 1; i < 4; ++i)
      {
        float p = dot(pb[i], axis);

        minB = std::min(minB, p);
        maxB = std::max(maxB, p);
      }

      if (minA < minB || maxB < maxA)
        return {-1.0f, {}};

      float overlap = std::min(maxA, maxB) - std::max(minA, minB);
      
      if (overlap < minOverlap)
      {
        minOverlap = overlap;
        normal = axis;
      }
    }
    
    p2::Vec2 centerA = (pa[0] + pa[1] + pa[2] + pa[3]) / 4;
    p2::Vec2 centerB = (pb[0] + pb[1] + pb[2] + pb[3]) / 4;

    if (dot(centerB - centerA, normal) < 0.0f)
      normal = -normal;

    return {minOverlap, normal};
  }

  p2::Collision rectCircle(const p2::RectangleShape& a, const p2::CircleShape& b)
  {
    
  }

  p2::Collision circleRect(const p2::CircleShape& a, const p2::RectangleShape& b)
  {
      // TODO
  }

  p2::Collision circleCircle(const p2::CircleShape& a, const p2::CircleShape& b)
  {
      // TODO
  }

  p2::Collision capsuleRect(const p2::CapsuleShape& a, const p2::RectangleShape& b)
  {
      // TODO
  }

  p2::Collision rectCapsule(const p2::RectangleShape& a, const p2::CapsuleShape& b)
  {
      // TODO
  }

  p2::Collision capsuleCircle(const p2::CapsuleShape& a, const p2::CircleShape& b)
  {
      // TODO
  }

  p2::Collision circleCapsule(const p2::CircleShape& a, const p2::CapsuleShape& b)
  {
      // TODO
  }

  p2::Collision capsuleCapsule(const p2::CapsuleShape& a, const p2::CapsuleShape& b)
  {
      // TODO
  }
}

p2::Collision p2::collide(p2::World world, const p2::Shape& a, const p2::Shape& b)
{
  if (a.type == p2::ShapeType::Rectangle && b.type == p2::ShapeType::Rectangle)
  {
    const RectangleShape& rect1 = static_cast<const RectangleShape&>(a);
    const RectangleShape& rect2 = static_cast<const RectangleShape&>(b);

    return rectRect(rect1, rect2, world);
  }

  return {-1.0f, {}};
}

bool p2::intersects(p2::World world, const Shape& a, const Shape& b)
{
    return collide(world, a, b).overlap > 0.0f;
}
