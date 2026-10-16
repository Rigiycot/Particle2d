#include "particle2d/world/intersects.hpp"
#include "particle2d/particle/shape.hpp"
#include "particle2d/vector.hpp"

#include "particle2d/world/world.hpp"

#include <cfloat>
#include <algorithm>

namespace 
{
  p2::Collision rectRect(
  p2::World& world,
  const p2::RectangleShape& a,
  const p2::RectangleShape& b)
{
  std::array<p2::Vec2, 4> pa;
  std::array<p2::Vec2, 4> pb;

  for (int i = 0; i < 4; ++i)
  {
    pa[i] = world.getParticle(a.points[i]).pos;
    pb[i] = world.getParticle(b.points[i]).pos;
  }

  p2::Vec2 axes[] =
  {
    (pa[3] - pa[0]).perpendicular().norm(),
    (pa[1] - pa[0]).perpendicular().norm(),
    (pb[3] - pb[0]).perpendicular().norm(),
    (pb[1] - pb[0]).perpendicular().norm()
  };

  float minOverlap = FLT_MAX;
  p2::Vec2 normal;

  for (const p2::Vec2& axis : axes)
  {
    float minA = dot(pa[0], axis);
    float maxA = minA;

    float minB = dot(pb[0], axis);
    float maxB = minB;

    for (int i = 1; i < 4; ++i)
    {
      float projA = dot(pa[i], axis);
      float projB = dot(pb[i], axis);

      minA = std::min(minA, projA);
      maxA = std::max(maxA, projA);

      minB = std::min(minB, projB);
      maxB = std::max(maxB, projB);
    }

    if (maxA < minB || maxB < minA)
      return {-1.0f, {}, {}};

    float overlap =
      std::min(maxA, maxB) -
      std::max(minA, minB);

    if (overlap < minOverlap)
    {
      minOverlap = overlap;
      normal = axis;
    }
  }

  p2::Vec2 centerA =
    (pa[0] + pa[1] + pa[2] + pa[3]) * 0.25f;

  p2::Vec2 centerB =
    (pb[0] + pb[1] + pb[2] + pb[3]) * 0.25f;

  if (dot(centerB - centerA, normal) < 0.0f)
    normal = -normal;

  int edgeA = 0;
  int edgeB = 0;

  float bestA = -FLT_MAX;
  float bestB = -FLT_MAX;

  for (int i = 0; i < 4; ++i)
  {
    int j = (i + 1) % 4;

    p2::Vec2 midA =
      (pa[i] + pa[j]) * 0.5f;

    p2::Vec2 midB =
      (pb[i] + pb[j]) * 0.5f;

    float projA = dot(midA, normal);
    float projB = dot(midB, -normal);

    if (projA > bestA)
    {
      bestA = projA;
      edgeA = i;
    }

    if (projB > bestB)
    {
      bestB = projB;
      edgeB = i;
    }
  }

  p2::Vec2 a1 = pa[edgeA];
  p2::Vec2 a2 = pa[(edgeA + 1) % 4];

  p2::Vec2 b1 = pb[edgeB];
  p2::Vec2 b2 = pb[(edgeB + 1) % 4];

  p2::Vec2 tangent =
    (a2 - a1).norm();

  float aMin = dot(a1, tangent);
  float aMax = dot(a2, tangent);

  if (aMin > aMax)
    std::swap(aMin, aMax);

  float bMin = dot(b1, tangent);
  float bMax = dot(b2, tangent);

  if (bMin > bMax)
    std::swap(bMin, bMax);

  float contactMin =
    std::max(aMin, bMin);

  float contactMax =
    std::min(aMax, bMax);

  p2::Vec2 point;

  if (contactMin <= contactMax)
  {
    float contactMiddle =
      (contactMin + contactMax) * 0.5f;

    point =
      a1 +
      tangent *
      (contactMiddle - dot(a1, tangent));
  }
  else
  {
    auto closestPoint =
      [](p2::Vec2 p, p2::Vec2 x, p2::Vec2 y)
      {
        p2::Vec2 edge = y - x;
        float lengthSq = edge.lengthSquared();

        if (lengthSq == 0.0f)
          return x;

        float t = dot(p - x, edge) / lengthSq;

        t = std::clamp(t, 0.0f, 1.0f);

        return x + edge * t;
      };

    p2::Vec2 pointOnA =
      closestPoint(b1, a1, a2);

    p2::Vec2 pointOnB =
      closestPoint(pointOnA, b1, b2);

    point =
      (pointOnA + pointOnB) * 0.5f;
  }

  return {
    minOverlap,
    normal,
    point
  };
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

p2::Collision p2::collide(p2::World& world, const p2::Shape& a, const p2::Shape& b)
{
  if (a.type == p2::ShapeType::Rectangle && b.type == p2::ShapeType::Rectangle)
  {
    const RectangleShape& rect1 = static_cast<const RectangleShape&>(a);
    const RectangleShape& rect2 = static_cast<const RectangleShape&>(b);

    return rectRect(world, rect1, rect2);
  }

  return {-1.0f, {}, {}};
}

bool p2::intersects(p2::World& world, const Shape& a, const Shape& b)
{
    return collide(world, a, b).overlap >= 0.0f;
}
