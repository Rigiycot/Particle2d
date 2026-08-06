#pragma once

struct Vec2
{
  float x;
  float y;

  Vec2(float x, float y);

  Vec2() = default;

  float length() const;
  float lengthSquared() const;
  Vec2 norm() const;

  Vec2 operator+(const Vec2& other) const;
  Vec2 operator-(const Vec2& other) const;
  Vec2 operator*(const float scalar) const;
  Vec2 operator/(const float scalar) const;

  Vec2 operator-() const;

  Vec2& operator+=(const Vec2& other);
  Vec2& operator-=(const Vec2& other);
  Vec2& operator*=(const float scalar);
  Vec2& operator/=(const float scalar);

  bool operator==(const Vec2& other) const;
  bool operator!=(const Vec2& other) const;

  bool equal(const Vec2& other, float epsilon) const;
  bool nequal(const Vec2& other, float epsilon) const;

  Vec2 rotate(float angle);

  Vec2 rotated(float angle);
};

float dot(const Vec2& a, const Vec2& b);
float cross(const Vec2& a, const Vec2& b);

struct Edge
{
  Vec2 a;
  Vec2 b;

  Edge() = default;

  Edge(const Vec2& a, const Vec2& b);
  Edge(const Edge& other);

  float length() const;
  float lengthSquared() const;

  Edge rotate(float angle) const;
  Edge rotate(const Vec2& pivot, float angle) const;
  
  Vec2 center() const;
};
