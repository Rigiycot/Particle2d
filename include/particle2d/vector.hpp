#pragma once

struct AngleRad;

struct AngleGrad
{
  float grad;

  AngleRad toRad() const;

  AngleGrad() = default;
  AngleGrad(float angle);
  AngleGrad(const AngleRad& angle);
};

struct AngleRad
{
  float rad;

  AngleGrad toGrad() const;

  AngleRad() = default;
  AngleRad(float angle);
  AngleRad(const AngleGrad& angle);
};


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

  Vec2 rotate(const AngleGrad& angle) const;
  Vec2 rotate(const AngleRad&  angle) const;

  Vec2 perpendicular() const;
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

  Edge rotate(const AngleGrad& angle) const;
  Edge rotate(const AngleRad&  angle) const;
  Edge rotate(const Vec2& pivot, const AngleGrad& angle) const;
  Edge rotate(const Vec2& pivot, const AngleRad&  angle) const;
  
  Vec2 center() const;
};
