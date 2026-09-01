#pragma once
#include <string>
#include <sstream>
#include <cstdint>
#include <cmath>
#include <iostream>

typedef uint8_t pros_adi_port;
typedef int8_t pros_standard_port;
typedef void (*SimpleCallback)();

template <typename T>
std::string to_string(T value)
{
        std::ostringstream os;
        os << value;
        return os.str();
}

class Vector2 {
public:
        float X;
        float Y;

        Vector2(void);
        Vector2(float x, float y);

        Vector2 operator+(const Vector2& other);
        Vector2 operator-(const Vector2& other);
        Vector2 operator*(const float& other);
        Vector2 operator/(const float& other);
        Vector2 operator=(const Vector2& other);

        Vector2 operator+=(const Vector2& other);
        Vector2 operator-=(const Vector2& other);
        Vector2 operator*=(const float& other);
        Vector2 operator/=(const float& other);

        // Unary minus
        Vector2 operator-();


        float MagSquared(void);
        float Mag(void);
        float DistanceSquared(Vector2 other);
        float Distance(Vector2 other);

        float Dot(Vector2 other);
        float Cross2(Vector2 other);

        static float Arg(Vector2 vec);
        static float ArgD(Vector2 vec); // Pls don't use, try always use rads

        float AngleBetween(Vector2 vec);

        std::string ToString(void);
};

const float IN_TO_MM = 2.54f;
const float MM_TO_IN = 1 / IN_TO_MM;

const float PI = 3.142f;
const float SQRT2 = 1.414f;

const float MILLISECONDS_TO_SECONDS = 0.001f;
const float SECONDS_TO_MILLISECONDS = 1000.0f;

const float sin90 = 1.0f;
const float cos90 = 0.0f;

float Clamp(float value, float low, float high);
float FloatModulus(float value, float moduland);
bool Within(float value, float target, float plusMinus);
float SignedHeadingDifference(float value, float target);
bool WithinHeading(float value, float target, float plusMinus);