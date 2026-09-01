#include "helpers.hpp"

Vector2::Vector2() {
        this->X = 0;
        this->Y = 0;
}

Vector2::Vector2(float x, float y) {
        this->X = x;
        this->Y = y;
}

Vector2 Vector2::operator+(const Vector2& other) {
        return Vector2(other.X + this->X, other.Y + this->Y);
}

Vector2 Vector2::operator-(const Vector2& other) {
        return Vector2(this->X - other.X, this->Y - other.Y);
}

Vector2 Vector2::operator*(const float& other) {
        return Vector2(this->X * other, this->Y * other);
}

Vector2 Vector2::operator/(const float& other) {
        return Vector2(this->X * (1 / other), this->Y * (1 / other));
}

Vector2 Vector2::operator-() {
        return Vector2(-this->X, -this->Y);
}

Vector2 Vector2::operator=(const Vector2& other) {
        this->X = other.X;
        this->Y = other.Y;
        return *this;
}

Vector2 Vector2::operator+=(const Vector2& other) {
        this->X += other.X;
        this->Y += other.Y;
        return *this;
}

Vector2 Vector2::operator-=(const Vector2& other) {
        this->X -= other.X;
        this->Y -= other.Y;
        return *this;
}

Vector2 Vector2::operator*=(const float& other) {
        this->X *= other;
        this->Y *= other;
        return *this;
}

// Broken TODO: fix
Vector2 Vector2::operator/=(const float& other) {
        if (other == 0.0f) throw std::exception();

        this->X /= other;
        this->Y /= other;
        return *this;
}

float Vector2::MagSquared() {
        return this->X * this->X + this->Y * this->Y;
}

float Vector2::Mag() {
        std::cerr << "Warning: Vector2::Mag() is slow. Use Vector2::MagSquared() instead." << std::endl;
        return sqrt(this->MagSquared());
}

float Vector2::DistanceSquared(Vector2 other) { // distance to another vector (position)
        return (this->X - other.X) * (this->X - other.X) + (this->Y - other.Y) * (this->Y - other.Y); // could be optimised maybe?
}

float Vector2::Distance(Vector2 other) {
        std::cerr << "Warning: Vector2::Distance() is slow. Use Vector2::DistanceSquared() instead." << std::endl;
        return sqrt(this->DistanceSquared(other));
}

float Vector2::Dot(Vector2 other) {
        return this->X * other.X + this->Y * other.Y;
}

float Vector2::Cross2(Vector2 other) { // Cross product in 2D
        return this->X * other.Y - this->Y * other.X;
}

float Vector2::Arg(Vector2 vec) {
        return atan2(vec.Y, vec.X);
}

float Vector2::ArgD(Vector2 vec) {
        return Vector2::Arg(vec) * (180.0 / PI);
}

float Vector2::AngleBetween(Vector2 vec) {
        return acos(this->Dot(vec) / this->Mag() * vec.Mag());
}

float FloatModulus(float value, float moduland) {
        if (value > 0)
                while (value > moduland)
                        value -= moduland;
        else if (value < 0)
                while (value < 0)
                        value += moduland;

        return value;
}

float Clamp(float value, float low, float high) {
        return value > high
                ? high
                : value < low
                ? low
                : value;
}

bool Within(float value, float target, float plusMinus) {
        return std::abs(target - value) < plusMinus;
}

float SignedHeadingDifference(float value, float target) {
        return FloatModulus(target - value + 180, 360) - 180;
}

bool WithinHeading(float value, float target, float plusMinus) {
        return std::abs(SignedHeadingDifference(value, target)) < plusMinus;
}

std::string Vector2::ToString(void) {
        return "(" + to_string(this->X) + ", " + to_string(this->Y) + ")";
}