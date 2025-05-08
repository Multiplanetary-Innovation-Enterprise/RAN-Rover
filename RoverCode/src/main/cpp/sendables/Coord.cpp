#include "sendables/Coord.h"
#include <fmt/format.h>

Coord::Coord() { x = 0.0, y = 0.0; }
Coord::Coord(double x, double y): x(x), y(y) {}

double Coord::dist(Coord other) {
    return fabs(x - other.x) + fabs(y - other.y);
}

std::string Coord::toStr() {
    return fmt::format("<{:.2f}, {:.2f}>", x, y);
}

Coord Coord::operator+(Coord other) {
    return Coord{x + other.x, y + other.y};
}

Coord Coord::operator-(Coord other) {
    return Coord{x - other.x, y - other.y};
}

Coord Coord::operator*(Coord other) {
    return Coord{x * other.x, y * other.y};
}

Coord Coord::operator/(Coord other) {
    return Coord{x / other.x, y / other.y};
}

Coord Coord::operator+(const Coord other) const {
    return Coord{x + other.x, y + other.y};
}

Coord Coord::operator-(const Coord other) const {
    return Coord{x - other.x, y - other.y};
}

Coord Coord::operator*(const Coord other) const {
    return Coord{x * other.x, y * other.y};
}

Coord Coord::operator/(const Coord other) const {
    return Coord{x / other.x, y / other.y};
}

Coord Coord::operator+=(Coord other) {
    x += other.x;
    y += other.y;
    return *this;
}

Coord Coord::operator-=(Coord other) {
    x -= other.x;
    y -= other.y;
    return *this;
}

Coord Coord::operator*=(Coord other) {
    x *= other.x;
    y *= other.y;
    return *this;
}

Coord Coord::operator/=(Coord other) {
    x /= other.x;
    y /= other.y;
    return *this;
}

bool Coord::operator>(Coord other) {
    return (x > other.x && y > other.y);
}

bool Coord::operator>=(Coord other) {
    return (x >= other.x && y >= other.y);
}

bool Coord::operator<(Coord other) {
    return (x < other.x && y < other.y);
}

bool Coord::operator<=(Coord other) {
    return (x <= other.x && y <= other.y);
}

bool Coord::operator==(Coord other) {
    return (x == other.x && y == other.y);
}

bool Coord::operator!=(Coord other) {
    return (x != other.x && y != other.y);
}