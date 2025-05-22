#pragma once
#include <string>
#include <cmath>

class Coord {
    public:
        Coord();
        Coord(double x, double y);
        double x, y;

        std::string toStr();
        double dist(Coord other);

        Coord operator+(Coord other);
        Coord operator-(Coord other);
        Coord operator*(Coord other);
        Coord operator/(Coord other);

        Coord operator+(const Coord other) const;
        Coord operator-(const Coord other) const;
        Coord operator*(const Coord other) const;
        Coord operator/(const Coord other) const;

        Coord operator+=(Coord other);
        Coord operator-=(Coord other);
        Coord operator*=(Coord other);
        Coord operator/=(Coord other);

        bool operator>(Coord other);
        bool operator>=(Coord other);
        bool operator<(Coord other);
        bool operator<=(Coord other);
        bool operator==(Coord other);
        bool operator!=(Coord other);
};