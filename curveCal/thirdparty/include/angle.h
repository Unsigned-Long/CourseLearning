#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

namespace ns_angle
{
#pragma region global
    class Degree;
    class Radian;
    constexpr double PI = M_PI;
    Radian operator+(const Radian &radian, const Degree &degree);
    Degree operator+(const Degree &degree, const Radian &radian);
    Radian operator-(const Radian &radian, const Degree &degree);
    Degree operator-(const Degree &degree, const Radian &radian);
    Degree operator"" _Deg(long double v);
    Radian operator"" _Rad(long double v);
    std::string toAngleStrExp(const Degree &degree, std::size_t dec = 1);
    std::string toAngleStrExp(const Radian &radian, std::size_t dec = 1);
#pragma endregion

#pragma region Degree class
    class Degree
    {
        friend Radian;
        friend std::string toAngleStrExp(const Degree &degree, std::size_t dec);
        friend std::ostream &operator<<(std::ostream &os, const Degree &degree);
        friend Degree operator/(float value, const Degree &degree);

    private:
        float _degree;

    public:
        Degree() = default;
        Degree(float degree) : _degree(degree) {}
        operator float() const;
        explicit operator Radian() const;
        Radian toRadian() const;
        Degree operator+(const Degree &degree) const;
        Degree operator+() const;
        Degree operator-(const Degree &degree) const;
        Degree operator-() const;
        Degree operator*(float value) const;
        Degree operator/(float value) const;
        Degree &operator+=(const Degree &degree);
        Degree &operator-=(const Degree &degree);
        Degree &operator+=(const Radian &radian);
        Degree &operator-=(const Radian &radian);
        Degree &operator*=(float value);
        Degree &operator/=(float value);
        ~Degree() {}
    };
    std::ostream &operator<<(std::ostream &os, const Degree &degree);
    Degree operator*(float value, const Degree &degree);
    Degree operator/(float value, const Degree &degree);

#pragma endregion

#pragma region Radian class
    class Radian
    {
        friend Degree;
        friend std::string toAngleStrExp(const Radian &radian, std::size_t dec);
        friend std::ostream &operator<<(std::ostream &os, const Radian &radian);
        friend Radian operator/(float value, const Radian &radian);

    private:
        float _radian;

    public:
        Radian() = default;
        Radian(float radian) : _radian(radian) {}
        operator float() const;
        explicit operator Degree() const;
        Degree toDegree() const;
        Radian operator+(const Radian &radian) const;
        Radian operator+() const;
        Radian operator-(const Radian &radian) const;
        Radian operator-() const;
        Radian operator*(float value) const;
        Radian operator/(float value) const;
        Radian &operator+=(const Radian &radian);
        Radian &operator-=(const Radian &radian);
        Radian &operator+=(const Degree &degree);
        Radian &operator-=(const Degree &degree);
        Radian &operator*=(float value);
        Radian &operator/=(float value);
        ~Radian() {}
    };
    std::ostream &operator<<(std::ostream &os, const Radian &radian);
    Radian operator*(float value, const Radian &radian);
    Radian operator/(float value, const Radian &radian);
#pragma endregion
} // namespace ns_angle
