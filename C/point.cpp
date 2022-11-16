#include <iostream>
#include "point.h"

using namespace std;

point::point(int a, int b)
{
    x = a;
    y = b;
}

point::~point()
{
}

point point::operator+(point second)
{
    point temp;
    temp.x = x + second.x;
    temp.y = y + second.y;
    return temp;
}
point point::operator-(point second)
{
    point temp;
    temp.x = x - second.x;
    temp.y = y - second.y;
    return temp;
}
bool point::operator==(point second)
{
    return (second.x == x && second.y == y);
}
int point::distance(point dst)
{
    return (abs(x - dst.x) + abs(y - dst.y));
}
bool point::isValidPoint(int Env_dim)
{
    if (x >= 0 && x < Env_dim && y >= 0 && y < Env_dim)
        return true;
    return false;
}