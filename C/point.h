#pragma once
#include <iostream>


class point
{
private:
    /* data */
public:
    int x, y;
    point(int a=0, int b=0);
    ~point();

    point operator+(point second);
    point operator-(point second);
    bool operator==(point second);
    int distance(point second);
    bool isValidPoint(int Env_dim);

};
