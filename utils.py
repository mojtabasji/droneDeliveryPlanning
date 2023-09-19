import numpy as np


def loc2point(inp):
    return int(inp[0] / 10), int(inp[1] / 10)


def point2loc(pnt):
    return [pnt.x, pnt.y]


def point_print(pnt):
    if pnt.x == 1:
        return "right"
    if pnt.x == -1:
        return "left"
    if pnt.y == 1:
        return "up"
    if pnt.y == -1:
        return "down"
    return "stay"





