import json
from math import dist
from point import point
import routeFinding as rf
import numpy as np  

stopsLoc = open('config.json')
stopsLoc = json.load(stopsLoc)
stopsLoc = stopsLoc['BusStopsLoc']

def loc2point(inp):
    return (int(inp[0]/10), int(inp[1] /10))

def nearStops(pnt, mxdis):
    nearStopsList = []
    for i in stopsLoc:
        if pnt.distance(point(*loc2point(stopsLoc[i]))) < mxdis:
            nearStopsList.append(i)
    
    nearStopsList = list(map(lambda item: item.replace('_0',''), nearStopsList))
    nearStopsList = list(map(lambda item: item.replace('_1',''), nearStopsList))
    nearStopsList = list(dict.fromkeys(nearStopsList))

    return nearStopsList


def reviewNearStops(curLoc, stopList, lines, StopObj):
    for line in lines:
        lineStopsIndexs = []
        for stop in stopList:
            if rf.findStopLine(int(stop.split('_')[0])) == line:
                lineStopsIndexs.append(stopList.index(stop))

        if len(lineStopsIndexs) > 2:
            cost = []
            for i in lineStopsIndexs:
                passengers = int((len(StopObj[stopList[i] + '_0'].passengers)+ len(StopObj[stopList[i] + '_1'].passengers)) / 2 ) + 1
                dist = curLoc.distance(StopObj[stopList[i] + '_0'].loc)
                cost.append((passengers * dist, stopList[i]))
            cost.sort()
            for i in range(2, len(cost)):
                stopList.remove(cost[i][1])

    return stopList