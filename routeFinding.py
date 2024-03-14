import json
import yaml

from numpy import linspace

from point import point
from copy import deepcopy

yaml_data = None
with open('conf.yaml', 'r') as file:
    yaml_data = yaml.load(file, Loader=yaml.FullLoader)


cnfg = open('config.json')
f = open('lineConfig.json')

f1 = open('linesRoute.json','r')

LinesRoute = json.load(f1)

maxdist=9000000
lines = json.load(f)
configData = json.load(cnfg)
stopsloc = configData['BusStopsLoc']
# BeforeStopCheckCount = configData['BeforeStopCheckCount']
BeforeStopCheckCount = yaml_data['PREVIOUS_STATIONS_CHECKED_COUNT']


def __loc2point(inp):
    return int(inp[0]/10), int(inp[1] /10)


def find(Sstop , dest): # args Like => ('80', point(123,234))
    for i in lines:
        if Sstop in lines[i]["stops"]:
            cost, rout =  __treeSearch(Sstop, i,dest,[i],[Sstop]) 
            rout = __resetRoute(rout, dest)
            return cost , rout


def __resetRoute(rout, dest):   # args Like => ([int 12, int 45, ... odd count], point(12,12) ) -> [12, 44]
    lastStop = rout[-1]
    secondlastStop = rout[-2]
    if secondlastStop > lastStop:
        alternativeStop = lastStop + 1
    else:
        alternativeStop = lastStop -1
    keepAlterID = alternativeStop
    if secondlastStop == alternativeStop or findStopLine(int(lastStop)) != findStopLine(int(alternativeStop)):
        return rout

    if alternativeStop < lastStop:
        alternativeStop = str(alternativeStop) + '_0'
        lastStop = str(lastStop) + '_0'
    else:
        alternativeStop = str(alternativeStop) + '_1'
        lastStop = str(lastStop) + '_1'

    altStpPNT = point(*__loc2point(stopsloc[alternativeStop]))
    lstStpPNT = point(*__loc2point(stopsloc[lastStop]))
    if altStpPNT.distance(dest) < altStpPNT.distance(lstStpPNT):
        rout[-1] = keepAlterID
    return rout


def __treeSearch(reachStop, Sline, dest, passedLines, route):   # have problem
    gdist = maxdist

    for i in lines[Sline]['stops']:
        dist = dest.distance(point(*__loc2point(stopsloc[str(i)+'_0'])))
        distRev = dest.distance(point(*__loc2point(stopsloc[str(i)+'_1'])))
        if distRev < dist:
            dist = distRev
        if dist < gdist:
            gdist = dist
            troute = [i]

    for i in lines[Sline]['Intersection']:
        if i not in passedLines:
            psdlines= deepcopy(passedLines)
            psdlines.append(i)
            rut = [lines[Sline]['Intersection'][i]['from'], lines[Sline]['Intersection'][i]['to']]
            gdistLineChange, trouteLineChange =  __treeSearch(lines[Sline]['Intersection'][i]['to'], i, dest, psdlines,rut )
            
            if gdistLineChange < gdist:
                gdist = gdistLineChange
                troute = trouteLineChange
            elif gdistLineChange == gdist and len(trouteLineChange) < len(troute):
                gdist = gdistLineChange
                troute = trouteLineChange

    rut = deepcopy(route)
    rut.extend(troute)
    return gdist , rut

    '''if best:
        rut=deepcopy(route)
        rut.extend(troute)
        return gdist , rut
    else:
        return gdist, troute'''


def findStopLine(stp: int) -> str: # args Like => (int 121 ) -> string 'line4'
    for i in lines:
        if stp in lines[i]["stops"]:
            return i
    return str(stp)+"'s Line Not Found"
    

def __stopINline(stp, Lin): # match stop location and line points. args Like => (str '80_1', str 'line4')
    linePoints = LinesRoute[Lin]
    stopPNT = list(__loc2point(stopsloc[stp]))
    if stopPNT in linePoints:
        return linePoints.index(stopPNT)
    else:
        for i in [[0,1],[0, -1], [1, 0], [-1, 0]]:
            stopPNT = list(__loc2point(stopsloc[stp]))
            stopPNT[0] += i[0]
            stopPNT[1] += i[1]
            if stopPNT in linePoints:
                return linePoints.index(stopPNT)
    print("stop ", stp, " is not in Line ", Lin)
    return


def Stop2StopCost(stp1, stp2): #arguments must be string like (str '75',str '80')
    Lin = findStopLine(int(stp1))
    if Lin != findStopLine(int(stp2)):
        print ("stops is not in same Line ...")
        return
    
    if int(stp1) < int(stp2):
        stp1 +='_0'
        stp2 +='_0'
    else:
        stp1 +='_1'
        stp2 +='_1'
    ind1 = __stopINline(stp1, Lin)
    ind2 = __stopINline(stp2, Lin)
    if ind1 != None and ind2 != None:
        return abs(ind1 - ind2)


def risingAndFly_cost(secondLStop ,LStop, dest): # arg Like => (int 80, int 90, point(123,123)) -> 13, 23, [12, 454]
    
    Lin = findStopLine(int(secondLStop))
    linePoints = LinesRoute[Lin]
    if Lin != findStopLine(int(LStop)):
        print ("stops is not in same Line ...")
        return
    
    if secondLStop < LStop:
        secondLStop = str(secondLStop) + '_0'
        LStop = str(LStop) +'_0'
    else:
        secondLStop = str(secondLStop) + '_1'
        LStop = str(LStop) + '_1'
    ind1 = __stopINline(secondLStop, Lin)
    ind2 = __stopINline(LStop, Lin)
    TPoint = point(*linePoints[ind2])
    distan = TPoint.distance(dest)
    if ind1 < ind2 :
        counter = ind2 + 1
        while(1):
            TPoint = point(*linePoints[counter])
            Tdistan = TPoint.distance(dest)
            if Tdistan > distan:
                counter -= 1
                break
            distan = Tdistan
            counter += 1
            if counter == len(linePoints):
                counter = 0
    else:
        counter = ind2 - 1
        while(1):
            TPoint = point(*linePoints[counter])
            Tdistan = TPoint.distance(dest)
            if Tdistan > distan:
                counter += 1
                break
            distan = Tdistan
            counter -= 1
            if counter == -1:
                counter = len(linePoints) -1
    
    TPoint = point(*linePoints[counter])
    risingCost = abs(ind2 - counter)
    flingCost = TPoint.distance(dest)

    return risingCost, flingCost, linePoints[counter]


def Costing(curLoc, route, destLoc):    # args Like => (point(1,1), [12,14,15,18], point(2,2))
    Tp1 = route[0]
    Tp2 = route[1]
    if Tp1 < Tp2:
        Tp1 = str(Tp1) +'_0'
        Tp2 = str(Tp2) + '_0'
        direction = '_0'
    else:
        Tp1 = str(Tp1) + '_1'
        Tp2 = str(Tp2) + '_1'
        direction = '_1'
    Tp1 = point(*__loc2point(stopsloc[Tp1]))
    sourceFlyCst = curLoc.distance(Tp1)
    routeCopy = deepcopy(route)
    tranportCost = 0
    while(len(routeCopy)>0):
        Tp1 = routeCopy.pop(0)
        Tp2 = routeCopy.pop(0)
        tranportCost += Stop2StopCost(str(Tp1),str(Tp2))
    restTransportCost , destFlyCost, disjointLoc = risingAndFly_cost(Tp1, Tp2, destLoc)
    tranportCost += restTransportCost
    return {'sourcefly' : sourceFlyCst, 'transport':tranportCost, 'destfly':destFlyCost, 'disjointLoc': disjointLoc ,'direction':direction }
    

def beforeStops(stp:str) -> list[str]:  # args Like => ('77_0', '76_0', '76_1', '77_1')
    befores = []
    ref_station: int = int(stp.split('_')[0])
    line_name: str = findStopLine(ref_station)
    line_stops: list[int] = lines[line_name]['stops']
    start_station: int = line_stops[0]
    end_station: int = line_stops[-1]
    last_station: int = int(stp.split('_')[0])
    in_go_path: bool = False
    addition: str = '_0'
    inv_addition: str = '_1'
    if int(stp[-1]) == 0:
        in_go_path = True

    for i in range(BeforeStopCheckCount):
        if in_go_path:
            station = last_station - 1
        else:
            station = last_station + 1

        if station < start_station:
            station = start_station
            in_go_path = False
        elif station > end_station:
            station = end_station
            in_go_path = True

        res = addition if in_go_path else inv_addition
        befores.append(str(station) + res)
        last_station = station

    return befores

