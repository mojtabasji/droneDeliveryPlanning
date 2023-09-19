import numpy as np
from Elements import Depot
from point import point
import findNearStops as fns
import routeController as rc
import threading

approach = "deep"
MaxFlyDist = 999
ReqSubSet_len = 100
requests = []
Depots = []
UAVs = []

class MyCTable:
    def __init__(self) -> None:
        self.FullCost_mem = {}
        self.FlyCost_mem = {}

    def insert(self, d_id, fullCostArray, flyCostArray):
        self.FullCost_mem[d_id] = fullCostArray
        self.FlyCost_mem[d_id] = flyCostArray

    def DropOuters(self):
        ousterListForRemove = []
        for i in range(ReqSubSet_len):
            sumCostsPearDep = 0
            for d in range(len(Depots)):
                sumCostsPearDep += self.FlyCost_mem[d][i]
            if sumCostsPearDep > len(Depots) * 5500:
                #just drop or add to outer list
                ousterListForRemove.append(i)
        for item in ousterListForRemove:
            requests.pop(item)
            for d in range(len(Depots)):
                self.FullCost_mem[d].pop(item)
                self.FlyCost_mem[d].pop(item)
    
    def getMinVal(self, dep):
        return np.min(self.FullCost_mem[dep])

    def getMinInd(self, dep):
        return np.argmin(self.FlyCost_mem[dep])
        

def StopStates():
    pass

def dpCost(depot, checkLen = None):
    global finisher
    global BussList
    global BusMaxCap
    global Lines
    global BusStopStatus
    if checkLen == None or checkLen > len(requests):
        checkLen = len(requests)
    if not len(requests):
        print(" all requests Done.")
        finisher = True
        return [0 for i in range(checkLen)], [0 for i in range(checkLen)]

    totalCostList = []
    flyCostList = []
    
    for reqIndex in range(checkLen):
        startPoint = Depots[depot].loc
        endPoint = point(*requests[reqIndex])
        stoplist = fns.nearStops(startPoint, int(MaxFlyDist * 0.45))
        stoplist = fns.reviewNearStops(startPoint, stoplist, Lines, BusStopStatus)
        hiperPower = 0
        while not len(stoplist):
            stoplist = fns.nearStops(startPoint, MaxFlyDist + hiperPower)
            stoplist = fns.reviewNearStops(startPoint, stoplist, Lines, BusStopStatus)
            print("Try To find BS to Come Back --> ", MaxFlyDist + hiperPower)
            hiperPower += 40


        netState = {'curLoc': startPoint, 'destLoc': endPoint , 'BStop': StopStates(stoplist),
         'BussList': BussList, 'BusMaxCap':BusMaxCap}
        if approach == 'fairnessDecide':
            tmpCostVal, fly_Coust = rc.Cost_fairness(stoplist, netState, Lines)
        elif approach == 'greedlyDecide':
            tmpCostVal, fly_Coust = rc.greedlyDecide(stoplist, netState, Lines)
        elif approach == 'TimingDecide':
            tmpCostVal, fly_Coust = rc.Cost_Timing(stoplist, netState, Lines)
        elif approach == 'deepDecide':
            tmpCostVal, fly_Coust = rc.Cost_deep(stoplist, netState, Lines)
        elif approach == 'algorithm':
            tmpCostVal, fly_Coust = rc.algorithm(stoplist, netState, Lines)

        if fly_Coust * 2 > MaxFlyDist:
            fly_Coust = 6000

        totalCostList.append(tmpCostVal)
        flyCostList.append(fly_Coust)

    return totalCostList, flyCostList

def choiseTaskFromSubset(UAV_id, flied):
    global BussList
    global BusMaxCap
    global Lines
    global BusStopStatus
    MytempCostTable = Table()
    finalRes = {}
    fly_count_c2d = 0
    for dep in range(len(Depots)):
        
        startPoint = UAVs[UAV_id].loc 
        endPoint = Depots[dep].loc
        if startPoint == endPoint:
            tmpCostVal = 0
            fly_count_c2d = 0
        else:
            stoplist = fns.nearStops(startPoint, int(MaxFlyDist * 0.45))
            stoplist = fns.reviewNearStops(startPoint, stoplist, Lines, BusStopStatus)
            hiperPower = 0
            while not len(stoplist):
                stoplist = fns.nearStops(startPoint, MaxFlyDist + hiperPower)
                stoplist = fns.reviewNearStops(startPoint, stoplist, Lines, BusStopStatus)
                print("Try To find BS to Come Back --> ", MaxFlyDist + hiperPower)
                hiperPower += 40
            netState = {'curLoc': startPoint, 'destLoc': endPoint , 'BStop': StopStates(stoplist),
             'BussList': BussList, 'BusMaxCap':BusMaxCap}
            if approach == 'fairnessDecide':
                tmpCostVal, fly_count_c2d = rc.Cost_fairness(stoplist, netState, Lines)
            elif approach == 'greedlyDecide':
                tmpCostVal, fly_count_c2d = rc.greedlyDecide(stoplist, netState, Lines)
            elif approach == 'TimingDecide':
                tmpCostVal, fly_count_c2d = rc.Cost_Timing(stoplist, netState, Lines)
            elif approach == 'deepDecide':
                tmpCostVal, fly_count_c2d = rc.Cost_deep(stoplist, netState, Lines)
            elif approach == 'algorithm':
                tmpCostVal, fly_count_c2d = rc.algorithm(stoplist, netState, Lines)

        if fly_count_c2d + flied > MaxFlyDist:
            fly_count_c2d = 6000

        finalRes[dep] = fly_count_c2d

        MytempCostTable.insert(dep, *dpCost(dep, ReqSubSet_len))
        MytempCostTable.DropOuters()

    minCosts = []
    for d in range(len(Depots)):
        minCosts.append(finalRes[d] + MytempCostTable.getMinVal(d))

    bestDep_id = np.argmin(minCosts)
    bestReq_id = MytempCostTable.get_min_index(bestDep_id)

    result = [[Depots[bestDep_id].loc.x, Depots[bestDep_id].loc.y], requests.pop(bestReq_id)]
        
    return result, Depots[bestDep_id].id