import numpy as np
from point import point

class UAV:
    def __init__(self, location = point(0,0), iden = None):
        self.loc = location
        self.id = iden
        self.status = 0
        self.flied = 0
        self.stepet = 0
        self.delay = 0


class Bus:
    def __init__(self, ID, Line, location, lasStp=None) -> None:
        self.id = ID
        self.line = Line
        self.direction = 0
        self.loc = location
        self.lastStop = lasStp

    def setLast(self, bs):
        self.lastStop = bs

class BusStop:
    def __init__(self, Name, location) -> None:
        self.passengers = []
        self.successRate = 1
        self.calCount = 1
        self.name = Name
        self.loc = location

class Depot:
    def __init__(self, ID, location = point(0,0)) -> None:
        self.id = ID
        self.loc = location


class CostTable:
    def __init__(self, Req, deps) -> None:
        self.__CostTbl = []
        for R in Req:
            self.__CostTbl.append([None for i in range(len(deps))])

    def SetCost(self, Req_id, dep_id, Cost_val, FlyCostVal):
        self.__CostTbl[Req_id][dep_id] = [Cost_val, FlyCostVal]
    
    def GetCost(self, Req_id, dep_id):
        return self.__CostTbl[Req_id][dep_id][0]
    
    def GetDepotMin(self, dep_id):
        tmpCostList = []
        for r_cost in [self.__CostTbl[req_id][dep_id] for req_id in range(len(self.__CostTbl))]:
            tmpCostList.append(r_cost[0])
        return np.min(tmpCostList), np.argmin(tmpCostList), self.__CostTbl[np.argmin(tmpCostList)][dep_id][1]

    def GetLen(self):
        return len(self.__CostTbl)

    def removeRequest(self, req_id):
        self.__CostTbl.pop(req_id)