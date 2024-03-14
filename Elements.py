import numpy as np
from point import point


class UAV:
    def __init__(self, location = point(0, 0), iden=None):
        self.is_working = True
        self.change_depot = False
        self.loc = location
        self.id = iden
        self.status = 0
        self.flied = 0
        self.wait_step = 0
        self.stepet = 0
        self.delay = 0
        self.flayFail = False
        self.start_from = "depot"
        self.dest_fly_estimate = 0
        self.wait_in_depot = 0
        self.wait_estimate = 0

    def remain_fly(self):
        return self.dest_fly_estimate - self.flied


class Bus:
    def __init__(self, ID, Line, location, lasStp=None) -> None:
        self.id = ID
        self.passengers = []
        self.line = Line
        self.direction = 0
        self.loc = location     # point
        self.lastStop = lasStp

    def setLast(self, bs):
        self.lastStop = bs
        
    def get_passengers_count(self):
        return len(self.passengers)


class BusStop:
    def __init__(self, Name, location) -> None:
        self.passengers = []
        self.coming = []
        self.success_rate = 1
        self.calCount = 1
        self.name = Name
        self.loc = location
        if "_0" in self.name:
            self.direction = 0
        else:
            self.direction = 1

    def get_passengers_count(self):
        return len(self.passengers)
    
    def get_coming_count(self):
        return len(self.coming)


class Depot:
    def __init__(self, ID, location=point(0, 0)) -> None:
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


class Line_class:
    def __init__(self, id):
        self.id = id
        self.stations = {}
        self.busList = {}

    def addBusStation(self, ID, location):
        self.stations[ID] = BusStop(ID, location)

    def addBus(self, bus_id, bus_line, bus_loc, bus_lastStop=None):
        self.busList[bus_id] = Bus(bus_id, bus_line, bus_loc, bus_lastStop)

    def get_bus_locations(self):
        return [self.busList[bus].loc for bus in self.busList]

    def get_bus_station_count(self):
        return len(self.stations)

    def get_bus_count(self):
        return len(self.busList)

    def create_deep_input(self):
        X_input = []
        X_input.extend([[self.stations[station].get_passengers_count(), self.stations[station].get_coming_count()] for station in self.stations])
        X_input.extend([[self.busList[bus].loc.x, self.busList[bus].loc.y] for bus in self.busList])
        X_input.extend([[self.busList[bus].direction, self.busList[bus].get_passengers_count()] for bus in self.busList])
        return X_input
        
    def have_passenger(self, pass_id):
        for station in self.stations:
            if pass_id in self.stations[station].passengers:
                return f"Station as passenger %{station}"
            if pass_id in self.stations[station].coming:
                return f"Station as in_way %{station}"
        for bus in self.busList:
            if pass_id in self.busList[bus].passengers:
                return f"Bus as passenger %{bus}"
        return False
        