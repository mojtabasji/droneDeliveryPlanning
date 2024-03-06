from typing import List
import keyboard
import time
import sys
import os
import json
import copy
import optparse
import random
from DataStore import StoreData
from StoreMoreData import StoreMore
from storeFaillure import info_storage
import cv2
from sumolib import checkBinary
import traci
from PIL import Image, ImageDraw, ImageFont
from point import point
from Elements import UAV, BusStop, Bus, Depot, Line_class
from uav_tjp import uav_tjp
import routeFinding as Rf
import routeController as Rc
import findNearStops as Fns
from utils import *
from constants import *
import yaml


class SubRequest:
    def __init__(self, length, depot_count):
        self.length = length
        self.depot_count = depot_count
        self.last_request_index = 0
        self.sub_requests = []
        self.depots_reachability = {i: [True for _ in range(length)] for i in range(depot_count)}
        self.serve_cost = {i: [] for i in range(depot_count)}

    def refill(self, main_request_list):
        if self.last_request_index < len(main_request_list):
            shortage = self.length - len(self.sub_requests)
            if shortage > len(main_request_list) - self.last_request_index:
                shortage = len(main_request_list) - self.last_request_index
            new_last = self.last_request_index + shortage
            shorter_counter = 0
            tempo_counter = self.last_request_index
            tempo_list = []
            while shorter_counter < shortage:
                if tempo_counter >= len(main_request_list):
                    break
                if main_request_list[tempo_counter][1] != CustomerSate.not_reachable:
                    tempo_list.append(tempo_counter)
                    shorter_counter += 1
                tempo_counter += 1
            self.sub_requests.extend(tempo_list)
            self.last_request_index = tempo_counter
            for d in self.serve_cost:
                self.serve_cost[d].extend([None for _ in range(shorter_counter)])
            #     self.depots_reachability[d].extend([True for _ in range(shortage)])

    def remove_unreachable(self, main_request_list):
        remove_list = []
        for ind in range(len(self.sub_requests)):
            if all([not self.depots_reachability[d][ind] for d in self.depots_reachability]):
                remove_list.append(ind)
        remove_list.reverse()
        for ind in remove_list:
            main_request_list[self.sub_requests[ind]][1] = CustomerSate.not_reachable
            self.sub_requests.pop(ind)
            for d in self.depots_reachability:
                self.depots_reachability[d].pop(ind)
                self.serve_cost[d].pop(ind)

    def insert_depot_cost(self, depot_id, indexes, costs):
        cost_list_index_counter = 0
        for ind in indexes:
            self.serve_cost[depot_id][ind] = costs[cost_list_index_counter]
            # if costs[cost_list_index_counter] > MaxFlyDist * OUT_OF_REACH_COSTUMERS:
            #     self.depots_reachability[depot_id][ind] = False
            cost_list_index_counter += 1

    def get_none_cost_indexes(self, depot_id):
        indexes = []
        for ind in range(len(self.serve_cost[depot_id])):
            if self.serve_cost[depot_id][ind] is None:
                indexes.append(ind)
        return indexes

    def get_request_loc(self, sub_req_indexes):
        return [requests[self.sub_requests[i]][0] for i in sub_req_indexes]

    def get_min_cost_request(self, depot_id):
        min_cost = None
        min_cost_index = None
        if CHOOSE_CUSTOMER_RANDOMLY:
            if len(self.serve_cost[depot_id]) > 0:
                min_cost_index = random.randint(0, len(self.serve_cost[depot_id]) - 1)
        else:
            for ind in range(len(self.serve_cost[depot_id])):
                if self.serve_cost[depot_id][ind] is not None:
                    if (min_cost is None or min_cost > self.serve_cost[depot_id][ind]):
                        # and self.depots_reachability[depot_id][ind]
                        min_cost = self.serve_cost[depot_id][ind]
                        min_cost_index = ind
        return min_cost_index

    def pop_to_server(self, sub_request_index, main_request_list):
        resp = main_request_list[self.sub_requests[sub_request_index]][0]
        main_request_list[self.sub_requests[sub_request_index]][1] = CustomerSate.served
        self.sub_requests.pop(sub_request_index)
        for d in self.serve_cost:
            self.serve_cost[d].pop(sub_request_index)
            # self.depots_reachability[d].pop(sub_request_index)
        return resp


with open('conf.yaml', 'r') as file:
    yaml_data = yaml.load(file, Loader=yaml.FullLoader)

config = json.load(open('config.json'))
# Options { 'greedyDecide' , 'fairnessDecide', 'deepDecide', 'algorithm', 'TimingDecide}
approach = 'deepDecide' if yaml_data['DECISION_APPROACH'] == 'DEEP' else 'greedyDecide'
requestConf_id = yaml_data['COSTUMER_DEST']

loadModel = False
showImage = False
createGif = yaml_data['CREATE_GIF']
UAVs: List[UAV] = []
# reachTimeStoreXl = []
reach2finish = yaml_data['COSTUMER_COUNT'] * 2
workingTime = yaml_data['TIME_DURATION']
finisher = None
finish_time = None
BussList = {}
BussLastStations = {}
Lines = {}
ReqSubSet = []
ReqSubSet_len = 50
Depots = []
lines_json = None

back2depotCount = 0
flyFailureCount = 0
costumer_id = 0
BUSS_DISTANCE_DELAY = 450
AVAILABLE_STOP_COEFFICIENT = 0.3
OUT_OF_REACH_COSTUMERS = yaml_data['OUT_OF_REACH_COSTUMERS']
SOURCE_FLY_COEFFICIENT = yaml_data['SOURCE_FLY_COEFFICIENT']
CHOOSE_CUSTOMER_RANDOMLY = yaml_data['CHOOSE_CUSTOMER_RANDOMLY']

BUS_MAX_CAPACITY: int
MAX_WAITING_TIME: float
MaxFlyDist: int
uav_costumer: any

storeFailure = info_storage()
rc: Rc.brain
requests = []
requests_subset: SubRequest = None
BusStopStatus = None
images = []
COLORS: List[tuple] = [(204, 10, 0), (204, 102, 0), (204, 204, 0), (102, 204, 0),
                       (0, 204, 0), (0, 204, 102), (0, 204, 204), (0, 102, 204),
                       (0, 0, 204), (102, 0, 204), (204, 0, 204), (204, 0, 102),
                       (96, 96, 96)]  # 13
uav_wait_on_go_path = None

Actions = [point(0, 0), point(0, 1), point(0, -1), point(1, 0), point(-1, 0)]

'''
UAVPath = [[{"actionType": 'fly', "loc": point(1, 9)}, {"actionType": 'land', "loc": point(20, 40)}],
           [{"actionType": 'fly', "loc": point(9, 1)}, {
               "actionType": 'land', "loc": point(10, 10)}],
           [{"actionType": 'fly', "loc": point(0, 0)}]] '''


# local function's region  ( this functions may call in each other)

def drup_unreachable_customers(main_request_list):
    print("drup_unreachable_customers ...")
    depot_stops = []
    depot_fixed_stop_states = []
    counter = 0
    for depot in range(len(Depots)):
        stoplist = Fns.nearStops(Depots[depot].loc, int(
            MaxFlyDist * AVAILABLE_STOP_COEFFICIENT))
        stoplist = Fns.reviewNearStops(
            Depots[depot].loc, stoplist, lines_json, BusStopStatus)
        depot_stops.append(stoplist)
        depot_fixed_stop_states.append(stop_states(stoplist))
    for req in main_request_list:
        depot_reachable = [True for _ in range(len(Depots))]
        for depot in range(len(Depots)):
            network_status = {'curLoc': Depots[depot].loc, 'destLoc': point(*req[0]),
                              'BStop': depot_fixed_stop_states[depot],
                              'BussList': BussList, 'BusMaxCap': BUS_MAX_CAPACITY, 'MAX_FLY_DIST': MaxFlyDist,
                              'UAV_battery': MaxFlyDist}
            _, temp_cost = rc.cost_greedy(depot_stops[depot],
                                          network_status, lines=Lines, ignore_wait=True)
            if temp_cost > MaxFlyDist * OUT_OF_REACH_COSTUMERS:
                depot_reachable[depot] = False
        if not any(depot_reachable):
            counter += 1
            print(counter, ": ", req[0], " is not reachable")
            req[1] = CustomerSate.not_reachable
    print("drup_unreachable_customers done")


def depot_customer_cost(depot, sub_request_indexes):
    global finisher
    global BussList
    global BUS_MAX_CAPACITY
    global lines_json
    global Lines
    global BusStopStatus
    global requests
    global rc
    global MaxFlyDist
    global requests_subset

    customers_loc = requests_subset.get_request_loc(sub_request_indexes)

    total_cost_list = []
    fly_cost_list = []

    start_point = Depots[depot].loc
    stoplist = Fns.nearStops(start_point, int(
        MaxFlyDist * AVAILABLE_STOP_COEFFICIENT))
    stoplist = Fns.reviewNearStops(
        start_point, stoplist, lines_json, BusStopStatus)
    hiper_power = 0
    while not len(stoplist):
        stoplist = Fns.nearStops(start_point, MaxFlyDist + hiper_power)
        stoplist = Fns.reviewNearStops(
            start_point, stoplist, lines_json, BusStopStatus)
        print("Try To find BS to Come Back --> ", MaxFlyDist + hiper_power)
        hiper_power += 40
    for reqIndex in range(len(customers_loc)):
        # change here if input requests location range Or point range
        end_point = point(*customers_loc[reqIndex])

        network_status = {'curLoc': start_point, 'destLoc': end_point, 'BStop': stop_states(stoplist),
                          'BussList': BussList, 'BusMaxCap': BUS_MAX_CAPACITY, 'MAX_FLY_DIST': MaxFlyDist,
                          'UAV_battery': MaxFlyDist}
        if approach == 'greedyDecide':
            tmp_cost_val, fly_cost = rc.cost_greedy(
                stoplist, network_status, lines=Lines, ignore_wait=True)
        elif approach == 'deepDecide':
            tmp_cost_val, fly_cost = rc.Cost_deep(stoplist, network_status, Lines)
            # tmp_cost_val, fly_cost = rc.cost_greedy(
            #     stoplist, network_status, lines=Lines, ignore_wait=False)
        else:
            tmp_cost_val = 0
            fly_cost = 0

        if fly_cost * 2 > MaxFlyDist:
            fly_cost = 6000

        total_cost_list.append(tmp_cost_val)
        fly_cost_list.append(fly_cost)

    return fly_cost_list


def choice_task_from_subset(UAV_id, flied):
    global BussList
    global BUS_MAX_CAPACITY
    global lines_json
    global Lines
    global BusStopStatus
    global MaxFlyDist
    global requests_subset
    global requests

    finalRes = {}
    fly_count_c2d = 0

    start_point = UAVs[UAV_id].loc
    stoplist = Fns.nearStops(start_point, int(
        MaxFlyDist * AVAILABLE_STOP_COEFFICIENT))
    stoplist = Fns.reviewNearStops(
        start_point, stoplist, lines_json, BusStopStatus)
    hiper_power = 0
    while not len(stoplist):
        stoplist = Fns.nearStops(start_point, MaxFlyDist + hiper_power)
        stoplist = Fns.reviewNearStops(
            start_point, stoplist, lines_json, BusStopStatus)
        print("Try To find BS to Come Back --> ", MaxFlyDist + hiper_power)
        hiper_power += 40
    network_status = {'curLoc': start_point, 'BStop': stop_states(stoplist),
                      'BussList': BussList, 'BusMaxCap': BUS_MAX_CAPACITY, 'MAX_FLY_DIST': MaxFlyDist,
                      'UAV_battery': MaxFlyDist - flied}

    for dep in range(len(Depots)):
        end_point = Depots[dep].loc

        # 'destLoc': end_point ,
        network_status['destLoc'] = end_point

        if start_point == end_point:
            tmp_cost_val = 0
            fly_count_c2d = 0
        else:
            if approach == 'greedyDecide':
                tmp_cost_val, fly_count_c2d = rc.cost_greedy(
                    stoplist, network_status, lines=Lines)
            elif approach == 'deepDecide':
                tmp_cost_val, fly_count_c2d = rc.Cost_deep(
                    stoplist, network_status, Lines)

        # if fly_count_c2d + flied > MaxFlyDist:
        #     fly_count_c2d = 6000

        finalRes[dep] = fly_count_c2d

        # MytempCostTable.insert(dep, *depot_customer_cost(dep, ReqSubSet_len))

    # MytempCostTable.drop_outers()

    minCosts = []
    failed = False
    lowest_cost_depot_id = None
    lowest_cost_depot_value = None
    for d in range(len(Depots)):
        if lowest_cost_depot_value is None or lowest_cost_depot_value > finalRes[d]:
            lowest_cost_depot_value = finalRes[d]
            lowest_cost_depot_id = d
        # if all costumer removed it's mean for all depot removed
        # if MytempCostTable.get_min_value(d) == -1:
        #     failed = True
        #     break
        # minCosts.append(finalRes[d] + MytempCostTable.get_min_value(d))
    if failed:
        return [[UAVs[UAV_id].loc.x, UAVs[UAV_id].loc.y], [UAVs[UAV_id].loc.x, UAVs[UAV_id].loc.y]], 1

    # best_depot_id = np.argmin(minCosts)   # temporary comment for ensure that UAVs can reach to depot
    best_depot_id = lowest_cost_depot_id
    requests_subset.refill(requests)
    empty_customer_depot_indexes = requests_subset.get_none_cost_indexes(best_depot_id)
    requests_subset.insert_depot_cost(best_depot_id, empty_customer_depot_indexes,
                                      depot_customer_cost(best_depot_id, empty_customer_depot_indexes))
    # requests_subset.remove_unreachable(requests)

    # if finalRes[best_depot_id] > 5500:     # uav locked in costumer location and it seems that can't reach any depot
    #     return [[UAVs[UAV_id].loc.x, UAVs[UAV_id].loc.y], [UAVs[UAV_id].loc.x, UAVs[UAV_id].loc.y]], 1

    best_request_id = requests_subset.get_min_cost_request(best_depot_id)

    change_depot = False
    depots_id_list = [i for i in range(len(Depots))]
    depots_id_list.remove(best_depot_id)
    chosen_depot = best_depot_id
    while best_request_id is None:
        change_depot = True
        if len(depots_id_list) == 0:
            # reachable customer's done. turn off uav in depot
            print("reachable customer's done. turn off uav in depot")
            return [[Depots[best_depot_id].loc.x, Depots[best_depot_id].loc.y], ["turn", "off"]], best_depot_id
        chosen_depot = np.random.choice(depots_id_list)
        depots_id_list.remove(chosen_depot)
        requests_subset.refill(requests)
        empty_customer_depot_indexes = requests_subset.get_none_cost_indexes(chosen_depot)
        requests_subset.insert_depot_cost(chosen_depot, empty_customer_depot_indexes,
                                          depot_customer_cost(chosen_depot, empty_customer_depot_indexes))
        # requests_subset.remove_unreachable(requests)
        best_request_id = requests_subset.get_min_cost_request(chosen_depot)

    best_request = requests_subset.pop_to_server(best_request_id, requests)

    if change_depot:
        result = [[Depots[best_depot_id].loc.x,
                   Depots[best_depot_id].loc.y],
                  [Depots[chosen_depot].loc.x,
                   Depots[chosen_depot].loc.y],
                  best_request]
    else:
        result = [[Depots[best_depot_id].loc.x,
                   Depots[best_depot_id].loc.y], best_request]

    return result, Depots[chosen_depot].id


def task_manager():
    global finisher
    global costTbl
    global BussList
    global BUS_MAX_CAPACITY
    global Lines
    global lines_json
    global BusStopStatus
    global costumer_id
    global MAX_WAITING_TIME
    global uav_costumer
    global MaxFlyDist
    global request_id

    for counter in range(UAVCount):
        if not UAVs[counter].is_working:
            continue
        if UAVs[counter].delay > 1:
            UAVs[counter].delay -= 1
            # destination[counter] = {"actionType": 'delay', 'loc': 'there'}
            continue
        if UAVs[counter].delay == 1:
            UAVs[counter].delay = 0
        if not UAVs[counter].status:  # be 0 mean subtask done      # reached to depot or costumer
            print(counter, ": --- %s seconds ---" % (time.time() - start_time))
            Uav_request[counter] = request_id
            request_id += 1

            debug_list: List[int] = []
            uav_debug_list: List[int] = [31]
            if costumer_id in debug_list or counter in uav_debug_list:
                lets_debug = True

            # if UAV is in costumer location and need cdC' task
            if len(UAVTasks[counter]) == 0 or UAVTasks[counter][0] is None:
                UAVs[counter].start_from = "costumer"
                if finisher is not None and finisher:
                    return

                UAVTasks[counter], chosen_depot = choice_task_from_subset(counter, UAVs[counter].flied)
                storeMore.increaseDepotUsed(str(chosen_depot))
                # if UAV in depot. in initial state
                if point(*UAVTasks[counter][0]) == UAVs[counter].loc:
                    UAVs[counter].flied = 0
                    UAVs[counter].start_from = "depot"
                    UAVTasks[counter].pop(0)
                    if UAVTasks[counter][0] == ["turn", "off"]:
                        UAVs[counter].is_working = False
                        continue
                    if len(UAVTasks[counter]) >= 2:  # change depot if costumer is out of reach from depot
                        storeData.setPathType(counter, 3)  # depot2depot
                        uav_costumer[counter] = costumer_id
                        storeData.setCostumer_id(counter, costumer_id)
                        UAVs[counter].change_depot = True
                    else:
                        UAVTasks[counter].append(None)
                        storeData.setPathType(counter, 0)  # to costumer
                        uav_costumer[counter] = costumer_id
                        storeData.setCostumer_id(counter, costumer_id)
                    costumer_id += 1
                else:
                    # point(*UAVTasks[counter][0]) in [ de.loc for de in Depots]:
                    if Uav_request[counter] < UAVCount:
                        storeData.setPathType(counter, 3)
                        UAVs[counter].start_from = "depot"
                        UAVs[counter].flied = 0
                    else:
                        storeData.setPathType(counter, 1)
                    if len(UAVTasks[counter]) >= 3:  # change depot if costumer is out of reach from depot
                        storeData.setPathType(counter, 1)  # depot2depot
                        UAVs[counter].start_from = "costumer"
                        uav_costumer[counter] = costumer_id

                storeData.setDepot_id(counter, chosen_depot)

                '''T_task = requests.pop(0)
                UAVs[counter].loc = point(*T_task.pop(0))
                UAVTasks[counter] = T_task'''

            else:  # uav is in depot and recharge battery
                if UAVTasks[counter][0] == ["turn", "off"]:
                    UAVs[counter].is_working = False
                    continue

                UAVs[counter].flied = 0
                UAVs[counter].start_from = "depot"
                storeData.setCostumer_id(counter, costumer_id)
                uav_costumer[counter] = costumer_id
                if len(UAVTasks[counter]) == 1:  # uav is in depot and wants go to customer location
                    storeData.setPathType(counter, 0)
                    costumer_id += 1
                else:  # uav is in depot and wants to change depot
                    storeData.setPathType(counter, 3)

                for dep in Depots:
                    if dep.loc == UAVs[counter].loc:
                        storeData.setDepot_id(counter, dep.id)
                        break

            stoplist = Fns.nearStops(UAVs[counter].loc, int(
                MaxFlyDist * AVAILABLE_STOP_COEFFICIENT))
            stoplist = Fns.reviewNearStops(UAVs[counter].loc, stoplist, lines_json,
                                           BusStopStatus)  # just keep 2 stop in each line

            hiper_power = 0
            while not len(stoplist):
                stoplist = Fns.nearStops(
                    UAVs[counter].loc, MaxFlyDist + hiper_power)
                stoplist = Fns.reviewNearStops(
                    UAVs[counter].loc, stoplist, lines_json, BusStopStatus)
                print("Try To find BS to Come Back --> ",
                      MaxFlyDist + hiper_power)
                hiper_power += 40

            uav_battery = int(MaxFlyDist / 2) if UAVs[counter].start_from == "depot" else int(
                MaxFlyDist - UAVs[counter].flied)
            network_status = {'curLoc': UAVs[counter].loc, 'destLoc': point(
                *UAVTasks[counter][0]), 'BStop': stop_states(stoplist), 'BussList': BussList,
                              'BusMaxCap': BUS_MAX_CAPACITY, 'UAV_battery': uav_battery,
                              'MAX_FLY_DIST': MaxFlyDist}
            # lineBusyRateUpdate()
            er = 1
            if approach == 'greedyDecide':
                start_station, wait_time = rc.greedy(stoplist, network_status, lines=Lines)
                mem_id = 1
            elif approach == 'deepDecide':
                mem_id, start_station, er, wait_time = rc.decide(stoplist, network_status, Lines)
            selected_line = Rf.findStopLine(int(start_station))
            storeData.setRouteLine(counter, Rf.findStopLine(int(start_station)))
            storeMore.storedecideParams(er, len(stoplist), selected_line)
            destini = point(*UAVTasks[counter].pop(0))
            nothing, route = Rf.find(int(start_station), destini)
            Costs = Rf.Costing(UAVs[counter].loc, route, destini)
            UAVs[counter].dest_fly_estimate = Costs['destfly']
            route = list(map(str, route))
            UAVPath[counter], start_station_with_direction = route2path(  # for change line this function need to fix
                route, destini)
            UAVHistory[counter] = mem_id
            Lines[selected_line].stations[start_station_with_direction].coming.append(
                counter)

            coming2stations = len(
                Lines[selected_line].stations[start_station_with_direction].coming) - 1
            waiting2stations = len(
                Lines[selected_line].stations[start_station_with_direction].passengers)
            bus_count2reach = int(
                (coming2stations + waiting2stations) / BUS_MAX_CAPACITY)
            # drop delay for busy stations ((bus_count2reach + 1) * BUSS_DISTANCE_DELAY)
            if UAVs[counter].start_from == "depot" and wait_time - UAVs[counter].loc.distance(
                    UAVPath[counter][0]['loc']) > MAX_WAITING_TIME:
                if wait_time > 140000:
                    wait_time = bus_count2reach * BUSS_DISTANCE_DELAY
                UAVs[counter].delay = wait_time  # bus_count2reach * BUSS_DISTANCE_DELAY
                UAVs[counter].wait_in_depot = wait_time


def lineBusyRateUpdate():
    for Lin in Lines:
        passengerCount_0 = 1
        passengerCount_1 = 1
        for cntr in Lines[Lin]['stops']:
            passengerCount_0 += len(BusStopStatus[str(cntr) + '_0'].passengers)
            passengerCount_1 += len(BusStopStatus[str(cntr) + '_1'].passengers)

        Lines[Lin]['busyRate_0'] = 1 - (1 / passengerCount_0)
        Lines[Lin]['busyRate_1'] = 1 - (1 / passengerCount_1)


# todo: to change Line, the part < dispatch a bus and fly to new station > should be add.
def route2path(routeOrg, dest):
    route = copy.deepcopy(routeOrg)
    for i in range(0, len(route), 2):
        if int(route[i]) < int(route[i + 1]):
            route[i] += '_0'
            route[i + 1] += '_0'
        else:
            route[i] += '_1'
            route[i + 1] += '_1'

    pth = []
    pth.append({"actionType": 'fly', "loc": point(
        *loc2point(BusStopsLoc[route[0]]))})
    start_station_with_driection = route[0]
    while (len(route) > 0):
        pth.append({"actionType": 'land', 'loc': route.pop(0)})
        pth.append({"actionType": 'drive', 'loc': route.pop(0)})
        if len(route) > 0:
            pth.append({"actionType": 'changeLine', 'loc': route[0]})
    pth.append({"actionType": 'rise', 'loc': dest})
    pth.append({'actionType': 'fly', 'loc': dest})
    pth.append({'actionType': 'finish'})
    return pth, start_station_with_driection


def stop_states(Slist):
    Slst = copy.deepcopy(Slist)
    Stt = {}
    for stp in Slst:
        # get po #get p #get loc
        T_s = {}
        T_s['passengers'] = len(BusStopStatus[str(stp) + '_0'].passengers)
        T_s['p'] = BusStopStatus[str(stp) + '_0'].success_rate
        T_s['loc'] = point(*loc2point(BusStopsLoc[stp + '_0']))
        temp_beforeStops, T_s['freeSpace'], T_s['comingBuss'] = __bus_stop_success_rate(str(stp) + '_0')
        T_s['beforeStops'] = temp_beforeStops
        T_s['coming'] = __flier_count2bs(str(stp) + '_0')
        sumBeforFlierCount = 0
        for cunt in temp_beforeStops:
            sumBeforFlierCount += __flier_count2bs(cunt)
        T_s['goingToBefore'] = sumBeforFlierCount
        Stt[stp + '_0'] = T_s

        T_s = {}
        T_s['passengers'] = len(BusStopStatus[str(stp) + '_1'].passengers)
        T_s['p'] = BusStopStatus[str(stp) + '_1'].success_rate
        T_s['loc'] = point(*loc2point(BusStopsLoc[stp + '_1']))
        temp_beforeStops, T_s['freeSpace'], T_s['comingBuss'] = __bus_stop_success_rate(str(stp) + '_1')
        T_s['beforeStops'] = temp_beforeStops
        T_s['coming'] = __flier_count2bs(str(stp) + '_1')
        sumBeforFlierCount = 0
        for cunt in temp_beforeStops:
            sumBeforFlierCount += __flier_count2bs(cunt)
        T_s['goingToBefore'] = sumBeforFlierCount
        Stt[stp + '_1'] = T_s

    return Stt


def __flier_count2bs(chosen_bs: str) -> int:  # args Like => ('79_0') --> 2
    cnt = 0
    goal_bs = {"actionType": 'land', 'loc': chosen_bs}
    for tmp_pth in UAVPath:
        if goal_bs in tmp_pth:
            cnt += 1
    return cnt


# args Like => ('79_1') ---> ['78_1','78_0','79_0'] ,4  count of from number stop before bus reach with which capacity
def __bus_stop_success_rate(busStp):
    beforeStops = Rf.beforeStops(busStp)  # ,['78_1','78_0','79_0']
    comingBuss = []
    temporal_passengers = []

    for bus in BussList:
        if BussList[bus].lastStop in beforeStops:
            comingBuss.append(bus)  # [23, 12, 32 ,..] Buss_ID

    for bs_cnt in beforeStops:
        temporal_passengers.append(len(BusStopStatus[bs_cnt].passengers))

    freeSpace = 0  # number of free space reach to goal station from N station before
    for combus in comingBuss:
        B_freeSpace = BUS_MAX_CAPACITY
        UavsOnThisBus = [i for i, x in enumerate(UonBusID) if x == combus]
        for tmp_uav in UavsOnThisBus:
            if destination[tmp_uav]["actionType"] == "drive" and destination[tmp_uav]["loc"] not in beforeStops:
                B_freeSpace -= 1
        for tmp_BS in range(beforeStops.index(BussList[combus].lastStop) + 1, len(beforeStops)):
            while (B_freeSpace > 0):
                if temporal_passengers[tmp_BS] > 0:
                    temporal_passengers[tmp_BS] -= 1
                    B_freeSpace -= 1
                else:
                    break

        freeSpace += B_freeSpace

    return beforeStops, freeSpace, comingBuss


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                          default=False, help="run the commandLine version of sumo")
    options_, args = opt_parser.parse_args()
    return options_


def convert(iAction):
    res = []
    for k in range(UAVCount):
        tmp = int(iAction / pow(len(Actions), k))
        tmp = int(tmp % len(Actions))
        res.append(Actions[tmp])
    return res


def convertActionFromString(iAct, cnt):
    res = []
    iAct = str(iAct)
    iAct = iAct.split(',')
    for k in range(cnt):
        res.append(Actions[int(iAct[k])])
    return res


def loadConfig():
    global UAVCount
    global Depots
    global group_uperbound
    global Env_dim
    global UonBusID
    global BusStopsLoc
    global UAVTasks
    global BusStopStatus
    global BUS_MAX_CAPACITY
    global requests
    global UAVHistory
    global images
    global Lines
    global MaxFlyDist
    global MAX_WAITING_TIME
    global Uav_request
    global request_id
    global requests_subset
    global destination
    global storeData
    global storeMore
    global lines_json
    global uav_costumer
    global uav_wait_on_go_path

    Env_dim = int(traci.simulation.getNetBoundary()[1][0] / 10)
    # UAVCount = config['UAVCount']
    UAVCount = yaml_data['UAVS_COUNT']
    MaxFlyDist = yaml_data['UAV_MAX_FLY_DIST']  # config['MaxFlyDist']
    MAX_WAITING_TIME = MaxFlyDist * SOURCE_FLY_COEFFICIENT
    # BUS_MAX_CAPACITY = config['BusMaxCap']
    BUS_MAX_CAPACITY = yaml_data['BUS_CAPACITY']
    group_uperbound = config["group_uperbound"]
    BusStopsLoc = config["BusStopsLoc"]
    tmpDepots = config["3_Depots"] if yaml_data['DEPOT_DEST'] == 1 else config["4_Depots"]
    Depots = [Depot(iD, point(tmpDepots[iD]['x'], tmpDepots[iD]['y']))
              for iD in tmpDepots]
    BusStopStatus = {iD: BusStop(
        iD, point(*loc2point(BusStopsLoc[iD]))) for iD in BusStopsLoc}
    UAVTasks = [[] for i in range(UAVCount)]
    uav_costumer = [None for i in range(UAVCount)]
    UAVHistory = np.zeros(UAVCount)
    f = open('requestConf' + str(requestConf_id) + '.json', 'r')
    requests = json.load(f)
    f.close()
    # split some first items from requests
    requests = [[requests[i], CustomerSate.not_served] for i in range(yaml_data['COSTUMER_COUNT'])]
    requests_subset = SubRequest(ReqSubSet_len, len(Depots))
    lineFile = open('lineConfig.json')
    lines_json = json.load(lineFile)
    lineFile.close()
    for line in lines_json:
        new_line_details = lines_json[line]
        new_line = Line_class(new_line_details["id"])
        for station in new_line_details["stops"]:
            new_line.addBusStation(
                str(station) + "_0", point(*loc2point(BusStopsLoc[str(station) + "_0"])))
            new_line.addBusStation(
                str(station) + "_1", point(*loc2point(BusStopsLoc[str(station) + "_1"])))
        Lines[line] = new_line

    setUavs()
    destination = [None for _ in range(UAVCount)]
    UonBusID = [-1] * UAVCount
    Uav_request = [None for _ in range(UAVCount)]
    uav_wait_on_go_path = [0 for _ in range(UAVCount)]
    request_id = 0
    # images = [[] for i in range(UAVCount)]
    storeData = StoreData(UAVCount, reach2finish, len(lines_json))
    storeMore = StoreMore()


def map_station2line(station_id):  # station_id = 79
    global lines_json
    # if station_id is string
    if type(station_id) == str:
        station_id = int(station_id.split("_")[0])
    for line in lines_json:
        if station_id in lines_json[line]["stops"]:
            return line
    return None


def setUavs():
    UAV_conf = config['3_UAVs'] if yaml_data['DEPOT_DEST'] == 1 else config['4_UAVs']
    for i in range(1, UAVCount + 1):
        UAVs.append(
            UAV(Depots[UAV_conf[f"{i}"]['depot'] - 1].loc, i))
        # UAV(point(UAV_conf[f"{i}"]['x'], UAV_conf[f"{i}"]['y']), i))


# main function region


def go_forward():
    global UAVPath
    global images
    global showImage
    global finisher
    global Lines
    global MaxFlyDist
    global back2depotCount
    global flyFailureCount
    global MAX_WAITING_TIME
    global uav_costumer
    global uav_wait_on_go_path
    global finish_time

    monitoring = 0
    take_pic = False
    finisher = False
    UAVPath = [[] for i in range(UAVCount)]
    # UAVPath = [[{"actionType": 'fly', "loc": point(*loc2point(BusStopsLoc['85_0']))},
    # {"actionType":'land','loc':'85_0'},
    #           {"actionType":'drive','loc':'95_0'},{"actionType":'rise','loc':point(1493, 852)},
    #           {'actionType':'fly', 'loc': point(1493, 852)},{'actionType':'finish'}], ]
    prev_action = []

    # drop Delay **************
    '''if UAVCount > 40 :
        for i in range(40):
            UAVs[i].delay = 40'''
    # End drop Delay

    for i in range(UAVCount):
        prev_action.append(Actions[0])

    printCounter = 1
    UAVActions = [None] * UAVCount
    LastDistance = [None] * UAVCount

    drup_unreachable_customers(requests)

    i: int = -1
    while True:  # start to go
        i = i + 1
        traci.simulationStep()
        bus_state_update()  # update buss location and last station and location
        task_manager()

        if i < 80:
            print("step: ", i, ": --- %s seconds ---" %
                  (time.time() - start_time))

        if len([u for u in UAVs if u.is_working]) == 0:
            finisher = True  # all UAVs are off

        if finisher or i == workingTime:
            finish_time = i
            break

        for ctr in range(UAVCount):
            if UAVs[ctr].is_working is False:
                continue
            # when uav get new task, it's status is 0, and it's delay is 0 ## or UAVs[ctr].delay
            if not (UAVs[ctr].status):
                destination[ctr] = UAVPath[ctr].pop(0)
                UAVs[ctr].status = 1
                UAVs[ctr].stepet = 0
                UAVs[ctr].wait_step = 0
                # UAVs[ctr].flied = 0   # should not set here for every path, just for first path (set in task_manager)
                UAVs[ctr].flayFail = False

        for tmp in range(UAVCount):
            if UAVs[tmp].delay or UAVs[tmp].is_working is False:
                # storeData.incrementStep(tmp)
                continue
            elif destination[tmp]["actionType"] == "back2depot":
                UAVs[tmp].status = 0
                UAVTasks[tmp] = []
                storeData.resetStepsData(tmp)

            elif destination[tmp]["actionType"] == "finish":
                UAVs[tmp].status = 0
                print(printCounter, ": hey, I'm ", tmp, " in destination. (",
                      UAVs[tmp].loc.x, ", ", UAVs[tmp].loc.y, ") =>", ' with ', UAVs[tmp].stepet, ' whole step and ',
                      UAVs[tmp].flied, " fly step From ", UAVs[tmp].start_from)
                if approach == 'deepDecide':
                    # UAVs[tmp].stepet)
                    rc.saveData(UAVHistory[tmp], UAVs[tmp].wait_step)

                storeData.storeTiming(Uav_request[tmp], tmp, timeSlot=i)
                storeData.increseReachs()
                # sheet.write(Uav_request[tmp], 0, printCounter)
                # sheet.write(Uav_request[tmp], 1, UAVs[tmp].stepet)
                printCounter += 1
                if printCounter == reach2finish:
                    finisher = True
            elif destination[tmp]["actionType"] == "fly":
                storeData.incrementStep(tmp)
                if destination[tmp]["loc"] == UAVs[tmp].loc:
                    storeData.storeTiming(Uav_request[tmp], tmp)
                    destination[tmp] = UAVPath[tmp].pop(0)
                    if destination[tmp]["actionType"] == "land":
                        goal_bs = destination[tmp]['loc']
                        current_line = map_station2line(goal_bs)
                        if tmp not in Lines[current_line].stations[goal_bs].passengers:
                            Lines[current_line].stations[goal_bs].coming.remove(
                                tmp)
                            Lines[current_line].stations[goal_bs].passengers.append(
                                tmp)
                        if UAVs[tmp].start_from == "depot":
                            storeData.setWaitInDepot(Uav_request[tmp], tmp, UAVs[tmp].wait_in_depot)

            elif destination[tmp]["actionType"] == "changeLine":
                current_line = traci.vehicle.getRouteID(UonBusID[tmp])
                Lines[current_line].busList[UonBusID[tmp]].passengers.pop(
                    Lines[current_line].busList[UonBusID[tmp]].passengers.index(tmp))
                UonBusID[tmp] = -1
                UAVs[tmp].loc = tmpVhclPnt
                destination[tmp] = UAVPath[tmp].pop(0)
                new_station = destination[tmp]['loc']
                new_line = map_station2line(new_station)
                Lines[new_line].stations[new_station].passengers.append(tmp)
                print("line changed from " + current_line + " to " + new_line)

            # UAV wait's in stations to bus reach
            elif destination[tmp]["actionType"] == "land":
                if UAVs[tmp].start_from == "depot":
                    uav_wait_on_go_path[tmp] += 1
                else:
                    uav_wait_on_go_path[tmp] = 0

                storeData.incrementStep(tmp)
                goal_bs = destination[tmp]['loc']
                # BusStop witing vehicles
                current_line = map_station2line(goal_bs)

                TmpBusStopVhcl = traci.busstop.getVehicleIDs(goal_bs)
                if len(TmpBusStopVhcl) != 0:  # at last one bus exist in station
                    prob = Lines[current_line].stations[goal_bs].success_rate * \
                           Lines[current_line].stations[goal_bs].calCount
                    Lines[current_line].stations[goal_bs].calCount += 1
                    newProb = (BUS_MAX_CAPACITY - UonBusID.count(TmpBusStopVhcl[0])) / BUS_MAX_CAPACITY
                    prob += newProb
                    Lines[current_line].stations[goal_bs].success_rate = prob / \
                                                                         Lines[current_line].stations[goal_bs].calCount
                    # check free space exist on bus  AND  uav in first free-space of UAVs
                    if UonBusID.count(TmpBusStopVhcl[0]) < BUS_MAX_CAPACITY and \
                            is_in_land_priority(tmp, goal_bs, BUS_MAX_CAPACITY - UonBusID.count(TmpBusStopVhcl[0])):
                        UonBusID[tmp] = TmpBusStopVhcl[0]
                        destination[tmp] = UAVPath[tmp].pop(0)
                        Lines[current_line].stations[goal_bs].passengers.remove(
                            tmp)
                        Lines[current_line].busList[TmpBusStopVhcl[0]
                        ].passengers.append(tmp)
                        storeData.storeTiming(Uav_request[tmp], tmp)

            elif destination[tmp]["actionType"] == "drive":
                storeData.incrementStep(tmp)
                TmpBusStopVhcl = traci.busstop.getVehicleIDs(
                    destination[tmp]['loc'])
                if UonBusID[tmp] in TmpBusStopVhcl:
                    storeData.storeTiming(Uav_request[tmp], tmp)
                    destination[tmp] = UAVPath[tmp].pop(0)
                    TmpVhclPos = traci.vehicle.getPosition(UonBusID[tmp])
                    tmpVhclPnt = point(*loc2point(TmpVhclPos))
                    if destination[tmp]["actionType"] == "rise":
                        storeData.resetStoreOption(tmp, "transport")
                        LastDistance[tmp] = tmpVhclPnt.distance(
                            destination[tmp]['loc'])

            elif destination[tmp]["actionType"] == "rise":
                storeData.incrementStep(tmp)
                TmpVhclPos = traci.vehicle.getPosition(UonBusID[tmp])
                tmpVhclPnt = point(*loc2point(TmpVhclPos))
                TmpVhclDstnc = tmpVhclPnt.distance(destination[tmp]['loc'])
                if TmpVhclDstnc < LastDistance[tmp]:
                    LastDistance[tmp] = TmpVhclDstnc
                else:
                    current_line = traci.vehicle.getRouteID(UonBusID[tmp])
                    Lines[current_line].busList[UonBusID[tmp]].passengers.pop(
                        Lines[current_line].busList[UonBusID[tmp]].passengers.index(tmp))
                    UonBusID[tmp] = -1
                    UAVs[tmp].loc = tmpVhclPnt
                    destination[tmp] = UAVPath[tmp].pop(0)
                    storeData.storeTiming(Uav_request[tmp], tmp)

        for cntr in range(UAVCount):  # update driveing UAVs location
            if UAVs[cntr].is_working is False:
                continue
            if UonBusID[cntr] != -1:
                TmpVhclPos = traci.vehicle.getPosition(UonBusID[cntr])
                tmpVhclPnt = point(*loc2point(TmpVhclPos))
                UAVs[cntr].loc = tmpVhclPnt

        # start create gif ***********

        if keyboard.is_pressed('I'):
            cv2.destroyAllWindows()
            showImage = not showImage
            time.sleep(0.5)

        # if keyboard.is_pressed('P'):
        #     take_pic = True
        #     time.sleep(0.5)

        # '''
        if createGif and monitoring < 1:
            im = Image.open('./images/tehranGray_withLines.jpg')
            draw = ImageDraw.Draw(im)
            width, height = im.size
            font = ImageFont.truetype("arial.ttf", 30)

            for k in Lines:
                for j in Lines[k].busList:
                    point2pixel_x = Lines[k].busList[j].loc.x
                    point2pixel_y = height - Lines[k].busList[j].loc.y
                    # clr = (0, 0, 0)
                    draw.rounded_rectangle(
                        [point2pixel_x - 20, point2pixel_y - 20,
                         point2pixel_x + 20, point2pixel_y + 20],
                        radius=10,
                        outline=(0, 0, 0),
                        width=2,
                    )

            for k in UAVs:
                point2pixel_x = k.loc.x
                point2pixel_y = height - k.loc.y
                # clr = (0, 0, 0)
                # if not k.id == 1:
                #     clr = colors[int(k.id % 13)]
                clr = (0, 255, 255)
                if k.flied > MaxFlyDist:
                    clr = (255, 0, 0)
                draw.rounded_rectangle(
                    [point2pixel_x - 10, point2pixel_y - 10,
                     point2pixel_x + 10, point2pixel_y + 10],
                    radius=10, fill=clr)
                draw.text([point2pixel_x, point2pixel_y],
                          str(k.id - 1), fill=(0, 0, 0), font=font)
            for k in range(UAVCount):
                pnt = None
                if len(UAVPath[k]) > 1:
                    pnt = UAVPath[k][-2]['loc']
                if len(UAVPath[k]) == 1:
                    pnt = destination[k]['loc']
                if pnt is not None:
                    point2pixel_x = pnt.x
                    point2pixel_y = height - pnt.y
                    clr = (25, 25, 255)
                    # draw.text([point2pixel_x, point2pixel_y], "*", fill=clr, font=font)
                    draw.rounded_rectangle(
                        [point2pixel_x - 7, point2pixel_y - 7,
                         point2pixel_x + 7, point2pixel_y + 7],
                        radius=5, fill=clr)

            for k in Depots:
                point2pixel_x = k.loc.x
                point2pixel_y = height - k.loc.y
                draw.polygon([(point2pixel_x, point2pixel_y - 30), (point2pixel_x - 30, point2pixel_y),
                              (point2pixel_x + 30, point2pixel_y)], fill=(245, 70, 60))

            im = im.resize((800, 800))
            im_pil = im
            if showImage:
                im = np.array(im)
                cv2.destroyAllWindows()
                # cv2.namedWindow("tehran", cv2.WINDOW_NORMAL)
                # im = cv2.resize(im, (960, 540))
                cv2.imshow('tehran', im)
                cv2.waitKey(40)
            if take_pic:
                # '+ str(datetime.now())+'
                cv2.imwrite('./images/store/runConf.png', im)
                take_pic = False
            monitoring = 10
            images.append(im_pil)
        monitoring -= 1
        # End create gif ***********'''

        storeData.storeLineCondition(LinesStatus())

        fliers = []

        for tmp in range(UAVCount):
            if UAVs[tmp].is_working is False:
                continue
            UAVs[tmp].stepet += 1
            if destination[tmp]["actionType"] == "fly" and not UAVs[tmp].delay:
                fliers.append(tmp)
                UAVs[tmp].flied += 1

            # add if fly failed
            if UAVs[tmp].flied > MaxFlyDist and UAVs[tmp].flayFail is False and UAVs[tmp].start_from == "costumer":
                UAVs[tmp].flayFail = True
                flyFailureCount += 1
                if destination[tmp]["actionType"] == "fly":
                    if "loc" in UAVPath[tmp][0]:  # fly -> land
                        goal_bs = UAVPath[tmp][0]['loc']
                        current_line = map_station2line(goal_bs)
                        situation = "fly to station"
                    else:
                        goal_bs = "near depot"
                        current_line = "passed"
                        situation = "fly to depot"
                elif destination[tmp]["actionType"] == "land":
                    goal_bs = destination[tmp]["loc"]
                    current_line = map_station2line(goal_bs)
                    situation = "waiting in station"
                else:
                    goal_bs = "there is a problem"
                    current_line = "Error"
                    situation = "somthing else with error"
                storeFailure.add_info(
                    str(flyFailureCount) + " - UAV: " + str(tmp) + " Go path wait: " + str(uav_wait_on_go_path[
                                                                                               tmp]) + "  Flied: " + str(
                        UAVs[tmp].flied) + "  From: " + str(
                        UAVs[tmp].start_from) + "  Step: " + str(UAVs[tmp].stepet) + "  Wait: " + str(
                        UAVs[tmp].wait_step) + "  Line: " + str(current_line) + "  Station: " + str(
                        goal_bs) + " RowInFile: " + str(uav_costumer[tmp]) + " Situation: " + situation)

            if destination[tmp]["actionType"] == "land":
                UAVs[tmp].wait_step += 1
                UAVs[tmp].flied += 1
                # if wait too much return to depot
                if UAVs[tmp].start_from == "depot" and UAVs[tmp].flied > MAX_WAITING_TIME:
                    goal_bs = destination[tmp]['loc']
                    current_line = map_station2line(goal_bs)
                    if tmp in Lines[current_line].stations[goal_bs].passengers:
                        Lines[current_line].stations[goal_bs].passengers.remove(
                            tmp)

                    storeData.storeTiming(Uav_request[tmp], tmp)
                    storeData.resetStoreOption(tmp, "dest_fly")
                    storeData.setPathType(tmp, 2, rowIndex=Uav_request[tmp])
                    destination[tmp]["actionType"] = "fly"
                    min_dist = 3000
                    for dep in Depots:
                        if dep.loc.distance(UAVs[tmp].loc) < min_dist:
                            best_depot = dep
                            min_dist = dep.loc.distance(UAVs[tmp].loc)
                    destination[tmp]["loc"] = best_depot.loc
                    # return UAVs costumer to request list
                    costumer_location_point = UAVPath[tmp][-2]["loc"]
                    if len(UAVTasks[tmp]) > 0 and UAVTasks[tmp][0] is not None:
                        requests.append(
                            [UAVTasks[tmp][0],
                             CustomerSate.not_served])
                        UAVTasks[tmp] = []
                    else:
                        requests.append(
                            [[costumer_location_point.x, costumer_location_point.y],
                             CustomerSate.not_served])
                    # reset UAVs path
                    UAVPath[tmp] = [{'actionType': 'back2depot'}]
                    print("One more back 2 depot, id: ", tmp, " from line: ",
                          current_line, " to depot: ", best_depot.id)
                    back2depotCount += 1

        # print(i,": flier Count: ........    ",len(fliers))
        # to_test = np.array(destination)
        # print("destinies",[point2loc(i["loc"]) for i in to_test[fliers] ])

        fly = uav_tjp(Env_dim, group_uperbound)

        for i_imp in fliers:
            fly.imp_uav(UAVs[i_imp].loc, prev_action[i_imp],
                        destination[i_imp]["loc"])

        actNumber = fly.go_forward()
        UAVActions = convertActionFromString(actNumber, len(fliers))
        # print("Fliers Act: ", [ point_print(flier) for flier in UAVActions])
        # go to next step
        for tmp in range(len(fliers)):
            UAVs[fliers[tmp]].loc = UAVs[fliers[tmp]].loc + UAVActions[tmp]
            prev_action[fliers[tmp]] = UAVActions[tmp]


def is_in_land_priority(uav_id, station_id, free_spaces) -> bool:
    uavs_on_station = Lines[map_station2line(station_id)].stations[station_id].passengers
    times = {}
    for uav in uavs_on_station:
        times[uav] = UAVs[uav].remain_fly()

    sorted_times = []
    for i in times.keys():
        if len(sorted_times) == 0:
            sorted_times.append({i: times[i]})
        else:
            for j in range(len(sorted_times)):
                if list(sorted_times[j].values())[0] > times[i]:
                    sorted_times.insert(j, {i: times[i]})
                    break
                elif j == len(sorted_times) - 1:
                    sorted_times.append({i: times[i]})
                    break

    for i in range(len(sorted_times)):
        if list(sorted_times[i].keys())[0] == uav_id:
            if i < free_spaces:
                return True
            else:
                return False


# --> [flying to 1 , waiting on 1, driving on 1, flying to 2, waiting on 2, ... ]
def LinesStatus():
    global lines_json
    res = []
    for line in lines_json:
        flierCounter = 0
        flierCounterReverse = 0
        BStationPassengers = 0
        BStationPassengersReverse = 0
        Linepassenger = 0
        LinepassengerReverse = 0

        for BStation in lines_json[line]['stops']:
            flierCounter += __flier_count2bs(str(BStation) + '_0')
            flierCounterReverse += __flier_count2bs(str(BStation) + '_1')
            current_line = map_station2line(BStation)
            BStationPassengers += Lines[current_line].stations[str(
                BStation) + '_0'].get_passengers_count()
            BStationPassengersReverse += Lines[current_line].stations[str(
                BStation) + '_1'].get_passengers_count()

        for UaV in UonBusID:
            if UaV != -1 and BussList[UaV].line == line:
                if BussList[UaV].direction == 0:
                    Linepassenger += 1
                else:
                    LinepassengerReverse += 1

        res.extend([flierCounter, flierCounterReverse, BStationPassengers, BStationPassengersReverse,
                    Linepassenger, LinepassengerReverse])

    return res


def bus_state_update():
    for line in Lines:
        for bs in Lines[line].stations:
            vehicles_in_bus_station = traci.busstop.getVehicleIDs(bs)
            for veh in vehicles_in_bus_station:
                Lines[line].busList[veh].setLast(bs)
                Lines[line].busList[veh].direction = int(bs[-1])
        for veh in Lines[line].busList:
            Lines[line].busList[veh].loc = point(
                *loc2point(traci.vehicle.getPosition(veh)))


def __loadSimulationParams():
    global BussLastStations
    T_BusList = traci.vehicle.getIDList()
    for bs in T_BusList:
        # *****# lineConfig.js -> line name must bi Like what is in sumo configuration
        routeID = traci.vehicle.getRouteID(bs)
        BussList[bs] = Bus(bs, routeID, point(
            *loc2point(traci.vehicle.getPosition(bs))))
        BussList[bs].setLast(BussLastStations[bs])
        Lines[routeID].addBus(bs, routeID, point(
            *loc2point(traci.vehicle.getPosition(bs))))
    del BussLastStations


def __loadBrain():
    global rc
    global lines_json
    global Lines
    rc = Rc.brain(UAVCount, LoadModel=loadModel, lines=Lines, exploration_decay=yaml_data['EXPLORE_DECAY'])


def set_last_stations():
    for line in Lines:
        for bs in Lines[line].stations:
            vehicles_in_bus_station = traci.busstop.getVehicleIDs(bs)
            for veh in vehicles_in_bus_station:
                BussLastStations[veh] = bs


class DebugTools:
    def find_uav(self, uav_id):
        global Lines
        for line in Lines:
            res = Lines[line].have_passenger(uav_id)
            if res:
                print("UAV ", uav_id, " is in line ", line, " and ", res)
                return line, res


# Starter
if __name__ == "__main__":
    print("checking parameters ...")
    result_path_extend: str = ""
    if len(sys.argv) > 1:
        result_path_extend = str(sys.argv[1]) + "/"
        # create path if not exist
        if not os.path.exists("result/" + result_path_extend):
            os.makedirs("result/" + result_path_extend)
        open("result/" + result_path_extend + "init.txt", "w").close()
    print("back2depotCount: ", back2depotCount, "flyFailureCount: ", flyFailureCount, "customer to serve: ",
          reach2finish / 2, "len_request: ", len(requests))

    # sumo config
    options = get_options()

    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo')  # 'sumo-gui'

    traci.start([sumoBinary, "-c", "osm.sumocfg",
                 "--tripinfo-output", "tripinfo.xml"])

    loadConfig()  # import config
    for distrubute in range(13000):
        traci.simulation.step()
        if distrubute > 12000:
            set_last_stations()

    debug_tools = DebugTools()
    start_time = time.time()
    __loadSimulationParams()
    __loadBrain()
    print("Deep Q-network Loaded ... ")
    go_forward()
    print("--- %s seconds ---" % (time.time() - start_time))
    print("Back 2 depot count : ", back2depotCount)

    if yaml_data['SAVE_MODEL']:
        rc.save_model_weights()

    unreachable_customer_count = len([Cus for Cus in requests if Cus[1] == CustomerSate.not_reachable])
    moreData2json = {"back2depotCount": back2depotCount,
                     "flyFailureCount": flyFailureCount,
                     "unreachable_customer_count": unreachable_customer_count,
                     "served_customer_count": (int(reach2finish) / 2) - unreachable_customer_count,
                     "end_time": finish_time}
    jsonString = json.dumps(moreData2json)
    jsonFile = open(
        "result/" + result_path_extend + "moreInJson_" + str(UAVCount) + "_" + str(len(Depots)) + "_" + time.strftime(
            '%Y-%m-%d_%H-%M-%S') + ".json", "w")
    jsonFile.write(jsonString)
    jsonFile.close()

    storeFailure.write_info_to_file(
        "result/" + result_path_extend + "flyFailureCount_" + str(UAVCount) + "_" + str(
            len(Depots)) + "_" + time.strftime(
            '%Y-%m-%d_%H-%M-%S') + ".txt")

    '''pic = open('picpath.json','w')
    pic.write(json.dumps(images))'''
    storeMore.setKeyVal('running Time', time.time() - start_time)
    storeData.SaveToFile(str(UAVCount) + "-" + str(len(Depots)), path_extend=result_path_extend)
    storeMore.Save2file(str(UAVCount) + "-" + str(len(Depots)), path_extend=result_path_extend)
    # rc.saveModel(UAVCount)

    if createGif:
        images[0].save('./images/_image_draw.gif',
                       save_all=True, append_images=images[1:], duration=300, loop=0)

    traci.close()
