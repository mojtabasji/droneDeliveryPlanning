from cmath import isnan
import keyboard
import time
import json
import copy
import optparse
from DataStore import StoreData
from StoreMoreData import StoreMore
from storeFaillure import info_storage
import cv2
import numpy as np
from sumolib import checkBinary
import traci
from PIL import Image, ImageDraw, ImageFont
from point import point
from Elements import UAV, BusStop, Bus, Depot, Line_class
from uav_tjp import uav_tjp
import routeFinding as Rf
import routeController as Rc
import findNearStops as Fns
import yaml


class MyCTable:
    def __init__(self) -> None:
        self.FullCost_mem = {}
        self.FlyCost_mem = {}

    def insert(self, d_id, fullCostArray, flyCostArray):
        self.FullCost_mem[d_id] = fullCostArray
        self.FlyCost_mem[d_id] = flyCostArray

    def DropOuters(self, depot_id=None):  # remove costumer if is out of reach from depots
        global requests

        ousterListForRemove = []
        if depot_id is None:
            for i in range(len(self.FlyCost_mem[0])):
                sumCostsPearDep = 0
                for d in range(len(Depots)):
                    sumCostsPearDep += self.FlyCost_mem[d][i]
                if sumCostsPearDep > len(Depots) * 5500:
                    # just drop or add to outer list
                    ousterListForRemove.append(i)
        else:
            for i in range(len(self.FlyCost_mem[depot_id])):
                if self.FlyCost_mem[depot_id][i] > MaxFlyDist * OUT_OF_REACH_COSTUMERS:
                    # just drop or add to outer list
                    ousterListForRemove.append(i)

        ousterListForRemove.reverse()
        for item in ousterListForRemove:
            requests.append(requests.pop(item))
            if depot_id is None:
                for d in range(len(Depots)):
                    self.FullCost_mem[d].pop(item)
                    self.FlyCost_mem[d].pop(item)
            else:
                self.FullCost_mem[depot_id].pop(item)
                self.FlyCost_mem[depot_id].pop(item)

    def getMinVal(self, dep):
        if len(self.FullCost_mem[dep]) == 0:
            return -1
        return np.min(self.FullCost_mem[dep])

    def getMinInd(self, dep):
        if len(self.FlyCost_mem[dep]) == 0:
            return -1
        return np.argmin(self.FlyCost_mem[dep])


def loc2point(inp):
    return (int(inp[0] / 10), int(inp[1] / 10))


def point2Loc(pnt):
    return [pnt.x, pnt.y]


with open('conf.yaml', 'r') as file:
    yaml_data = yaml.load(file, Loader=yaml.FullLoader)

config = json.load(open('config.json'))
# Options { 'greedlyDecide' , 'fairnessDecide', 'deepDecide', 'algorithm', 'TimingDecide}
approach = 'deepDecide' if yaml_data['DECISION_APPROACH'] == 'DEEP' else 'greedlyDecide'
requestConf_id = yaml_data['COSTUMER_DEST']

loadModel = False
showImage = False
createGif = yaml_data['CREATE_GIF']
UAVs = []
# reachTimeStoreXl = []
episod = 9999999
reach2finish = yaml_data['COSTUMER_COUNT'] * 2
workingTime = 60000
finisher = None
BussList = {}
Lines = {}
ReqSubSet = []
ReqSubSet_len = 50
Depots = []
uav_costumer = None

back2depotCount = 0
flyFailureCount = 0
costumer_id = 0
BUSS_DISTANCE_DELAY = 450
AVَAILABLE_STOP_COEFFICIENT = 0.3
OUT_OF_REACH_COSTUMERS = yaml_data['OUT_OF_REACH_COSTUMERS']
SOURCE_FLY_COEFFICIENT = yaml_data['SOURCE_FLY_COEFFICIENT']

storeFailure = info_storage()
rc = None
requests = []
BusStopStatus = None
images = []
colors = [(204, 0, 0), (204, 102, 0), (204, 204, 0), (102, 204, 0),
          (0, 204, 0), (0, 204, 102), (0, 204, 204), (0, 102, 204),
          (0, 0, 204), (102, 0, 204), (204, 0, 204), (204, 0, 102),
          (96, 96, 96)]  # 13

Actions = [point(0, 0), point(0, 1), point(0, -1), point(1, 0), point(-1, 0)]

'''
UAVPath = [[{"actionType": 'fly', "loc": point(1, 9)}, {"actionType": 'land', "loc": point(20, 40)}],
           [{"actionType": 'fly', "loc": point(9, 1)}, {
               "actionType": 'land', "loc": point(10, 10)}],
           [{"actionType": 'fly', "loc": point(0, 0)}]] '''


# local functions region  ( this functions may call in eachother)


def dpCost(depot, checkLen=None):
    global finisher
    global BussList
    global BusMaxCap
    global Lines_json
    global Lines
    global BusStopStatus
    global requests
    global rc

    if checkLen == None or checkLen > len(requests):
        checkLen = len(requests)
        if len(requests) < 10:
            f = open('requestConf' + str(requestConf_id) + '.json', 'r')
            requests = json.load(f)

    if not len(requests):
        print(" all requests Done.")
        finisher = True
        return [0 for i in range(checkLen)], [0 for i in range(checkLen)]

    totalCostList = []
    flyCostList = []

    for reqIndex in range(checkLen):
        start_point = Depots[depot].loc
        # change here if input requests location range Or point range
        endPoint = point(*requests[reqIndex])
        stoplist = Fns.nearStops(start_point, int(
            MaxFlyDist * AVَAILABLE_STOP_COEFFICIENT))
        stoplist = Fns.reviewNearStops(
            start_point, stoplist, Lines_json, BusStopStatus)
        hiperPower = 0
        while not len(stoplist):
            stoplist = Fns.nearStops(start_point, MaxFlyDist + hiperPower)
            stoplist = Fns.reviewNearStops(
                start_point, stoplist, Lines_json, BusStopStatus)
            print("Try To find BS to Come Back --> ", MaxFlyDist + hiperPower)
            hiperPower += 40

        netState = {'curLoc': start_point, 'destLoc': endPoint, 'BStop': StopStates(stoplist),
                    'BussList': BussList, 'BusMaxCap': BusMaxCap}
        if approach == 'fairnessDecide':
            tmp_cost_val, fly_cost = rc.Cost_fairness(
                stoplist, netState, Lines)
        elif approach == 'greedlyDecide':
            tmp_cost_val, fly_cost = rc.cost_greedy(
                stoplist, netState, lines=Lines)
        elif approach == 'TimingDecide':
            tmp_cost_val, fly_cost = rc.Cost_Timing(stoplist, netState, Lines)
        elif approach == 'deepDecide':
            tmp_cost_val, fly_cost = rc.Cost_deep(stoplist, netState, Lines)
        elif approach == 'algorithm':
            tmp_cost_val, fly_cost = rc.algorithm(stoplist, netState, Lines)
        else:
            tmp_cost_val = 0
            fly_cost = 0

        if fly_cost * 2 > MaxFlyDist:
            fly_cost = 6000

        totalCostList.append(tmp_cost_val)
        flyCostList.append(fly_cost)

    return totalCostList, flyCostList


def choise_task_from_subset(UAV_id, flied):
    global BussList
    global BusMaxCap
    global Lines_json
    global Lines
    global BusStopStatus
    MytempCostTable = MyCTable()
    finalRes = {}
    fly_count_c2d = 0

    start_point = UAVs[UAV_id].loc
    stoplist = Fns.nearStops(start_point, int(
        MaxFlyDist * AVَAILABLE_STOP_COEFFICIENT))
    stoplist = Fns.reviewNearStops(
        start_point, stoplist, Lines_json, BusStopStatus)
    hiperPower = 0
    while not len(stoplist):
        stoplist = Fns.nearStops(start_point, MaxFlyDist + hiperPower)
        stoplist = Fns.reviewNearStops(
            start_point, stoplist, Lines_json, BusStopStatus)
        print("Try To find BS to Come Back --> ", MaxFlyDist + hiperPower)
        hiperPower += 40
    netState = {'curLoc': start_point, 'BStop': StopStates(stoplist),
                'BussList': BussList, 'BusMaxCap': BusMaxCap}

    for dep in range(len(Depots)):
        endPoint = Depots[dep].loc

        # 'destLoc': endPoint ,
        netState['destLoc'] = endPoint

        if start_point == endPoint:
            tmp_cost_val = 0
            fly_count_c2d = 0
        else:
            if approach == 'fairnessDecide':
                tmp_cost_val, fly_count_c2d = rc.Cost_fairness(
                    stoplist, netState, Lines)
            elif approach == 'greedlyDecide':
                tmp_cost_val, fly_count_c2d = rc.cost_greedy(
                    stoplist, netState, lines=Lines)
            elif approach == 'TimingDecide':
                tmp_cost_val, fly_count_c2d = rc.Cost_Timing(
                    stoplist, netState, Lines)
            elif approach == 'deepDecide':
                tmp_cost_val, fly_count_c2d = rc.Cost_deep(
                    stoplist, netState, Lines)
            elif approach == 'algorithm':
                tmp_cost_val, fly_count_c2d = rc.algorithm(
                    stoplist, netState, Lines)

        # if fly_count_c2d + flied > MaxFlyDist:
        #     fly_count_c2d = 6000

        finalRes[dep] = fly_count_c2d

        # MytempCostTable.insert(dep, *dpCost(dep, ReqSubSet_len))

    # MytempCostTable.DropOuters()

    minCosts = []
    failed = False
    lowest_cost_depot_id = None
    lowest_cost_depot_value = None
    for d in range(len(Depots)):
        if lowest_cost_depot_value is None or lowest_cost_depot_value > finalRes[d]:
            lowest_cost_depot_value = finalRes[d]
            lowest_cost_depot_id = d
        # if all costumer removed it's mean for all depot removed
        # if MytempCostTable.getMinVal(d) == -1:
        #     failed = True
        #     break
        # minCosts.append(finalRes[d] + MytempCostTable.getMinVal(d))
    if failed:
        return [[UAVs[UAV_id].loc.x, UAVs[UAV_id].loc.y], [UAVs[UAV_id].loc.x, UAVs[UAV_id].loc.y]], 1

    # best_depot_id = np.argmin(minCosts)   # temporary comment for ensure that UAVs can reach to depot
    best_depot_id = lowest_cost_depot_id
    MytempCostTable.insert(
        best_depot_id, *dpCost(best_depot_id, ReqSubSet_len))
    MytempCostTable.DropOuters(depot_id=best_depot_id)
    # if finalRes[best_depot_id] > 5500:     # uav locked in costumer location and it seems that can't reach any depot
    #     return [[UAVs[UAV_id].loc.x, UAVs[UAV_id].loc.y], [UAVs[UAV_id].loc.x, UAVs[UAV_id].loc.y]], 1

    best_request_id = MytempCostTable.getMinInd(best_depot_id)

    result = [[Depots[best_depot_id].loc.x,
               Depots[best_depot_id].loc.y], requests.pop(best_request_id)]

    return result, Depots[best_depot_id].id


def task_manager():  # TODO Add loadSubSet function and check
    global finisher
    global costTbl
    global BussList
    global BusMaxCap
    global Lines
    global Lines_json
    global BusStopStatus
    global costumer_id
    global MAX_WAITING_TIME
    global uav_costumer

    for counter in range(UAVCount):
        if UAVs[counter].delay > 1:
            UAVs[counter].delay -= 1
            # destenation[counter] = {"actionType": 'delay', 'loc': 'there'}
            continue
        if UAVs[counter].delay == 1:
            UAVs[counter].delay = 0
        if not UAVs[counter].status:  # be 0 mean subtask done      # reached to depot or costumer
            global request_id
            print(counter, ": --- %s seconds ---" % (time.time() - start_time))
            Uav_request[counter] = request_id
            request_id += 1
            # if UAV is in costumer location and need cdC' task
            if len(UAVTasks[counter]) == 0 or UAVTasks[counter][0] is None:
                UAVs[counter].start_from = "costumer"
                if (not isnan(finisher)) and finisher:
                    return

                UAVTasks[counter], chosen_depot = choise_task_from_subset(
                    counter, UAVs[counter].flied)
                storeMore.increaseDepotUsed(str(chosen_depot))
                # if UAV in depot. in initial state
                if point(*UAVTasks[counter][0]) == UAVs[counter].loc:
                    UAVs[counter].flied = 0
                    UAVs[counter].start_from = "depot"
                    UAVTasks[counter].pop(0)
                    UAVTasks[counter].append(None)
                    storeData.setPathType(counter, 0)   # to costumer
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
                storeData.setDepot_id(counter, chosen_depot)

                '''T_task = requests.pop(0)
                UAVs[counter].loc = point(*T_task.pop(0))
                UAVTasks[counter] = T_task'''

            else:  # uav is in depot and recharge battery
                UAVs[counter].flied = 0
                UAVs[counter].start_from = "depot"
                storeData.setPathType(counter, 0)
                storeData.setCostumer_id(counter, costumer_id)
                uav_costumer[counter] = costumer_id
                costumer_id += 1

            stoplist = Fns.nearStops(UAVs[counter].loc, int(
                MaxFlyDist * AVَAILABLE_STOP_COEFFICIENT))
            stoplist = Fns.reviewNearStops(UAVs[counter].loc, stoplist, Lines_json,
                                           BusStopStatus)  # just keep 2 stop in each line

            hiperPower = 0
            while not len(stoplist):
                stoplist = Fns.nearStops(
                    UAVs[counter].loc, MaxFlyDist + hiperPower)
                stoplist = Fns.reviewNearStops(
                    UAVs[counter].loc, stoplist, Lines_json, BusStopStatus)
                print("Try To find BS to Come Back --> ",
                      MaxFlyDist + hiperPower)
                hiperPower += 40

            netState = {'curLoc': UAVs[counter].loc, 'destLoc': point(
                *UAVTasks[counter][0]), 'BStop': StopStates(stoplist), 'BussList': BussList, 'BusMaxCap': BusMaxCap}
            # lineBusyRateUpdate()
            er = 1
            if approach == 'fairnessDecide':
                memid, Sstop = rc.fairnessDecide(stoplist, netState, Lines)
            elif approach == 'greedlyDecide':
                Sstop = rc.greedy(stoplist, netState, lines=Lines)
                memid = 1
            elif approach == 'TimingDecide':
                memid, Sstop = rc.TimingDecide(stoplist, netState, Lines)
            elif approach == 'deepDecide':
                memid, Sstop, er = rc.decide(stoplist, netState, Lines)
            elif approach == 'algorithm':
                memid, Sstop = rc.algorithm(stoplist, netState, Lines)
            selected_line = Rf.findStopLine(int(Sstop))
            storeData.setRouteLine(counter, Rf.findStopLine(int(Sstop)))
            storeMore.storedecideParams(er, len(stoplist), selected_line)
            destini = point(*UAVTasks[counter].pop(0))
            nothing, route = Rf.find(int(Sstop), destini)
            route = list(map(str, route))
            UAVPath[counter], start_station_with_direction = route2path(        # for change line this function need to fix
                route, destini)
            UAVHistory[counter] = memid
            Lines[selected_line].stations[start_station_with_direction].coming.append(
                counter)

            coming2stations = len(
                Lines[selected_line].stations[start_station_with_direction].coming) - 1
            waiting2stations = len(
                Lines[selected_line].stations[start_station_with_direction].passengers)
            bus_count2reach = int(
                (coming2stations + waiting2stations) / BusMaxCap)
            # drop delay for busy stations
            if UAVs[counter].start_from == "depot" and ((bus_count2reach + 1) * BUSS_DISTANCE_DELAY) > MAX_WAITING_TIME:
                UAVs[counter].delay = bus_count2reach * BUSS_DISTANCE_DELAY


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


def StopStates(Slist):
    Slst = copy.deepcopy(Slist)
    Stt = {}
    for stp in Slst:
        # get po #get p #get loc
        T_s = {}
        T_s['passengers'] = len(BusStopStatus[str(stp) + '_0'].passengers)
        T_s['p'] = BusStopStatus[str(stp) + '_0'].successRate
        T_s['loc'] = point(*loc2point(BusStopsLoc[stp + '_0']))
        temp_beforeStops, T_s['freeSpace'], T_s['comingBuss'] = __busStopSuccessRate(
            str(stp) + '_0')
        T_s['beforeStops'] = temp_beforeStops
        T_s['coming'] = __flierCountToBS(str(stp) + '_0')
        sumBeforFlierCount = 0
        for cunt in temp_beforeStops:
            sumBeforFlierCount += __flierCountToBS(cunt)
        T_s['goingToBefore'] = sumBeforFlierCount
        Stt[stp + '_0'] = T_s

        T_s = {}
        T_s['passengers'] = len(BusStopStatus[str(stp) + '_1'].passengers)
        T_s['p'] = BusStopStatus[str(stp) + '_1'].successRate
        T_s['loc'] = point(*loc2point(BusStopsLoc[stp + '_1']))
        temp_beforeStops, T_s['freeSpace'], T_s['comingBuss'] = __busStopSuccessRate(
            str(stp) + '_1')
        T_s['beforeStops'] = temp_beforeStops
        T_s['coming'] = __flierCountToBS(str(stp) + '_1')
        sumBeforFlierCount = 0
        for cunt in temp_beforeStops:
            sumBeforFlierCount += __flierCountToBS(cunt)
        T_s['goingToBefore'] = sumBeforFlierCount
        Stt[stp + '_1'] = T_s

    return Stt


def __flierCountToBS(chicedBS):  # args Like => ('79_0') --> 2
    cnt = 0
    gool_BS = {"actionType": 'land', 'loc': chicedBS}
    for tmp_pth in UAVPath:
        if gool_BS in tmp_pth:
            cnt += 1
    return cnt


# args Like => ('79_1') ---> ['78_1','78_0','79_0'] ,4  count of from number stop before bus reach with which capasity
def __busStopSuccessRate(busStp):
    beforeStops = Rf.beforeStops(busStp)  # ,['78_1','78_0','79_0']
    comingBuss = []
    temporal_passengers = []

    for bus in BussList:
        if BussList[bus].lastStop in beforeStops:
            comingBuss.append(bus)  # [23, 12, 32 ,..] Buss_ID

    for bs_cnt in beforeStops:
        temporal_passengers.append(len(BusStopStatus[bs_cnt].passengers))

    freeSpace = 0
    for combus in comingBuss:
        B_freeSpace = BusMaxCap
        UavsOnThisBus = [i for i, x in enumerate(UonBusID) if x == combus]
        for tmp_uav in UavsOnThisBus:
            if destenation[tmp_uav]["actionType"] == "drive" and destenation[tmp_uav]["loc"] not in beforeStops:
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
    options, args = opt_parser.parse_args()
    return options


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
    global BusMaxCap
    global requests
    global UAVHistory
    global images
    global Lines
    global MaxFlyDist
    global MAX_WAITING_TIME
    global Uav_request
    global request_id
    global destenation
    global storeData
    global storeMore
    global Lines_json
    global uav_costumer

    Env_dim = int(traci.simulation.getNetBoundary()[1][0] / 10)
    # UAVCount = config['UAVCount']
    UAVCount = yaml_data['UAVS_COUNT']
    MaxFlyDist = config['MaxFlyDist']
    MAX_WAITING_TIME = MaxFlyDist * SOURCE_FLY_COEFFICIENT
    # BusMaxCap = config['BusMaxCap']
    BusMaxCap = yaml_data['BUS_CAPACITY']
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
    requests = [requests[i]
                for i in range(yaml_data['COSTUMER_COUNT'] + UAVCount)]
    lineFile = open('lineConfig.json')
    Lines_json = json.load(lineFile)
    lineFile.close()
    for line in Lines_json:
        new_line_details = Lines_json[line]
        new_line = Line_class(new_line_details["id"])
        for station in new_line_details["stops"]:
            new_line.addBusStation(
                str(station) + "_0", point(*loc2point(BusStopsLoc[str(station) + "_0"])))
            new_line.addBusStation(
                str(station) + "_1", point(*loc2point(BusStopsLoc[str(station) + "_1"])))
        Lines[line] = new_line

    setUavs()
    destenation = [None for i in range(UAVCount)]
    UonBusID = [-1] * UAVCount
    Uav_request = [None for i in range(UAVCount)]
    request_id = 0
    # images = [[] for i in range(UAVCount)]
    storeData = StoreData(UAVCount, reach2finish, len(Lines_json))
    storeMore = StoreMore()


def map_station2line(station_id):  # station_id = 79
    global Lines_json
    # if station_id is string
    if type(station_id) == str:
        station_id = int(station_id.split("_")[0])
    for line in Lines_json:
        if station_id in Lines_json[line]["stops"]:
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

    monitoring = 0
    takePic = False
    finisher = False
    UAVPath = [[] for i in range(UAVCount)]
    # UAVPath = [[{"actionType": 'fly', "loc": point(*loc2point(BusStopsLoc['85_0']))},{"actionType":'land','loc':'85_0'},
    #           {"actionType":'drive','loc':'95_0'},{"actionType":'rise','loc':point(1493, 852)},{'actionType':'fly', 'loc': point(1493, 852)},{'actionType':'finish'}], ]
    prev_Action = []

    # drop Delay **************
    '''if UAVCount > 40 :
        for i in range(40):
            UAVs[i].delay = 40'''
    # End drop Delay

    for i in range(UAVCount):
        prev_Action.append(Actions[0])

    printCounter = 1
    UAVActions = [None] * UAVCount
    LastDistance = [None] * UAVCount

    for i in range(episod):  # start to go
        traci.simulationStep()
        Bus_state_update()  # update buss location and last station and location
        task_manager()

        if i < 80:
            print("step: ", i, ": --- %s seconds ---" %
                  (time.time() - start_time))

        if finisher or i == workingTime:
            break

        for ctr in range(UAVCount):
            # when uav get new task, it's status is 0, and it's delay is 0 ## or UAVs[ctr].delay
            if not (UAVs[ctr].status):
                destenation[ctr] = UAVPath[ctr].pop(0)
                UAVs[ctr].status = 1
                UAVs[ctr].stepet = 0
                UAVs[ctr].wait_step = 0
                # UAVs[ctr].flied = 0   # should not set here for every path, just for first path (set in task_manager)
                UAVs[ctr].flayFail = False

        for tmp in range(UAVCount):
            if UAVs[tmp].delay:
                # storeData.incrementStep(tmp)
                continue
            elif destenation[tmp]["actionType"] == "back2depot":
                UAVs[tmp].status = 0
                UAVTasks[tmp] = []
                storeData.resetStepsData(tmp)

            elif destenation[tmp]["actionType"] == "finish":
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
            elif (destenation[tmp]["actionType"] == "fly"):
                storeData.incrementStep(tmp)
                if destenation[tmp]["loc"] == UAVs[tmp].loc:
                    storeData.storeTiming(Uav_request[tmp], tmp)
                    destenation[tmp] = UAVPath[tmp].pop(0)
                    if destenation[tmp]["actionType"] == "land":
                        goal_bs = destenation[tmp]['loc']
                        current_line = map_station2line(goal_bs)
                        if tmp not in Lines[current_line].stations[goal_bs].passengers:
                            Lines[current_line].stations[goal_bs].coming.remove(
                                tmp)
                            Lines[current_line].stations[goal_bs].passengers.append(
                                tmp)

            elif destenation[tmp]["actionType"] == "changeLine":
                current_line = traci.vehicle.getRouteID(UonBusID[tmp])
                Lines[current_line].busList[UonBusID[tmp]].passengers.pop(
                    Lines[current_line].busList[UonBusID[tmp]].passengers.index(tmp))
                UonBusID[tmp] = -1
                UAVs[tmp].loc = tmpVhclPnt
                destenation[tmp] = UAVPath[tmp].pop(0)
                new_station = destenation[tmp]['loc']
                new_line = map_station2line(new_station)
                Lines[new_line].stations[new_station].passengers.append(tmp)
                print("line changed from " + current_line + " to " + new_line)

            elif destenation[tmp]["actionType"] == "land":
                storeData.incrementStep(tmp)
                goal_bs = destenation[tmp]['loc']
                # BusStop witing vehicles
                current_line = map_station2line(goal_bs)

                TmpBusStopVhcl = traci.busstop.getVehicleIDs(goal_bs)
                if len(TmpBusStopVhcl) != 0:
                    prob = Lines[current_line].stations[goal_bs].successRate * \
                        Lines[current_line].stations[goal_bs].calCount
                    Lines[current_line].stations[goal_bs].calCount += 1
                    newProb = (
                        BusMaxCap - UonBusID.count(TmpBusStopVhcl[0])) / BusMaxCap
                    prob += newProb
                    Lines[current_line].stations[goal_bs].successRate = prob / \
                        Lines[current_line].stations[goal_bs].calCount
                    if UonBusID.count(TmpBusStopVhcl[0]) < BusMaxCap and Lines[current_line].stations[
                            goal_bs].passengers.index(
                            tmp) < BusMaxCap - UonBusID.count(TmpBusStopVhcl[0]):
                        UonBusID[tmp] = TmpBusStopVhcl[0]
                        destenation[tmp] = UAVPath[tmp].pop(0)
                        Lines[current_line].stations[goal_bs].passengers.remove(
                            tmp)
                        Lines[current_line].busList[TmpBusStopVhcl[0]
                                                    ].passengers.append(tmp)
                        storeData.storeTiming(Uav_request[tmp], tmp)

            elif destenation[tmp]["actionType"] == "drive":
                storeData.incrementStep(tmp)
                TmpBusStopVhcl = traci.busstop.getVehicleIDs(
                    destenation[tmp]['loc'])
                if UonBusID[tmp] in TmpBusStopVhcl:
                    storeData.storeTiming(Uav_request[tmp], tmp)
                    destenation[tmp] = UAVPath[tmp].pop(0)
                    TmpVhclPos = traci.vehicle.getPosition(UonBusID[tmp])
                    tmpVhclPnt = point(*loc2point(TmpVhclPos))
                    if destenation[tmp]["actionType"] == "rise":
                        storeData.resetStoreOption(tmp, "transport")
                        LastDistance[tmp] = tmpVhclPnt.distance(
                            destenation[tmp]['loc'])

            elif destenation[tmp]["actionType"] == "rise":
                storeData.incrementStep(tmp)
                TmpVhclPos = traci.vehicle.getPosition(UonBusID[tmp])
                tmpVhclPnt = point(*loc2point(TmpVhclPos))
                TmpVhclDstnc = tmpVhclPnt.distance(destenation[tmp]['loc'])
                if TmpVhclDstnc < LastDistance[tmp]:
                    LastDistance[tmp] = TmpVhclDstnc
                else:
                    current_line = traci.vehicle.getRouteID(UonBusID[tmp])
                    Lines[current_line].busList[UonBusID[tmp]].passengers.pop(
                        Lines[current_line].busList[UonBusID[tmp]].passengers.index(tmp))
                    UonBusID[tmp] = -1
                    UAVs[tmp].loc = tmpVhclPnt
                    destenation[tmp] = UAVPath[tmp].pop(0)
                    storeData.storeTiming(Uav_request[tmp], tmp)

        for cntr in range(UAVCount):  # update driveing UAVs location
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
        #     takePic = True
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
                          str(k.id), fill=(0, 0, 0), font=font)
            for k in range(UAVCount):
                pnt = None
                if len(UAVPath[k]) > 1:
                    pnt = UAVPath[k][-2]['loc']
                if len(UAVPath[k]) == 1:
                    pnt = destenation[k]['loc']
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
            if takePic:
                # '+ str(datetime.now())+'
                cv2.imwrite('./images/store/runConf.png', im)
                takePic = False
            monitoring = 10
            images.append(im_pil)
        monitoring -= 1
        # End create gif ***********'''

        storeData.storeLineCondition(LinesStatus())

        fliers = []

        for tmp in range(UAVCount):
            UAVs[tmp].stepet += 1
            if destenation[tmp]["actionType"] == "fly" and not UAVs[tmp].delay:
                fliers.append(tmp)
                UAVs[tmp].flied += 1

            # add if fly failed
            if UAVs[tmp].flied > MaxFlyDist and UAVs[tmp].flayFail is False and UAVs[tmp].start_from == "costumer":
                UAVs[tmp].flayFail = True
                flyFailureCount += 1
                if destenation[tmp]["actionType"] == "fly":
                    if "loc" in UAVPath[tmp][0]:    # fly -> land
                        goal_bs = UAVPath[tmp][0]['loc']
                        current_line = map_station2line(goal_bs)
                        situation = "fly to station"
                    else:
                        goal_bs = "near depot"
                        current_line = "passed"
                        situation = "fly to depot"
                elif destenation[tmp]["actionType"] == "land":
                    goal_bs = destenation[tmp]["loc"]
                    current_line = map_station2line(goal_bs)
                    situation = "waiting in station"
                else:
                    goal_bs = "there is a problem"
                    current_line = "Error"
                    situation = "somthing else with error"
                storeFailure.add_info(str(flyFailureCount) + " - UAV: "+str(tmp)+"  Flied: "+str(UAVs[tmp].flied)+"  From: "+str(
                    UAVs[tmp].start_from)+"  Stept: "+str(UAVs[tmp].stepet) + "  Wait: " + str(UAVs[tmp].wait_step) + "  Line: "+str(current_line) + "  Station: "+str(goal_bs) + " RowInFile: "+str(uav_costumer[tmp]) + " Situation: "+situation)

            if destenation[tmp]["actionType"] == "land":
                UAVs[tmp].wait_step += 1
                UAVs[tmp].flied += 1
                # if wait too much return to depot
                if UAVs[tmp].start_from == "depot" and UAVs[tmp].flied > MAX_WAITING_TIME:
                    goal_bs = destenation[tmp]['loc']
                    current_line = map_station2line(goal_bs)
                    if tmp in Lines[current_line].stations[goal_bs].passengers:
                        Lines[current_line].stations[goal_bs].passengers.remove(
                            tmp)

                    storeData.setPathType(tmp, 2, rowIndex=Uav_request[tmp])
                    destenation[tmp]["actionType"] = "fly"
                    min_dist = 3000
                    for dep in Depots:
                        if dep.loc.distance(UAVs[tmp].loc) < min_dist:
                            best_depot = dep
                            min_dist = dep.loc.distance(UAVs[tmp].loc)
                    destenation[tmp]["loc"] = best_depot.loc
                    # return UAVs costumer to request list
                    costumer_location_point = UAVPath[tmp][-2]["loc"]
                    requests.append(
                        [costumer_location_point.x, costumer_location_point.y])
                    # reset UAVs path
                    UAVPath[tmp] = [{'actionType': 'back2depot'}]
                    print("One more back 2 depot, id: ", tmp, " from line: ",
                          current_line, " to depot: ", best_depot.id)
                    back2depotCount += 1

        # print(i,": flier Count: ........    ",len(fliers))
        # to_test = np.array(destenation)
        # print("destinies",[point2Loc(i["loc"]) for i in to_test[fliers] ])

        fly = uav_tjp(Env_dim, group_uperbound)

        for i_imp in fliers:
            fly.imp_uav(UAVs[i_imp].loc, prev_Action[i_imp],
                        destenation[i_imp]["loc"])

        actNumber = fly.go_forward()
        UAVActions = convertActionFromString(actNumber, len(fliers))
        # print("Fliers Act: ", [ pointPrint(flier) for flier in UAVActions])
        # go to next step
        for tmp in range(len(fliers)):
            UAVs[fliers[tmp]].loc = UAVs[fliers[tmp]].loc + UAVActions[tmp]
            prev_Action[fliers[tmp]] = UAVActions[tmp]


# --> [flying to 1 , waiting on 1, driving on 1, flying to 2, waiting on 2, ... ]
def LinesStatus():
    global Lines_json
    res = []
    for line in Lines_json:
        flierCounter = 0
        flierCounterReverse = 0
        BStationPassengers = 0
        BStationPassengersReverse = 0
        Linepassenger = 0
        LinepassengerReverse = 0

        for BStation in Lines_json[line]['stops']:
            flierCounter += __flierCountToBS(str(BStation) + '_0')
            flierCounterReverse += __flierCountToBS(str(BStation) + '_1')
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


def pointPrint(pnt):
    if pnt.x == 1:
        return "right"
    if pnt.x == -1:
        return "left"
    if pnt.y == 1:
        return "up"
    if pnt.y == -1:
        return "down"
    return "stay"


def Bus_state_update():
    for line in Lines:
        for bs in Lines[line].stations:
            vehicles_in_busStation = traci.busstop.getVehicleIDs(bs)
            for veh in vehicles_in_busStation:
                Lines[line].busList[veh].setLast(bs)
                Lines[line].busList[veh].direction = int(bs[-1])
        for veh in Lines[line].busList:
            Lines[line].busList[veh].loc = point(
                *loc2point(traci.vehicle.getPosition(veh)))


def __loadSimulationParams():
    T_BusList = traci.vehicle.getIDList()
    for bs in T_BusList:
        # *****# lineConfig.js -> line name must bi Like what is in sumo configuration
        routeID = traci.vehicle.getRouteID(bs)
        BussList[bs] = Bus(bs, routeID, point(
            *loc2point(traci.vehicle.getPosition(bs))))
        Lines[routeID].addBus(bs, routeID, point(
            *loc2point(traci.vehicle.getPosition(bs))))


def __loadBrain():
    global rc
    global Lines_json
    global Lines
    rc = Rc.brain(UAVCount, LoadModel=loadModel, lines=Lines)


class DegubTools:
    def find_uav(self, uav_id):
        global Lines
        for line in Lines:
            res = Lines[line].have_passenger(uav_id)
            if res != False:
                print("UAV ", uav_id, " is in line ", line, " and ", res)
                return line, res


# Starter
if __name__ == "__main__":

    print("have vars: ", back2depotCount, flyFailureCount)

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

    debug_tools = DegubTools()
    start_time = time.time()
    __loadSimulationParams()
    __loadBrain()
    go_forward()
    print("--- %s seconds ---" % (time.time() - start_time))
    print("Back 2 depot count : ", back2depotCount)

    moreData2json = {"back2depotCount": back2depotCount,
                     "flyFailureCount": flyFailureCount}
    jsonString = json.dumps(moreData2json)
    jsonFile = open("result/moreInJson_" + str(UAVCount) + "_" + str(len(Depots)) + "_" + time.strftime(
        '%Y-%m-%d_%H-%M-%S') + ".json", "w")
    jsonFile.write(jsonString)
    jsonFile.close()

    storeFailure.write_info_to_file("result/flyFailureCount_" + str(UAVCount) + "_" + str(len(Depots)) + "_" + time.strftime(
        '%Y-%m-%d_%H-%M-%S') + ".txt")

    '''pic = open('picpath.json','w')
    pic.write(json.dumps(images))'''
    storeMore.setKeyVal('running Time', time.time() - start_time)
    storeData.SaveToFile(str(UAVCount) + "-" + str(len(Depots)))
    storeMore.Save2file(str(UAVCount) + "-" + str(len(Depots)))
    # rc.saveModel(UAVCount)

    if createGif:
        images[0].save('./images/_image_draw.gif',
                       save_all=True, append_images=images[1:], duration=300, loop=0)

    traci.close()
