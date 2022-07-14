from cmath import isnan
from itertools import count
import os
from re import S
import keyboard
import sys
import time
import json
import random
import copy
import optparse
from DataStore import StoreData
from StoreMoreData import StoreMore
import cv2
import xlsxwriter
import xlwt
import numpy as np
from sumolib import checkBinary
import traci
from PIL import Image, ImageDraw, ImageFont
from point import point
from traci.main import switch
from Elements import UAV, BusStop, Bus, Depot, CostTable
from uav_tjp import uav_tjp
import routeFinding as rf
import routeController as RC
import findNearStops as fns
from datetime import datetime


class MyCTable:
    def __init__(self) -> None:
        self.FullCost_mem = {}
        self.FlyCost_mem = {}

    def insert(self, d_id, fullCostArray, flyCostArray):
        self.FullCost_mem[d_id] = fullCostArray
        self.FlyCost_mem[d_id] = flyCostArray

    def DropOuters(self):
        ousterListForRemove = []
        for i in range(len(self.FlyCost_mem[0])):
            sumCostsPearDep = 0
            for d in range(len(Depots)):
                sumCostsPearDep += self.FlyCost_mem[d][i]
            if sumCostsPearDep > len(Depots) * 5500:
                #just drop or add to outer list
                ousterListForRemove.append(i)
        ousterListForRemove.reverse()
        for item in ousterListForRemove:
            requests.pop(item)
            for d in range(len(Depots)):
                self.FullCost_mem[d].pop(item)
                self.FlyCost_mem[d].pop(item)
    
    def getMinVal(self, dep):
        if len(self.FullCost_mem[dep]) == 0:
            return -1
        return np.min(self.FullCost_mem[dep])

    def getMinInd(self, dep):
        if len(self.FlyCost_mem[dep]) == 0:
            return -1
        return np.argmin(self.FlyCost_mem[dep])
        

def loc2point(inp):
    return (int(inp[0]/10), int(inp[1] / 10))


def point2Loc(pnt):
    return [pnt.x, pnt.y]

config = json.load(open('config.json'))
# Options { 'greedlyDecide' , 'fairnessDecide', 'deepDecide', 'algorithm', 'TimingDecide}
approach = 'deepDecide'
showImage = False
UAVs = []
#reachTimeStoreXl = []
episod = 9999999
reach2finish = 20000
workingTime = 60000
finisher = None
BussList = {}
Lines = []
ReqSubSet = []
ReqSubSet_len = 25
Depots = []
unreachablePoints = []
requestConf_id = 1

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


def dpCost(depot, checkLen = None):
    global finisher
    global BussList
    global BusMaxCap
    global Lines
    global BusStopStatus
    global requests
    if checkLen == None or checkLen > len(requests):
        checkLen = len(requests)
        if len(requests) <10:
            f = open('requestConf'+str(requestConf_id)+'.json', 'r')
            requests = json.load(f)

    if not len(requests):
        print(" all requests Done.")
        finisher = True
        return [0 for i in range(checkLen)], [0 for i in range(checkLen)]

    totalCostList = []
    flyCostList = []
    
    for reqIndex in range(checkLen):
        startPoint = Depots[depot].loc
        endPoint = point(*requests[reqIndex])   # change here if input requests location range Or point range
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
    MytempCostTable = MyCTable()
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
    faild = False
    for d in range(len(Depots)):
        if MytempCostTable.getMinVal(d) == -1:
            faild = True
            break
        minCosts.append(finalRes[d] + MytempCostTable.getMinVal(d))
    if faild:
        return [[UAVs[UAV_id].loc.x, UAVs[UAV_id].loc.y], [UAVs[UAV_id].loc.x, UAVs[UAV_id].loc.y]], 1
    bestDep_id = np.argmin(minCosts)
    if finalRes[bestDep_id] >5500:
        return [[UAVs[UAV_id].loc.x, UAVs[UAV_id].loc.y], [UAVs[UAV_id].loc.x, UAVs[UAV_id].loc.y]], 1
    bestReq_id = MytempCostTable.getMinInd(bestDep_id)
    
    result = [[Depots[bestDep_id].loc.x, Depots[bestDep_id].loc.y], requests.pop(bestReq_id)]
        
    return result, Depots[bestDep_id].id

    


def taskManager(): #TODO Add loadSubSet function and check 
    global finisher
    global costTbl
    global BussList
    global BusMaxCap
    global Lines
    global BusStopStatus
    for counter in range(UAVCount):
        if UAVs[counter].delay > 1:
            UAVs[counter].delay -= 1
            destenation[counter] = {"actionType": 'delay', 'loc': 'there'}
            continue
        if UAVs[counter].delay == 1:
            UAVs[counter].delay = 0
        if not UAVs[counter].status:  # be 0 mean subtask done
            global request_id
            print(counter, ": --- %s seconds ---" % (time.time() - start_time))
            Uav_request[counter] = request_id
            request_id += 1
            stoplist = []
            if len(UAVTasks[counter]) == 0:
                
                if (not isnan(finisher)) and finisher :
                    return
                
                UAVTasks[counter], choisedDepot = choiseTaskFromSubset(counter, UAVs[counter].flied)
                storeMore.increaseDepotUsed(str(choisedDepot))
                if point(*UAVTasks[counter][0]) == UAVs[counter].loc:
                    UAVTasks[counter].pop(0)

                '''T_task = requests.pop(0)
                UAVs[counter].loc = point(*T_task.pop(0))
                UAVTasks[counter] = T_task'''
                
            # End if
                
            stoplist = fns.nearStops(UAVs[counter].loc, int(MaxFlyDist * 0.45))
            stoplist = fns.reviewNearStops(UAVs[counter].loc, stoplist, Lines, BusStopStatus)

            hiperPower = 0
            while not len(stoplist):
                stoplist = fns.nearStops(UAVs[counter].loc, MaxFlyDist + hiperPower)
                stoplist = fns.reviewNearStops(UAVs[counter].loc, stoplist, Lines, BusStopStatus)
                print("Try To find BS to Come Back --> ", MaxFlyDist + hiperPower)
                hiperPower += 40
                

            netState = {'curLoc': UAVs[counter].loc, 'destLoc': point(
                *UAVTasks[counter][0]), 'BStop': StopStates(stoplist), 'BussList': BussList, 'BusMaxCap':BusMaxCap}
            lineBusyRateUpdate()
            er = 1
            if approach == 'fairnessDecide':
                memid, Sstop = rc.fairnessDecide(stoplist, netState, Lines)
            elif approach == 'greedlyDecide':
                memid, Sstop = rc.greedlyDecide(stoplist, netState, Lines)
            elif approach == 'TimingDecide':
                memid, Sstop = rc.TimingDecide(stoplist, netState, Lines)
            elif approach == 'deepDecide':
                memid, Sstop, er = rc.decide(stoplist, netState, Lines)
            elif approach == 'algorithm':
                memid, Sstop = rc.algorithm(stoplist, netState, Lines)
            storeData.setRouteLine(counter, rf.findStopLine(int(Sstop)))
            storeMore.storedecideParams(er, len(stoplist))
            destini = point(*UAVTasks[counter].pop(0))
            nothing, route = rf.find(int(Sstop), destini)
            route = list(map(str, route))
            UAVPath[counter] = route2path(route,  destini)
            UAVHistory[counter] = memid


def lineBusyRateUpdate():
    for Lin in Lines:
        passengerCount_0 = 1
        passengerCount_1 = 1
        for cntr in Lines[Lin]['stops']:
            passengerCount_0 += len(BusStopStatus[str(cntr)+'_0'].passengers)
            passengerCount_1 += len(BusStopStatus[str(cntr)+'_1'].passengers)

        Lines[Lin]['busyRate_0'] = 1 - (1/passengerCount_0)
        Lines[Lin]['busyRate_1'] = 1 - (1/passengerCount_1)


def route2path(routeOrg, dest):
    route = copy.deepcopy(routeOrg)
    for i in range(0, len(route), 2):
        if int(route[i]) < int(route[i+1]):
            route[i] += '_0'
            route[i+1] += '_0'
        else:
            route[i] += '_1'
            route[i+1] += '_1'

    pth = []
    pth.append({"actionType": 'fly', "loc": point(
        *loc2point(BusStopsLoc[route[0]]))})
    while(len(route) > 0):
        pth.append({"actionType": 'land', 'loc': route.pop(0)})
        pth.append({"actionType": 'drive', 'loc': route.pop(0)})
    pth.append({"actionType": 'rise', 'loc': dest})
    pth.append({'actionType': 'fly', 'loc': dest})
    pth.append({'actionType': 'finish'})
    return pth


def StopStates(Slist):
    Slst = copy.deepcopy(Slist)
    Stt = {}
    for stp in Slst:
        # get po #get p #get loc
        T_s = {}
        T_s['passengers'] = len(BusStopStatus[str(stp)+'_0'].passengers)
        T_s['p'] = BusStopStatus[str(stp) + '_0'].successRate
        T_s['loc'] = point(*loc2point(BusStopsLoc[stp+'_0']))
        temp_beforeStops, T_s['freeSpace'],  T_s['comingBuss'] = __busStopSuccessRate(
            str(stp)+'_0')
        T_s['beforeStops'] = temp_beforeStops
        T_s['coming'] = __flierCountToBS(str(stp)+'_0')
        sumBeforFlierCount = 0
        for cunt in temp_beforeStops:
            sumBeforFlierCount += __flierCountToBS(cunt)
        T_s['goingToBefore'] = sumBeforFlierCount
        Stt[stp + '_0'] = T_s

        T_s = {}
        T_s['passengers'] = len(BusStopStatus[str(stp)+'_1'].passengers)
        T_s['p'] = BusStopStatus[str(stp) + '_1'].successRate
        T_s['loc'] = point(*loc2point(BusStopsLoc[stp+'_1']))
        temp_beforeStops, T_s['freeSpace'], T_s['comingBuss'] = __busStopSuccessRate(
            str(stp)+'_1')
        T_s['beforeStops'] = temp_beforeStops
        T_s['coming'] = __flierCountToBS(str(stp)+'_1')
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
    beforeStops = rf.beforeStops(busStp)  # ,['78_1','78_0','79_0']
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
        for tmp_BS in range(beforeStops.index(BussList[combus].lastStop)+1, len(beforeStops)):
            while(B_freeSpace > 0):
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
    global rc
    global BusStopStatus
    global BusMaxCap
    global requests
    global UAVHistory
    global images
    global Lines
    global MaxFlyDist
    global Uav_request
    global request_id
    global destenation
    global storeData
    global storeMore

    Env_dim = int(traci.simulation.getNetBoundary()[1][0] / 10)
    UAVCount = config['UAVCount']
    MaxFlyDist = config['MaxFlyDist']
    BusMaxCap = config['BusMaxCap']
    group_uperbound = config["group_uperbound"]
    BusStopsLoc = config["BusStopsLoc"]
    tmpDepots = config["Depots"]
    Depots = [Depot(iD, point(tmpDepots[iD]['x'], tmpDepots[iD]['y'])) for iD in tmpDepots]
    BusStopStatus = {iD: BusStop(iD, point(*loc2point(BusStopsLoc[iD]))) for iD in BusStopsLoc}
    UAVTasks = [[] for i in range(UAVCount)]
    UAVHistory = np.zeros(UAVCount)
    rc = RC.brain(UAVCount)
    f = open('requestConf'+str(requestConf_id)+'.json', 'r')
    requests = json.load(f)
    f.close()
    lineFile = open('lineConfig.json')
    Lines = json.load(lineFile)
    lineFile.close()
    setUavs()
    destenation = [None for i in range(UAVCount)]
    UonBusID = [-1] * UAVCount
    Uav_request = [None for i in range(UAVCount)]
    request_id = 0
    images = [[] for i in range(UAVCount)]
    storeData = StoreData(UAVCount, reach2finish, len(Lines))
    storeMore = StoreMore()


def setUavs():
    UAV_conf = config['UAVs']
    for i in range(1, UAVCount+1):
        UAVs.append(
            UAV(Depots[UAV_conf[f"{i}"]['depot'] - 1].loc , i))
            #UAV(point(UAV_conf[f"{i}"]['x'], UAV_conf[f"{i}"]['y']), i))


# main function region


def go_forward():
    global UAVPath
    global images
    global showImage
    global finisher
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

    for i in range(episod):     # start to go
        traci.simulationStep()
        Bus_state_update()
        taskManager()

        if i < 120:
            print("step: ",i, ": --- %s seconds ---" % (time.time() - start_time))

        if finisher or i == workingTime:
            break

        for ctr in range(UAVCount):
            if not (UAVs[ctr].status or UAVs[ctr].delay):
                destenation[ctr] = UAVPath[ctr].pop(0)
                UAVs[ctr].status = 1
                UAVs[ctr].stepet = 0
                UAVs[ctr].flied = 0

        for tmp in range(UAVCount):

            if destenation[tmp]["actionType"] == "finish":
                UAVs[tmp].status = 0
                print(printCounter, ": hey, I'm ", tmp, " in destination. (",
                      UAVs[tmp].loc.x, ", ", UAVs[tmp].loc.y, ") ..|:)", ' with ', UAVs[tmp].stepet, ' whole step and ', UAVs[tmp].flied, " fly step")
                if approach == 'deepDecide':
                    rc.saveData(UAVHistory[tmp],
                                UAVs[tmp].flied, UAVs[tmp].stepet)

                storeData.storeTiming(Uav_request[tmp], tmp)
                storeData.increseReachs()
                #sheet.write(Uav_request[tmp], 0, printCounter)
                #sheet.write(Uav_request[tmp], 1, UAVs[tmp].stepet)
                printCounter += 1
                if printCounter == reach2finish:
                    finisher = True
            elif (destenation[tmp]["actionType"] == "fly"):
                storeData.incrementStep(tmp)
                if destenation[tmp]["loc"] == UAVs[tmp].loc:
                    storeData.storeTiming(Uav_request[tmp], tmp)
                    destenation[tmp] = UAVPath[tmp].pop(0)

            elif destenation[tmp]["actionType"] == "land":
                storeData.incrementStep(tmp)
                goolBS = destenation[tmp]['loc']
                # BusStop witing vehicles
                if tmp not in BusStopStatus[goolBS].passengers:
                    BusStopStatus[goolBS].passengers.append(tmp)
                TmpBusStopVhcl = traci.busstop.getVehicleIDs(goolBS)
                if len(TmpBusStopVhcl) != 0:
                    prob = BusStopStatus[goolBS].successRate * \
                        BusStopStatus[goolBS].calCount
                    BusStopStatus[goolBS].calCount += 1
                    newProb = (
                        BusMaxCap - UonBusID.count(TmpBusStopVhcl[0])) / BusMaxCap
                    prob += newProb
                    BusStopStatus[goolBS].successRate = prob / \
                        BusStopStatus[goolBS].calCount
                    if UonBusID.count(TmpBusStopVhcl[0]) < BusMaxCap and BusStopStatus[goolBS].passengers.index(tmp) < BusMaxCap - UonBusID.count(TmpBusStopVhcl[0]):
                        UonBusID[tmp] = TmpBusStopVhcl[0]
                        destenation[tmp] = UAVPath[tmp].pop(0)
                        BusStopStatus[goolBS].passengers.pop(
                            BusStopStatus[goolBS].passengers.index(tmp))
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
                    else:
                        UonBusID[tmp] = -1
                        storeData.resetStoreOption(tmp, "wait")
                        

            elif destenation[tmp]["actionType"] == "rise":
                storeData.incrementStep(tmp)
                TmpVhclPos = traci.vehicle.getPosition(UonBusID[tmp])
                tmpVhclPnt = point(*loc2point(TmpVhclPos))
                TmpVhclDstnc = tmpVhclPnt.distance(destenation[tmp]['loc'])
                if TmpVhclDstnc < LastDistance[tmp]:
                    LastDistance[tmp] = TmpVhclDstnc
                else:
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

        if keyboard.is_pressed('P'):
            takePic = True
            time.sleep(0.5)

        # '''
        if showImage and monitoring < 1:
            im = Image.open('./images/tehranSumo.jpg')
            draw = ImageDraw.Draw(im)
            width, height = im.size
            font = ImageFont.truetype("arial.ttf", 10)
            for k in UAVs:
                point2pixcel_x = k.loc.x
                point2pixcel_y = height - k.loc.y
                clr = (0, 0, 0)
                if not k.id == 1:
                    clr = colors[int(k.id % 13)]
                draw.rounded_rectangle([point2pixcel_x-10, point2pixcel_y-10, point2pixcel_x+10, point2pixcel_y+10],
                                       radius=10, fill=clr)
            for k in range(UAVCount):
                pnt = None
                if len(UAVPath[k])>1:
                    pnt = UAVPath[k][-2]['loc']
                if len(UAVPath[k]) == 1:
                    pnt = destenation[k]['loc']
                if pnt != None:
                    point2pixcel_x = pnt.x
                    point2pixcel_y = height - pnt.y
                    clr = (230, 21, 21)
                    #draw.text([point2pixcel_x, point2pixcel_y], "*", fill=clr, font=font)
                    draw.rounded_rectangle([point2pixcel_x-7, point2pixcel_y-7, point2pixcel_x+7, point2pixcel_y+7],
                                       radius=5, fill=clr)

            for k in Depots:
                point2pixcel_x = k.loc.x
                point2pixcel_y = height - k.loc.y
                draw.polygon([(point2pixcel_x, point2pixcel_y-30), (point2pixcel_x-30, point2pixcel_y), (point2pixcel_x+30,point2pixcel_y)], fill = (245, 70, 60))

            im = im.resize((800, 800))
            im = np.array(im)
            cv2.destroyAllWindows()
            #cv2.namedWindow("tehran", cv2.WINDOW_NORMAL)
            #im = cv2.resize(im, (960, 540))
            cv2.imshow('tehran', im)
            cv2.waitKey(40)
            if takePic:
                cv2.imwrite('./images/store/runConf.png', im ) #'+ str(datetime.now())+'
                takePic = False
            monitoring = 10
        monitoring -= 1

        storeData.storeLineCondition(LinesStatus())

        # images.append(im)
        # End create gif ***********'''
        '''
        print (i, end="", flush= True)
        for k in range(UAVCount):
            print(" U",k,":(",UAVs[k].loc.x,",",UAVs[k].loc.y,"), ", end="", flush= True)
            # End group reward check
        
        print("\n") '''

        fliers = []

        for tmp in range(UAVCount):
            UAVs[tmp].stepet += 1
            if destenation[tmp]["actionType"] == "fly":
                UAVs[tmp].flied += 1
                fliers.append(tmp)
        #print(i,": flier Count: ........    ",len(fliers))
        #totest = np.array(destenation)
        #print("destinis",[point2Loc(i["loc"]) for i in totest[fliers] ])

        fly = uav_tjp(Env_dim, group_uperbound)

        for i_imp in fliers:
            fly.imp_uav(UAVs[i_imp].loc, prev_Action[i_imp],
                        destenation[i_imp]["loc"])

        actNumber = fly.go_forward()
        UAVActions = convertActionFromString(actNumber, len(fliers))
        #print("Fliers Act: ", [ pointPrint(flier) for flier in UAVActions])
        # go to next step
        for tmp in range(len(fliers)):
            UAVs[fliers[tmp]].loc = UAVs[fliers[tmp]].loc + UAVActions[tmp]
            prev_Action[fliers[tmp]] = UAVActions[tmp]


# --> [flying to 1 , witing on 1, driving on 1, flying to 2, witing on 2, ... ]
def LinesStatus():
    res = []
    for line in Lines:
        flierCounter = 0
        flierCounterReverse = 0
        BStationPassengers = 0
        BStationPassengersReverse = 0
        Linepassenger = 0
        LinepassengerReverse = 0

        for BStation in Lines[line]['stops']:
            flierCounter += __flierCountToBS(str(BStation) + '_0')
            flierCounterReverse += __flierCountToBS(str(BStation) + '_1')
            BStationPassengers += len(
                BusStopStatus[str(BStation)+'_0'].passengers)
            BStationPassengersReverse += len(
                BusStopStatus[str(BStation)+'_1'].passengers)

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
    for bs in BusStopsLoc:
        vehInBs = traci.busstop.getVehicleIDs(bs)
        for veh in vehInBs:
            BussList[veh].setLast(bs)
            BussList[veh].direction = int(bs[-1])
            BussList[veh].loc = point(
                *loc2point(traci.vehicle.getPosition(veh)))


def __loadSimulationParams():
    T_BusList = traci.vehicle.getIDList()
    for bs in T_BusList:
        # *****# lineConfig.js -> line name must bi Like what is in sumo configuration
        routeID = traci.vehicle.getRouteID(bs)
        BussList[bs] = Bus(bs, routeID, point(
            *loc2point(traci.vehicle.getPosition(bs))))


# Starter
if __name__ == "__main__":

    # sumo config
    options = get_options()

    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo')  # 'sumo-gui'

    traci.start([sumoBinary, "-c", "osm.sumocfg",
                "--tripinfo-output", "tripinfo.xml"])

    for distrubute in range(13000):
        traci.simulation.step()

    start_time = time.time()
    loadConfig()  # import config
    __loadSimulationParams()
    go_forward()
    print("--- %s seconds ---" % (time.time() - start_time))

    '''pic = open('picpath.json','w')
    pic.write(json.dumps(images))'''
    storeMore.setKeyVal('running Time', time.time() - start_time)
    storeData.SaveToFile(UAVCount)
    storeMore.Save2file(UAVCount)
    traci.close()

    # images[0].save('./images/_imagedraw.gif',
    #               save_all=True, append_images=images[1:], optimize=False, duration=400, loop=0)
