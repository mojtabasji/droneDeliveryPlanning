from collections import deque
from copy import deepcopy
import random
from re import L
from time import sleep
from point import point
import routeFinding as rf
from keras import models
from keras.models import Sequential, load_model
from keras.layers import Dense, Dropout, Flatten, Reshape
from tensorflow.keras.optimizers import Adam
import numpy as np
import random
import re
import os
import yaml
from Elements import UAV, Line_class, Bus

yaml_data = None
with open('conf.yaml', 'r') as file:
    yaml_data = yaml.load(file, Loader=yaml.FullLoader)

MIN_TRANSPORT_PATH = yaml_data['MIN_TRANSPORT_PATH']
DEEP_INITIALIZE_APPROACH = yaml_data['DEEP_INITIALIZE_APPROACH']
LOAD_MODEL = yaml_data['LOAD_MODEL']

TRANSPORT_REDUCE = 0.5
_2n = [2 ** i for i in range(1, 12)]


def first_collision_2n(inp):
    for i in _2n:
        if inp < i:
            return i


class ANN:
    def __init__(self, line_num=0, line_count=1, UAVCount=0, inpNodeCount=(1, 2), decay=0.995):
        self.lineNum = line_num
        self.lineCount = line_count
        self.UAVCount = UAVCount
        self.explore_rate = 1.0
        self.explore_min = 0.03
        self.explore_decay = decay
        self.inpNodeCount = inpNodeCount
        self.MemoryX = deque(maxlen=1000)
        self.MemoryY = deque(maxlen=1000)
        self.model = self.__build_model()
        self.outGama = 2
        self.sample_batch_size = UAVCount / line_count

    def __build_model(self):
        layer_node = first_collision_2n(self.inpNodeCount[0] * self.inpNodeCount[1])
        mdl = Sequential()
        mdl.add(Reshape((self.inpNodeCount[0] * self.inpNodeCount[1],), input_shape=self.inpNodeCount))
        mdl.add(Dense(layer_node, activation='relu'))
        mdl.add(Dense(layer_node, activation='softplus'))
        mdl.add(Dense(1, activation='linear'))  # linear softmax  sigmoid  softplus
        mdl.compile(loss='mse', optimizer=Adam(learning_rate=0.002),
                    metrics=['mae', 'mse'])  # optimizer='rmsprop'
        return mdl

    def predict(self, inp):
        return self.model.predict(inp)

    def save_model(self):
        self.model.save(f'weights/U_Brine_weight_line{str(self.lineNum)}.h5')

    def load_model(self):
        self.model = load_model(f'weights/U_Brine_weight_line{str(self.lineNum)}.h5')

    def train(self):
        if len(self.MemoryX) < self.sample_batch_size * 2 or self.explore_rate < self.explore_min:
            return
        sample_count = self.UAVCount
        if len(self.MemoryX) < self.UAVCount:
            sample_count = len(self.MemoryX)
        batch_index_list = random.sample(
            range(len(self.MemoryX)), sample_count)
        Xtrain = self.__subSet(self.MemoryX, batch_index_list)
        Ytrain = self.__subSet(self.MemoryY, batch_index_list)

        try:
            Xtrain = np.array(Xtrain, dtype=np.float16)
            Ytrain = np.array(Ytrain, dtype=np.float16)
            Xtrain = np.reshape(Xtrain, (sample_count, *self.inpNodeCount))
            Ytrain = np.reshape(Ytrain, (sample_count, 1))
        except:
            print("Xtrain: ", Xtrain)

        self.model.fit(Xtrain, Ytrain, epochs=1,
                       validation_split=0.1, verbose=0)
        if self.explore_rate > self.explore_min:
            self.explore_rate *= self.explore_decay

    def __subSet(self, set, setIndexs):
        newList = []
        for i in setIndexs:
            newList.append(set[i])
        return newList


class brain:
    def __init__(self, UAVCount, LoadModel=False, lines={}, exploration_decay=0.995) -> None:
        self.weight_backup = "U_Brine_weight.h5"
        self.temporalMemory = []  # deque(UAVCount)
        self.ann = {}
        line_count = len(lines)
        line_attubutes = list(lines.items())
        for i in range(line_count):
            input_nodes = lines[line_attubutes[i][0]].get_bus_station_count(
            ) + (lines[line_attubutes[i][0]].get_bus_count() * 2) + 1
            self.ann[line_attubutes[i][0]] = ANN(line_num=i, line_count=line_count, UAVCount=UAVCount,
                                                 inpNodeCount=(int(input_nodes), int(2)), decay=exploration_decay)
        self.outGama = 2
        self.Ucount = UAVCount

        if LOAD_MODEL:
            for i in range(line_count):
                self.ann[line_attubutes[i][0]].load_model()

    def save_model_weights(self):
        for i in self.ann:
            self.ann[i].save_model()

    def Cost_deep(self, stopsList, state, Lines):
        efected_lines = []
        for stp in stopsList:
            line_name = rf.findStopLine(int(stp))
            efected_lines.append(line_name)
        random_value = np.random.rand()
        if any([random_value <= self.ann[i].explore_rate for i in efected_lines]):
            if DEEP_INITIALIZE_APPROACH == 'RANDOM':
                # remove wrong line stations
                index = 0
                failure_list = []
                for stp in stopsList:
                    t, route = rf.find(int(stp), state['destLoc'])
                    Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
                    if Cost['transport'] < MIN_TRANSPORT_PATH:
                        failure_list.append(index)
                    index += 1
                failure_list.reverse()
                for i in failure_list:
                    stopsList.pop(i)

                choiced = np.random.choice(stopsList)
            elif DEEP_INITIALIZE_APPROACH == 'GREEDY':
                choiced, _ = self.greedy(stopsList, state, lines=Lines)
            else:
                print("ERROR: DEEP_INITIALIZE_APPROACH is not defined")
                exit(1)
        else:
            choiced = ''
            maxRew = 900000
            for stp in stopsList:
                inpnodes, stopInDirection = self.__inpCreate(stp, state, Lines)
                line_name = rf.findStopLine(int(stp))
                X_predict = np.reshape(inpnodes, (1, *self.ann[line_name].inpNodeCount))
                pval = self.ann[line_name].predict(X_predict)

                t, route = rf.find(int(stp), state['destLoc'])
                Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
                sumTime = Cost['destfly'] + Cost['sourcefly'] + (Cost['transport'] * TRANSPORT_REDUCE) + pval[0][0]

                # * (1 / (state['BStop'][stopInDirection]['passengers'] + 1))
                if sumTime < maxRew:
                    maxRew = sumTime
                    choiced = stp
        t, route = rf.find(int(choiced), state['destLoc'])
        Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
        return Cost['sourcefly'] + (Cost['transport'] * TRANSPORT_REDUCE) + Cost['destfly'], Cost['destfly'] + Cost[
            'sourcefly']

    def cost_greedy(self, stopsList, state, lines=None, ignore_wait=False):
        num = self.Ucount
        soufli = 500
        mins = []
        time2wait = []
        choiced = ''
        StopsDistances = 50
        for stp in stopsList:
            Lin = rf.findStopLine(int(stp))
            t, route = rf.find(int(stp), state['destLoc'])
            Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
            sumTime = Cost['destfly'] + Cost['sourcefly'] + (Cost['transport'] * TRANSPORT_REDUCE)  # + time2wait[-1]
            if not ignore_wait:
                time2wait.append(estimate_wait_time(state['UAV'], lines, state, stp + Cost['direction']))
                sumTime += time2wait[-1]
            if Cost['destfly'] + Cost['sourcefly'] > state['MAX_FLY_DIST'] / 2 or \
                    Cost['destfly'] + Cost['sourcefly'] > state['UAV_battery']:
                sumTime += 500000
            mins.append(sumTime)

        choiced = stopsList[np.argmin(mins)]
        t, route = rf.find(int(choiced), state['destLoc'])
        Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
        return Cost['sourcefly'] + (Cost['transport'] * TRANSPORT_REDUCE) + Cost['destfly'], Cost['destfly'] + Cost[
            'sourcefly']

    def greedy(self, stopsList, state, lines=None):
        num = self.Ucount
        soufli = 500
        mins = []
        time2wait = []
        choiced = ''
        StopsDistances = 50
        for stp in stopsList:
            Lin = rf.findStopLine(int(stp))
            t, route = rf.find(int(stp), state['destLoc'])
            Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
            time2wait.append(estimate_wait_time(state['UAV'], lines, state, stp + Cost['direction']))
            sumTime = Cost['destfly'] + Cost['sourcefly'] + (Cost['transport'] * TRANSPORT_REDUCE) + time2wait[-1]
            if Cost['destfly'] + Cost['sourcefly'] > state['MAX_FLY_DIST'] / 2 or \
                    Cost['destfly'] + Cost['sourcefly'] > state['UAV_battery']:
                sumTime += 500000
            mins.append(sumTime)

        choiced = stopsList[np.argmin(mins)]
        wait_time = time2wait[np.argmin(mins)]
        return choiced, wait_time

    def decide(self, stopsList, state, Lines) -> str:  # out -> str( stopID) Like "78"
        efected_lines = []
        wait_time = None
        StopsDistances = 50
        for stp in stopsList:
            line_name = rf.findStopLine(int(stp))
            efected_lines.append(line_name)
        random_value = np.random.rand()
        if any([random_value <= self.ann[i].explore_rate for i in efected_lines]):
            max_explore = [self.ann[i].explore_rate for i in efected_lines]
            if DEEP_INITIALIZE_APPROACH == 'RANDOM':
                # remove wrong line stations
                index = 0
                failure_list = []
                for stp in stopsList:
                    t, route = rf.find(int(stp), state['destLoc'])
                    Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
                    if Cost['transport'] < MIN_TRANSPORT_PATH:
                        failure_list.append(index)
                    index += 1
                failure_list.reverse()
                for i in failure_list:
                    max_explore.pop(i)
                    stopsList.pop(i)
                max_explore = max_explore / np.sum(max_explore)
                choiced = np.random.choice(stopsList, p=max_explore)
                _, route = rf.find(int(choiced), state['destLoc'])
                Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
                wait_time = estimate_wait_time(state['UAV'], Lines, state, choiced + Cost['direction'])
            elif DEEP_INITIALIZE_APPROACH == 'GREEDY':
                choiced, wait_time = self.greedy(stopsList, state, lines=Lines)
            else:
                print("ERROR: DEEP_INITIALIZE_APPROACH is not defined")
                exit(1)

            bestInpnodes, stopInDirection = self.__inpCreate(
                choiced, state, Lines)
        else:
            choiced = ''
            choicedStopInDirection = ''
            maxRew = 900000
            # bestInpnodes = np.zeros(self.inpNodeCount)
            print(
                " ***********       **********          Deep Q-Network decideing ******** ******")
            for stp in stopsList:
                inpnodes, stopInDirection = self.__inpCreate(stp, state, Lines)
                line_name = rf.findStopLine(int(stp))
                x_predict = np.reshape(inpnodes, (1, *self.ann[line_name].inpNodeCount))
                pval = self.ann[line_name].predict(x_predict)
                print('pval is: ', pval)

                t, route = rf.find(int(stp), state['destLoc'])
                Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
                sumTime = Cost['destfly'] + Cost['sourcefly'] + (Cost['transport'] * TRANSPORT_REDUCE) + pval[0][0]
                if Cost['destfly'] + Cost['sourcefly'] > state['MAX_FLY_DIST'] / 2 or \
                        Cost['destfly'] + Cost['sourcefly'] > state['UAV_battery']:
                    sumTime += 10000

                # * (1 / (state['BStop'][stopInDirection]['passengers'] + 1))
                if sumTime < maxRew:
                    maxRew = sumTime
                    choiced = stp
                    wait_time = pval[0][0]
                    choicedStopInDirection = stopInDirection
                    bestInpnodes = inpnodes
        detected_line_name = rf.findStopLine(int(choiced))
        memid = int(np.random.rand() * 9000 + 1000)
        self.temporalMemory.append(
            {'id': memid, 'value': bestInpnodes, 'line': detected_line_name})
        return memid, choiced, self.ann[detected_line_name].explore_rate, wait_time

    def __inpCreate(self, stp, state, Lines):

        t, route = rf.find(int(stp), state['destLoc'])
        if int(stp) < int(route[1]):
            stopInDirection = str(stp) + '_0'
        else:
            stopInDirection = str(stp) + '_1'

        current_line = rf.findStopLine(int(stp))
        inpnodes = Lines[current_line].create_deep_input()
        first_station_location = Lines[current_line].stations[stopInDirection].loc
        inpnodes.append([first_station_location.x, first_station_location.y])
        inpnodes = np.array(inpnodes)
        # inpnodes = np.subtract(inpnodes, self.normalizerValues)
        # inpnodes = np.divide(inpnodes, self.normalizerValues)
        return np.array(inpnodes, np.float16), 0

    def saveData(self, memid, wholestep):
        for i in self.temporalMemory:
            if i['id'] == memid:
                self.ann[i['line']].MemoryX.append(deepcopy(i['value']))
                self.ann[i['line']].MemoryY.append(wholestep)
                self.temporalMemory.remove(i)
                break
        self.ann[i['line']].train()



def estimate_wait_time(uav: UAV, lines: dict[str, Line_class], net_state: dict, goal_station: str) -> int:
    before_stations = net_state['BStop'][goal_station]['beforeStops']  # list of before stops ['1_0', '2_0', ...]
    goal_line = rf.findStopLine(int(goal_station[:-2]))
    goal_line = lines[goal_line]
    time_to_reach_goal = uav.loc.distance(goal_line.stations[goal_station].loc)
    drones_in_way_count = 0
    effected_station = goal_station
    for station in before_stations:
        drones_in_way_count += goal_line.stations[effected_station].get_coming_count() + \
                               goal_line.stations[effected_station].get_passengers_count()
        bus_list = station_passed_buss(station, goal_line)
        for bus in bus_list:
            if bus.get_passengers_count() + drones_in_way_count < net_state['BusMaxCap']:
                if time_to_reach_goal < bus.loc.distance(goal_line.stations[goal_station].loc):
                    return bus.loc.distance(goal_line.stations[goal_station].loc)
                drones_in_way_count -= (net_state['BusMaxCap'] - bus.get_passengers_count())
                if drones_in_way_count < 0:
                    drones_in_way_count = 0
        effected_station = station
    return 150000


def station_passed_buss(station: str, line: Line_class) -> list[Bus]:
    passed_buss = []
    for bus in line.busList:
        if station == line.busList[bus].lastStop:
            passed_buss.append(line.busList[bus])
    return passed_buss
