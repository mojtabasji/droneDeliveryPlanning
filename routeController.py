from collections import deque
from copy import deepcopy
import random
from re import L
from time import sleep
from point import point
import routeFinding as rf
from keras import models
from keras.models import Sequential, load_model
from keras.layers import Dense, Dropout, Flatten
from tensorflow.keras.optimizers import Adam
import numpy as np
import random
import re
import os


class ANN:
    def __init__(self, line_num=0, line_count=1, UAVCount=0, inpNodeCount=1):
        self.lineNum = line_num
        self.lineCount = line_count
        self.UAVCount = UAVCount
        self.explore_rate = 1.0
        self.explore_min = 0.03
        self.explore_decay = 0.995
        self.inpNodeCount = inpNodeCount
        self.MemoryX = deque(maxlen=1000)
        self.MemoryY = deque(maxlen=1000)
        self.model = self.__build_model()
        self.outGama = 2
        self.sample_batch_size = UAVCount / line_count

    def __build_model(self):
        mdl = Sequential()
        mdl.add(Dense(300, input_shape=self.inpNodeCount, activation='relu'))
        mdl.add(Dense(1000, activation='linear'))
        mdl.add(Flatten())
        mdl.add(Dense(1, activation='softplus'))  # linear softmax  sigmoid
        mdl.compile(loss='mse', optimizer=Adam(learning_rate=0.002),
                    metrics=['mae', 'mse'])  # optimizer='rmsprop'
        return mdl

    def predict(self, inp):
        return self.model.predict(inp)

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
    def __init__(self, UAVCount, LoadModel=False, lines={}) -> None:
        self.weight_backup = "U_Brine_weight.h5"
        self.temporalMemory = []  # deque(UAVCount)
        self.ann = {}
        line_count = len(lines)
        line_attubutes = list(lines.items())
        for i in range(line_count):
            input_nodes = lines[line_attubutes[i][0]].get_bus_station_count(
            ) + (lines[line_attubutes[i][0]].get_bus_count() * 2) + 1
            self.ann[line_attubutes[i][0]] = ANN(line_num=i, line_count=line_count, UAVCount=UAVCount,
                                              inpNodeCount=(int(input_nodes), int(2)))
        self.outGama = 2
        self.Ucount = UAVCount

    def Cost_deep(self, stopsList, state, Lines):
        efected_lines = []
        for stp in stopsList:
            line_name = rf.findStopLine(int(stp))
            efected_lines.append(line_name)
        random_value = np.random.rand()
        if any([random_value <= self.ann[i].explore_rate for i in efected_lines]):
            choiced = np.random.choice(stopsList)
        else:
            choiced = ''
            maxRew = 900000
            for stp in stopsList:
                inpnodes, stopInDirection = self.__inpCreate(stp, state, Lines)
                line_name = rf.findStopLine(int(stp))
                X_predict = np.reshape(inpnodes, (1, *self.ann[line_name].inpNodeCount))
                pval = self.ann[line_name].predict(X_predict)
                # * (1 / (state['BStop'][stopInDirection]['passengers'] + 1))
                if pval[0][0] < maxRew:
                    maxRew = pval
                    choiced = stp
        t, route = rf.find(int(choiced), state['destLoc'])
        Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
        return Cost['sourcefly'] + Cost['transport'] + Cost['destfly'], Cost['destfly'] + Cost['sourcefly']

    def decide(self, stopsList, state, Lines):  # out -> str( stopID) Like "78"
        efected_lines = []
        for stp in stopsList:
            line_name = rf.findStopLine(int(stp))
            efected_lines.append(line_name)
        random_value = np.random.rand()
        if any([random_value <= self.ann[i].explore_rate for i in efected_lines]):
            max_explore = [self.ann[i].explore_rate for i in efected_lines]
            max_explore = max_explore / np.sum(max_explore)
            choiced = np.random.choice(stopsList, p=max_explore)
            # choiced = np.random.choice(stopsList)
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
                # * (1 / (state['BStop'][stopInDirection]['passengers'] + 1))
                if pval[0][0] < maxRew:
                    maxRew = pval
                    choiced = stp
                    choicedStopInDirection = stopInDirection
                    bestInpnodes = inpnodes
        detected_line_name = rf.findStopLine(int(choiced))
        memid = int(np.random.rand() * 9000 + 1000)
        self.temporalMemory.append(
            {'id': memid, 'value': bestInpnodes, 'line': detected_line_name})
        return memid, choiced, self.ann[detected_line_name].explore_rate

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

    def saveData(self, memid, Flystep, wholestep):
        for i in self.temporalMemory:
            if i['id'] == memid:
                self.ann[i['line']].MemoryX.append(deepcopy(i['value']))
                # ((1/Flystep) * self.outGama) + (1/wholestep))
                self.ann[i['line']].MemoryY.append(wholestep)
                self.temporalMemory.remove(i)
                break
        self.ann[i['line']].train()


def reachFreeSpaceLoc(stopInDirection,
                      net):  # return the location (bus is in which bs) of the stop that the bus can reach the free space
    freeSpace = net['BStop'][stopInDirection]['freeSpace']
    wanters = net['BStop'][stopInDirection]['coming'] + \
        net['BStop'][stopInDirection]['goingToBefore'] + 1
    latest = freeSpace - wanters
    if latest <= 0:
        return 3000  # maximum Value

    latest = (latest / net['BusMaxCap']) + 1
    BussList = net['BussList']
    ComingBuss = net['BStop'][stopInDirection]['comingBuss']
    BBSs = deepcopy(net['BStop'][stopInDirection]['beforeStops'])
    BBSs.reverse()
    latest_counter = 0
    for i in BBSs:
        for j in ComingBuss:
            if BussList[j].lastStop == i:
                latest_counter += 1
            if latest_counter == latest:
                return abs(int(stopInDirection.split('_')[0]) - int(i.split('_')[0]))
    return 3000
