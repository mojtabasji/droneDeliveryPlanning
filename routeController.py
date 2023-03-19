from collections import deque
from copy import deepcopy
import random
from re import L
from time import sleep
from point import point
import routeFinding as rf
from keras import models
from keras.models import Sequential, load_model
from keras.layers import Dense, Dropout
from tensorflow.keras.optimizers import Adam
import numpy as np
import random
import re
import os


class brain:
    def __init__(self,UAVCount, LoadModel = False ) -> None:
        self.weight_backup = "U_Brine_weight.h5"
        self.exploration_rate = 1.0
        self.exploration_min = 0.03
        self.exploration_decay = 0.995
        if LoadModel:
            self.model = self.__loadModel( UAVCount )
            self.exploration_rate = 0.3
        else:
            self.model = self.__build_model()
        self.temporalMemory =  [] # deque(UAVCount)
        self.MemoryX = deque(maxlen= UAVCount * 10)
        self.MemoryY = deque(maxlen= UAVCount * 10)
        self.outGama = 2
        self.Ucount = UAVCount
        self.sample_batch_size = UAVCount
        self.normalizerValues = [3000, 3000, 3000, UAVCount, 15, 15, UAVCount, 1, 1 - (1/UAVCount), UAVCount]


    def saveModel(self, nameAdd):
        self.model.save(f'my_model_{str(nameAdd)}.h5')

    def __loadModel(self, nameAdd):
        weight_file = f'my_model_{str(nameAdd)}.h5'
        print("Loading model from file: ", weight_file)
        return models.load_model(weight_file)
        
    
    def __build_model(self):
        mdl = Sequential()
        mdl.add(Dense(300, input_dim= (10), activation='relu'))
        mdl.add(Dense(1000, activation='linear'))
        mdl.add(Dense(1, activation='linear')) #  linear softmax  sigmoid
        mdl.compile(loss='mse', optimizer=Adam(lr=0.002), metrics=['mae', 'mse']) #   optimizer='rmsprop'
        if os.path.isfile(self.weight_backup):
            mdl.load_weights(self.weight_backup)
        return mdl

    def Cost_Timing(self, stopsList, state, Lines):
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
            sumTime = Cost['destfly'] + Cost['sourcefly'] + Cost['transport'] + (reachFreeSpaceLoc(str(stp)+Cost['direction'], state) *  StopsDistances)
            mins.append(sumTime)
        
        choiced = stopsList[np.argmin(mins)]
        t, route = rf.find(int(choiced), state['destLoc'])
        Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
        return Cost['sourcefly'] + Cost['transport'] + Cost['destfly'], Cost['destfly'] + Cost['sourcefly']

    def Cost_fairness(self, stopsList, state, Lines):
        num = self.Ucount
        soufli = 500
        selLin = ""
        choiced = ''
        for stp in stopsList:
            Lin = rf.findStopLine(int(stp))
            t, route = rf.find(int(stp), state['destLoc'])
            Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
            if (state['BStop'][str(stp) + Cost['direction']]['passengers'] + state['BStop'][str(stp) + Cost['direction']]['coming'] < num and selLin != Lin) or (selLin == Lin and Cost['sourcefly'] < soufli):
                num = state['BStop'][str(stp) + Cost['direction']]['passengers'] + state['BStop'][str(stp) + Cost['direction']]['coming']
                selLin = Lin
                soufli = Cost['sourcefly']
                choiced = stp
        t, route = rf.find(int(choiced), state['destLoc'])
        Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
        return Cost['sourcefly'] + Cost['transport'] + Cost['destfly'], Cost['destfly'] + Cost['sourcefly']

    def Cost_deep(self, stopsList, state, Lines):
        if np.random.rand() <= self.exploration_rate:
            return self.Cost_Timing(stopsList, state, Lines)
        else:
            choiced = ''
            maxRew = 900000
            for stp in stopsList:
                inpnodes, stopInDirection = self.__inpCreate(stp, state, Lines)
                pval = self.model.predict(np.reshape(inpnodes,(1,10)))
                if pval[0][0] < maxRew: #* (1 / (state['BStop'][stopInDirection]['passengers'] + 1))
                    maxRew = pval
                    choiced = stp
            t, route = rf.find(int(choiced), state['destLoc'])
            Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
            return Cost['sourcefly'] + Cost['transport'] + Cost['destfly'], Cost['destfly'] + Cost['sourcefly']

    def TimingDecide(self, stopsList, state, Lines):
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
            time2wait.append(reachFreeSpaceLoc(str(stp)+Cost['direction'], state) *  StopsDistances)
            sumTime = Cost['destfly'] + Cost['sourcefly'] + Cost['transport'] + time2wait[-1]
            mins.append(sumTime)
        
        choiced = stopsList[np.argmin(mins)]
        return 1, choiced, time2wait[np.argmin(mins)]

    def fairnessDecide(self, stopsList, state, Lines):
        num = self.Ucount
        soufli = 500
        selLin = ""
        choiced = ''
        for stp in stopsList:
            Lin = rf.findStopLine(int(stp))
            t, route = rf.find(int(stp), state['destLoc'])
            Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
            if (state['BStop'][str(stp) + Cost['direction']]['passengers'] + state['BStop'][str(stp) + Cost['direction']]['coming'] < num and selLin != Lin) or (selLin == Lin and Cost['sourcefly'] < soufli):
                num = state['BStop'][str(stp) + Cost['direction']]['passengers'] + state['BStop'][str(stp) + Cost['direction']]['coming']
                selLin = Lin
                soufli = Cost['sourcefly']
                choiced = stp
        return 1, choiced

    def greedlyDecide(self, stopsList, state, Lines): # out -> str( stopID):
        stopDist = []
        for stp in stopsList:
            t, route = rf.find(int(stp), state['destLoc'])
            Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
            stopDist.append(Cost['sourcefly'])
        choiced = stopsList[np.argmin(stopDist)]
        bestInpnodes = self.__inpCreate(choiced, state, Lines)
        memid = int( np.random.rand() * 9000 + 1000)
        self.temporalMemory.append({'id': memid, 'value': bestInpnodes})
        return memid, choiced

        
    def algorithm(self, stopsList, state, Lines):
        LinRate = 1
        chprob = 0
        for stp in stopsList:
            Lin = rf.findStopLine(int(stp))
            t, route = rf.find(int(stp), state['destLoc'])
            Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
            dstnt = Cost['sourcefly'] + Cost['destfly']
            tmpchprob = pow(np.random.rand() ,5) * (1 /dstnt)
            if Lines[Lin]['busyRate' + Cost['direction'] ] < LinRate:
                choiced = stp
                LinRate = Lines[Lin]['busyRate' + Cost['direction']]
                chprob = tmpchprob
            elif Lines[Lin]['busyRate' + Cost['direction']] == LinRate and tmpchprob > chprob:
                choiced = stp
                chprob = tmpchprob
        return 1, choiced

    def decide(self, stopsList, state, Lines): # out -> str( stopID) Like "78"
        if np.random.rand() <= self.exploration_rate:
            nothing, choiced, waitingTime = self.TimingDecide(stopsList, state, Lines)
            '''
            LinRate = 1
            chprob = 0
            for stp in stopsList:
                Lin = rf.findStopLine(int(stp))
                t, route = rf.find(int(stp), state['destLoc'])
                Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
                dstnt = Cost['sourcefly'] + Cost['destfly']
                tmpchprob = pow(np.random.rand() ,5) * (1 /dstnt)
                if Lines[Lin]['busyRate' + Cost['direction'] ] < LinRate:
                    choiced = stp
                    LinRate = Lines[Lin]['busyRate' + Cost['direction']]
                    chprob = tmpchprob
                elif Lines[Lin]['busyRate' + Cost['direction']] == LinRate and tmpchprob > chprob:
                    choiced = stp
                    chprob = tmpchprob '''
            bestInpnodes ,stopInDirection = self.__inpCreate(choiced, state, Lines)
        else:
            choiced = ''
            choicedStopInDirection = ''
            maxRew = 900000
            bestInpnodes = np.zeros(10)
            print(" ***********       **********          Deep Q-Network decideing ******** ******")
            for stp in stopsList:
                inpnodes, stopInDirection = self.__inpCreate(stp, state, Lines)
                #inpnodes = np.array(inpnodes)
                #print('shape: ', np.shape(inpnodes))
                #inpnodes = np.reshape(inpnodes, (1,10))
                #print('inpnodes :', inpnodes)
                pval = self.model.predict(np.reshape(inpnodes,(1,10)))
                print('pval is: ', pval)
                if pval[0][0] < maxRew: #* (1 / (state['BStop'][stopInDirection]['passengers'] + 1))
                    maxRew = pval
                    choiced = stp
                    choicedStopInDirection = stopInDirection
                    bestInpnodes = inpnodes
            
            waitingTime = reachFreeSpaceLoc( choicedStopInDirection, state) * 50 # 50 is the distance between two stops
            
        memid = int( np.random.rand() * 9000 + 1000)
        self.temporalMemory.append({'id': memid, 'value': bestInpnodes})
        return memid, choiced, waitingTime, self.exploration_rate


    def __inpCreate(self, stp, state, Lines):
        #state = deepcopy(stt)
        t, route = rf.find(int(stp), state['destLoc'])
        Cost = rf.Costing(state['curLoc'], route, state['destLoc'])
        if int(stp) < int (route[1]):
            stopInDirection = str(stp) + '_0'
        else:
            stopInDirection = str(stp) + '_1'
        '''
        inpnodes = [state['curLoc'].x, state['curLoc'].y, state['destLoc'].x, state['destLoc'].y,
                    state['BStop'][stopInDirection][0], state['BStop'][stopInDirection][1], state['BStop'][stopInDirection][2].x, state['BStop'][stopInDirection][2].y] '''

        r = re.compile("([a-zA-Z_]+)([0-9]+)")
        Lin = rf.findStopLine(int(stp))

        inpnodes = []
        inpnodes.append(Cost['sourcefly'])
        inpnodes.append(Cost['destfly'])
        inpnodes.append(Cost['transport'])
        inpnodes.append(state['BStop'][stopInDirection]['passengers'])
        inpnodes.append(state['BStop'][stopInDirection]['freeSpace'])
        inpnodes.append(state['BStop'][stopInDirection]['coming'])
        inpnodes.append(state['BStop'][stopInDirection]['goingToBefore'])
        #inpnodes.append(Lines[Lin]['busyRate' + Cost['direction']])
        inpnodes.append(len(route))
        inpnodes.append(int(r.match(Lin).group(2)))
        inpnodes.append(int(Cost['direction'][1]))

        inpnodes = np.array(inpnodes)
        #inpnodes = np.subtract(inpnodes, self.normalizerValues)
        #inpnodes = np.divide(inpnodes, self.normalizerValues)
        return inpnodes.astype(np.float16) , stopInDirection

    def saveData(self, memid, Flystep, wholestep):
        for i in self.temporalMemory:
            if i['id'] == memid:
                self.MemoryX.append( deepcopy(i['value']))
                self.MemoryY.append(wholestep)#((1/Flystep) * self.outGama) + (1/wholestep))
                self.temporalMemory.remove(i)
                break
        
        if len(self.MemoryX) < self.sample_batch_size * 2 or self.exploration_rate <= self.exploration_min :
            return
        self.doTrain = 4
        batch_indexs = random.sample(range(len(self.MemoryX)), self.sample_batch_size)
        Xtrain = self.__subSet(self.MemoryX, batch_indexs)
        Ytrain = self.__subSet(self.MemoryY, batch_indexs)
        
        try:
            Xtrain = np.array(Xtrain, dtype= np.float16)
            Ytrain = np.array(Ytrain, dtype= np.float16)
            Xtrain = np.reshape(Xtrain, (self.sample_batch_size, 10))
            Ytrain = np.reshape(Ytrain, (self.sample_batch_size, 1))
        except:
            print('Xtrain: ',Xtrain)
        #print('Ytrain: ', Ytrain)
        self.model.fit(Xtrain, Ytrain, epochs= 1,  validation_split=0.1)
        #self.model.save(self.weight_backup)
        if self.exploration_rate > self.exploration_min:
            self.exploration_rate *= self.exploration_decay

    def __subSet(self, set, setindexs):
        newlist = []
        for i in setindexs:
            newlist.append(set[i])
        return newlist



def reachFreeSpaceLoc(stopInDirection, net): # return the location (bus is in which bs) of the stop that the bus can reach the free space
    freeSpace = net['BStop'][stopInDirection]['freeSpace']
    wanters = net['BStop'][stopInDirection]['coming'] + net['BStop'][stopInDirection]['goingToBefore'] + 1
    latest = freeSpace - wanters
    if latest <= 0:
        return 3000  # maximum Value

    latest = (latest / net['BusMaxCap'] ) + 1
    BussList = net['BussList']
    ComingBuss = net['BStop'][stopInDirection]['comingBuss']
    BBSs = deepcopy( net['BStop'][stopInDirection]['beforeStops'])
    BBSs.reverse()
    latest_counter = 0
    for i in BBSs:
        for j in ComingBuss:
            if BussList[j].lastStop == i:
                latest_counter += 1
            if latest_counter == latest:
                return abs(int(stopInDirection.split('_')[0]) - int(i.split('_')[0]))
    return 3000
