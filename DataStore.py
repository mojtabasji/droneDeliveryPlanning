import xlwt


class StoreData:

    def __init__(self, UAV_count, T_steps, line_count) -> None:
        self.__memory = {'timing':[],'status':[]}
        self.__storeOptions = [0 for i in range(UAV_count)]
        self.__UTimingRow = [None for i in range(UAV_count)]
        self.__stepCounter = [0 for i in range(UAV_count) ]
        self.__Reachs = 1
        self.__UAVReach = [0 for i in range(UAV_count)]
        self.__routeLine = [None for i in range(UAV_count)]
        self.__ConditionCounter = 0

    def storeLineCondition(self, status):
        self.__memory['status'].append(status)

    def storeTiming(self, rowIndex, UId): # (rowNumber, Uid, )
        if rowIndex >= len(self.__memory['timing']):
            for i in range(rowIndex - len(self.__memory['timing']) + 1):
                self.__memory['timing'].append([None, None, None ,None, None, None, None, None])
        if self.__storeOptions[UId] == 4:
            self.__memory['timing'][rowIndex][4] = self.__Reachs
            self.__UAVReach[UId] += 1
            self.__memory['timing'][rowIndex][5] = self.__UAVReach[UId]
            self.__memory['timing'][rowIndex][6] = self.__routeLine[UId]
            self.__memory['timing'][rowIndex][7] = UId
        else:
            if self.__memory['timing'][rowIndex][self.__storeOptions[UId]] == None:
                self.__memory['timing'][rowIndex][self.__storeOptions[UId]] = self.__stepCounter[UId]
            else:
                self.__memory['timing'][rowIndex][self.__storeOptions[UId]] += self.__stepCounter[UId]
            self.__stepCounter[UId] = 0
        self.__storeOptions[UId] = (self.__storeOptions[UId] + 1) % 5

    def incrementStep(self, UId):
        self.__stepCounter[UId] += 1

    def increseReachs(self):
        self.__Reachs += 1

    def resetStoreOption(self, Uid, opt):
        if opt == "transport":
            self.__storeOptions[Uid] = 2
        elif opt == "wait":
            self.__storeOptions[Uid] = 1
        elif opt == "Source_fly":
            self.__storeOptions[Uid] = 0
            
    def resetStepsData(self, UId):
        self.__stepCounter[UId] = 0
        self.__storeOptions[UId] = 0


    def setRouteLine(self,UId, Line):
        self.__routeLine[UId] = Line

    def SaveToFile(self, nameAddation):
        pageNumber = 1
        workbook = xlwt.Workbook()
        sheet = workbook.add_sheet("page"+str(pageNumber))
        header_font = xlwt.Font()
        header_font.name = 'Arial'
        header_font.bold = True
        header_style = xlwt.XFStyle()
        header_style.font = header_font
        sheet.write(0, 0, 'sourceFly', header_style)
        sheet.write(0, 1, 'waiting', header_style)
        sheet.write(0, 2, 'transport', header_style)
        sheet.write(0, 3, 'destFly', header_style)
        sheet.write(0, 4, 'reach order', header_style)
        sheet.write(0, 5, 'task Counter', header_style)
        sheet.write(0, 6, 'usedLine', header_style)
        sheet.write(0, 7, 'UAV ID', header_style)

        spaceBitween = 10
        for i in range(int(len(self.__memory['status'][0]) / 6)):
            colindex = i * 6 + spaceBitween
            sheet.write(0,0 + colindex,'L '+str(i)+' fly2' ) #im was here to set heders for saving line status
            sheet.write(0,1 + colindex,'L '+str(i)+' fly2R' ) 
            sheet.write(0,2 + colindex,'L '+str(i)+' wait' ) 
            sheet.write(0,3 + colindex,'L '+str(i)+' wait R' ) 
            sheet.write(0,4 + colindex,'L '+str(i)+' onLine' ) 
            sheet.write(0,5 + colindex,'L '+str(i)+' onLineR' ) 
        
        rowId =1
        for content in self.__memory['timing']:
            sheet.write(rowId, 0, content[0])
            sheet.write(rowId, 1, content[1])
            sheet.write(rowId, 2, content[2])
            sheet.write(rowId, 3, content[3])
            sheet.write(rowId, 4, content[4])
            sheet.write(rowId, 5, content[5])
            sheet.write(rowId, 6, content[6])
            sheet.write(rowId, 7, content[7])
            rowId += 1

        rowId = 1
        for content in self.__memory['status']:
            itemCol = spaceBitween
            if rowId < 65000:
                for contentItem in content:
                    sheet.write(rowId,itemCol,contentItem)
                    itemCol +=1
            else:
                pageNumber +=1
                sheet = workbook.add_sheet("page"+str(pageNumber))
                rowId = 0
            rowId +=1

        workbook.save('operations_'+ str(nameAddation) +'.xls')