import xlwt


class StoreMore:
    def __init__(self) -> None:
        self.keyVal = {}
        self.DepotsUseCount = {}
        self.learningRate = []
        self.existPathCount = []

    def setKeyVal(self, key, val):
        self.keyVal[key] = val
    
    def increaseKeyVal(self, key, val):
        if key in self.keyVal:
            self.keyVal[key] += val
        else :
            self.keyVal[key] = val

    def increaseDepotUsed(self, depot_id):
        if depot_id in self.DepotsUseCount:
            self.DepotsUseCount[depot_id] += 1
        else:
            self.DepotsUseCount[depot_id] = 1
    
    def storedecideParams(self, lr, pathCount):
        self.learningRate.append(lr)
        self.existPathCount.append(pathCount)

    
    def Save2file(self, nameAddation):
        workbook = xlwt.Workbook()
        sheet = workbook.add_sheet("sheet1")
        header_font = xlwt.Font()
        header_font.name = 'Arial'
        header_font.bold = True
        header_style = xlwt.XFStyle()
        header_style.font = header_font
        sheet.write(0, 0, 'Key', header_style)
        sheet.write(0, 1, 'Value', header_style)

        sheet.write(0, 3, 'LR', header_style)
        sheet.write(0, 4, 'PathCnt', header_style)

        rowCounter = 1
        for d in self.DepotsUseCount:
            sheet.write(rowCounter, 0, d)
            sheet.write(rowCounter, 1, self.DepotsUseCount[d])
            rowCounter += 1
        
        for i in self.keyVal:
            sheet.write(rowCounter, 0, i)
            sheet.write(rowCounter, 1, self.keyVal[i])
            rowCounter += 1

        for i in range(len(self.learningRate)):
            sheet.write(i+1,3, self.learningRate[i])
            sheet.write(i+1,4, self.existPathCount[i])
        
        workbook.save('moreData'+ str(nameAddation) +'.xls')
